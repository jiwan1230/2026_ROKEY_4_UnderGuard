# herding_controller/herding_controller/herding_node.py
"""rclpy adapter: the only file in this package that imports ROS. Wraps HerdingCore.

Every function that does not need a live ROS node (config→dict mapping, JSON
payload construction, belief-grid quantization) is kept as a plain function so
it can be unit tested without spinning up rclpy machinery.
"""
import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from herding_controller.herding_core import HerdingConfig, HerdingCore, HerdingOutput, Observation
from herding_controller.state_machine import FSMState

# Defaults mirror config/herding_params.yaml's ros__parameters block. Every
# field of HerdingConfig (see herding_core.py) must appear here (or have a
# dataclass default) or HerdingConfig(**values) raises TypeError at startup.
_PARAM_DEFAULTS = {
    "frame_id": "map",
    "control_rate_hz": 5.0,
    # --- Capture Zone ---
    "capture_zone_x_m": 3.0,
    "capture_zone_y_m": 3.0,
    "capture_radius_m": 0.5,
    "capture_hold_sec": 3.0,
    # --- Grid ---
    "grid_resolution_m": 0.25,
    "grid_width_cells": 40,
    "grid_height_cells": 40,
    "grid_origin_x_m": 0.0,
    "grid_origin_y_m": 0.0,
    # --- Target Estimator (KF) ---
    "kf_process_noise": 0.1,
    "kf_measurement_noise": 0.05,
    "occlusion_timeout_sec": 3.0,
    # --- Escape Model (Markov) ---
    "markov_wall_follow_p": 0.70,
    "markov_wall_hug_p": 0.20,
    "markov_center_p": 0.10,
    "momentum_weight": 0.4,
    "robot_repulsion_weight": 1.5,
    "wall_detect_radius_cells": 1,
    "escape_route_top_k": 3,
    "escape_concentration_threshold": 0.5,
    # --- Herding Control ---
    "drive_distance_m": 0.8,
    "flee_reaction_distance_m": 1.0,
    "panic_distance_m": 0.35,
    "alignment_threshold": 0.7,
    "drive_distance_ease_factor": 1.3,
    "block_lookahead_m": 1.2,
    # --- Role Assignment ---
    "role_swap_margin": 0.5,
    "role_swap_cooldown_sec": 2.0,
    "min_robot_separation_m": 0.6,
    "role_cost_turn_weight": 0.3,
    # --- Occlusion Grid ---
    "diffusion_rate": 0.2,
    "decay_factor": 0.9,
}


def _load_config(node: Node) -> HerdingConfig:
    """Declare every HerdingConfig field as a ROS parameter and build the config."""
    for name, default in _PARAM_DEFAULTS.items():
        node.declare_parameter(name, default)
    values = {name: node.get_parameter(name).value for name in _PARAM_DEFAULTS}
    return HerdingConfig(**values)


def _serialize_state(output: HerdingOutput) -> dict:
    """Build a JSON-safe dict from a HerdingOutput (no numpy arrays, no enums)."""
    return {
        "fsm_state": output.fsm_state.name,
        "roles": {"driver": output.driver_id, "blocker": output.blocker_id},
        "target_pos": output.target_position.tolist(),
        "target_vel": output.target_velocity.tolist(),
        "escape_prob_top3": [route.tolist() for route in output.escape_top3],
        "latency_ms": output.latency_ms,
        "panic": output.panic,
        "role_swapped": output.role_swapped,
    }


def _belief_to_flat_int8(belief: np.ndarray) -> list:
    """Quantize a [0, 1] belief grid to a flat, row-major list of int8 in [0, 100].

    Row-major (C order) matches how `_on_map` reshapes an incoming
    OccupancyGrid.data back into (height, width): `np.array(data).reshape(h, w)`
    is the exact inverse of `.flatten(order="C")` here.
    """
    scaled = np.clip(belief, 0.0, 1.0) * 100.0
    return scaled.astype(np.int8).flatten(order="C").tolist()


class HerdingNode(Node):
    """Subscribes to poses/map, runs HerdingCore, publishes goals and telemetry."""

    def __init__(self) -> None:
        super().__init__("herding_controller")
        self.config = _load_config(self)
        self.core = HerdingCore(self.config)

        self._target_pos = None
        self._robot1_pos = np.zeros(2)
        self._robot2_pos = np.zeros(2)
        self._robot1_heading = np.array([1.0, 0.0])
        self._robot2_heading = np.array([1.0, 0.0])
        self._occupancy = None
        self._sim_time = 0.0

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, "~/target_pose", self._on_target_pose, 10)
        self.create_subscription(PoseStamped, "~/robot1_pose", self._on_robot1_pose, 10)
        self.create_subscription(PoseStamped, "~/robot2_pose", self._on_robot2_pose, 10)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)

        self.robot1_goal_pub = self.create_publisher(PoseStamped, "~/robot1_goal", 10)
        self.robot2_goal_pub = self.create_publisher(PoseStamped, "~/robot2_goal", 10)
        self.state_pub = self.create_publisher(String, "~/herding_state", 10)
        self.escape_prob_pub = self.create_publisher(OccupancyGrid, "~/escape_probability", 10)
        self.capture_result_pub = self.create_publisher(Bool, "~/capture_result", 10)

        self.create_timer(1.0 / self.config.control_rate_hz, self._on_timer)

    def _on_target_pose(self, msg: PoseStamped) -> None:
        self._target_pos = np.array([msg.pose.position.x, msg.pose.position.y])

    def _on_robot1_pose(self, msg: PoseStamped) -> None:
        self._robot1_pos = np.array([msg.pose.position.x, msg.pose.position.y])

    def _on_robot2_pose(self, msg: PoseStamped) -> None:
        self._robot2_pos = np.array([msg.pose.position.x, msg.pose.position.y])

    def _on_map(self, msg: OccupancyGrid) -> None:
        # msg.data always has exactly height*width entries per the OccupancyGrid
        # spec, so this reshape can't fail even if the incoming map's extent
        # differs from our configured grid_width_cells/grid_height_cells.
        # HerdingCore.step() checks the shape against its own grid config and
        # logs + ignores a mismatched frame rather than crashing.
        self._occupancy = np.array(msg.data, dtype=int).reshape(msg.info.height, msg.info.width)

    def _on_timer(self) -> None:
        dt = 1.0 / self.config.control_rate_hz
        observation = Observation(
            target_measurement=self._target_pos, robot1_pos=self._robot1_pos, robot2_pos=self._robot2_pos,
            robot1_heading=self._robot1_heading, robot2_heading=self._robot2_heading,
            occupancy=self._occupancy, sim_time_sec=self._sim_time, dt=dt,
        )
        self._sim_time += dt
        output = self.core.step(observation)
        self._publish(output)

    def _publish(self, output: HerdingOutput) -> None:
        self.robot1_goal_pub.publish(self._to_pose(output.robot1_goal))
        self.robot2_goal_pub.publish(self._to_pose(output.robot2_goal))

        state_msg = String()
        state_msg.data = json.dumps(_serialize_state(output))
        self.state_pub.publish(state_msg)

        self.escape_prob_pub.publish(self._to_escape_grid())

        capture_msg = Bool()
        capture_msg.data = output.fsm_state == FSMState.CAPTURED
        self.capture_result_pub.publish(capture_msg)

    def _to_pose(self, point: np.ndarray) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.config.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(point[0])
        msg.pose.position.y = float(point[1])
        msg.pose.orientation.w = 1.0
        return msg

    def _to_escape_grid(self) -> OccupancyGrid:
        """Publish the LOST-recovery belief grid as the escape-probability map.

        HerdingOutput carries only the 8-direction escape distribution and its
        top-K candidate points (see escape_top3 in the state JSON) -- there is
        no full grid-shaped escape probability anywhere in HerdingCore. The
        occlusion belief grid (core.occlusion_grid.belief) is the one array
        that actually spans the whole map at grid resolution, so it is what
        gets published here: a live view of "where the target is believed to
        be" that is most informative exactly when the target is LOST.
        """
        msg = OccupancyGrid()
        msg.header.frame_id = self.config.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(self.config.grid_resolution_m)
        msg.info.width = int(self.config.grid_width_cells)
        msg.info.height = int(self.config.grid_height_cells)
        msg.info.origin.position.x = float(self.config.grid_origin_x_m)
        msg.info.origin.position.y = float(self.config.grid_origin_y_m)
        msg.info.origin.orientation.w = 1.0
        msg.data = _belief_to_flat_int8(self.core.occlusion_grid.belief)
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HerdingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
