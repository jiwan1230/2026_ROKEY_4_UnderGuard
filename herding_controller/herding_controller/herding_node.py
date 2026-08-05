# herding_controller/herding_controller/herding_node.py
"""rclpy adapter: the only file in this package that imports ROS. Wraps HerdingCore.

Every function that does not need a live ROS node (config→dict mapping, JSON
payload construction, escape-probability rasterization, quaternion-to-heading
conversion) is kept as a plain function so it can be unit tested without
spinning up rclpy machinery.
"""
import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from herding_controller.grid_map import GridMap
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


def _prob_grid_to_flat_int8(grid: np.ndarray) -> list:
    """Quantize a [0, 1] probability grid to a flat, row-major list of int8 in [0, 100].

    Row-major (C order) matches how `_on_map` reshapes an incoming
    OccupancyGrid.data back into (height, width): `np.array(data).reshape(h, w)`
    is the exact inverse of `.flatten(order="C")` here.
    """
    scaled = np.clip(grid, 0.0, 1.0) * 100.0
    return scaled.astype(np.int8).flatten(order="C").tolist()


def _rasterize_escape_probabilities(
    target_position: np.ndarray,
    directions: np.ndarray,
    probabilities: np.ndarray,
    grid_map: GridMap,
    cells_per_ray: int = 3,
) -> np.ndarray:
    """Paint the 8-direction escape distribution onto a (height, width) grid.

    For each of the 8 compass directions, a short ray of `cells_per_ray` cells
    extending from the target's own grid cell is painted with that direction's
    probability (rays stop early at the grid edge). Cells reachable from more
    than one ray keep the max, not the sum, so overlapping rays don't produce
    an out-of-range value. Returns an all-zero grid if the target position is
    off-grid (mirrors HerdingCore's own off-grid handling rather than raising).
    """
    height, width = grid_map.config.height_cells, grid_map.config.width_cells
    grid = np.zeros((height, width))
    try:
        row0, col0 = grid_map.world_to_cell(float(target_position[0]), float(target_position[1]))
    except (ValueError, TypeError):
        return grid
    for (dx, dy), probability in zip(directions, probabilities):
        for step in range(1, cells_per_ray + 1):
            row = row0 + int(round(dy * step))
            col = col0 + int(round(dx * step))
            if not grid_map.in_bounds(row, col):
                break
            grid[row, col] = max(grid[row, col], float(probability))
    return grid


def _quaternion_to_heading(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Extract planar (2D) yaw from a quaternion and return it as a unit heading vector.

    Standard quaternion-to-yaw for a rotation about the Z axis:
    yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)).
    """
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([np.cos(yaw), np.sin(yaw)])


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
        # Until a real pose arrives for each robot, robotN_pos is a meaningless
        # (0, 0) placeholder. Goal publishing is withheld (see _publish) until
        # both flip True, so the robots are never commanded toward the grid
        # origin before they have ever reported where they actually are.
        self._robot1_pose_received = False
        self._robot2_pose_received = False
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
        q = msg.pose.orientation
        self._robot1_heading = _quaternion_to_heading(q.x, q.y, q.z, q.w)
        self._robot1_pose_received = True

    def _on_robot2_pose(self, msg: PoseStamped) -> None:
        self._robot2_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        q = msg.pose.orientation
        self._robot2_heading = _quaternion_to_heading(q.x, q.y, q.z, q.w)
        self._robot2_pose_received = True

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
        # Before both robots have ever reported a real pose, robotN_pos is
        # still the (0, 0) placeholder set in __init__, and HerdingCore (in
        # IDLE/SEARCH/TRACK/CAPTURED) echoes robot_pos straight back as the
        # goal -- publishing it here would actively drive both robots toward
        # the grid origin. Withhold goal publishing until we have a real fix
        # on both.
        if self._robot1_pose_received and self._robot2_pose_received:
            self.robot1_goal_pub.publish(self._to_pose(output.robot1_goal))
            self.robot2_goal_pub.publish(self._to_pose(output.robot2_goal))

        state_msg = String()
        state_msg.data = json.dumps(_serialize_state(output))
        self.state_pub.publish(state_msg)

        self.escape_prob_pub.publish(self._to_escape_grid(output))

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

    def _to_escape_grid(self, output: HerdingOutput) -> OccupancyGrid:
        """Rasterize the current escape-direction distribution onto the map grid.

        `output.escape_directions`/`escape_probabilities` are populated by
        HerdingCore only when the escape model actually ran this cycle (i.e.
        the KF has converged and the target's cell is on-grid -- see
        HerdingCore.step()). In every other state (SEARCH/TRACK before
        convergence, LOST) there genuinely is no escape distribution, so an
        honest all-zero grid is published rather than repurposing an unrelated
        array (e.g. the occlusion belief, which represents "where the target
        might be while hidden", not "which way it's likely to flee").
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
        if output.escape_directions is not None and output.escape_probabilities is not None:
            grid = _rasterize_escape_probabilities(
                output.target_position, output.escape_directions, output.escape_probabilities,
                self.core.grid_map,
            )
        else:
            grid = np.zeros((self.config.grid_height_cells, self.config.grid_width_cells))
        msg.data = _prob_grid_to_flat_int8(grid)
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
