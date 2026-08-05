# herding_controller/herding_controller/herding_core.py
"""Pure-Python facade combining all herding sub-modules.

Hard constraint: this module, and every module it imports, must stay free of any ROS2
dependency so the whole algorithm can be exercised offline without a ROS installation.
"""
import logging
import time
from dataclasses import dataclass, field

import numpy as np

from herding_controller.escape_model import EscapeModel, EscapeModelConfig
from herding_controller.grid_map import GridConfig, GridMap
from herding_controller.herding_planner import (
    PlannerConfig,
    compute_blocking_point,
    compute_driving_point,
)
from herding_controller.occlusion_grid import OcclusionGrid, OcclusionGridConfig
from herding_controller.role_assigner import RoleAssigner, RoleAssignerConfig, resolve_separation
from herding_controller.state_machine import FSMInputs, FSMState, HerdingStateMachine
from herding_controller.target_estimator import EstimatorConfig, TargetEstimator

logger = logging.getLogger(__name__)


@dataclass
class HerdingConfig:
    """Flat mirror of config/herding_params.yaml's ros__parameters block."""
    frame_id: str
    control_rate_hz: float
    capture_zone_x_m: float
    capture_zone_y_m: float
    capture_radius_m: float
    capture_hold_sec: float
    grid_resolution_m: float
    grid_width_cells: int
    grid_height_cells: int
    kf_process_noise: float
    kf_measurement_noise: float
    occlusion_timeout_sec: float
    markov_wall_follow_p: float
    markov_wall_hug_p: float
    markov_center_p: float
    momentum_weight: float
    robot_repulsion_weight: float
    wall_detect_radius_cells: int
    escape_route_top_k: int
    escape_concentration_threshold: float
    drive_distance_m: float
    flee_reaction_distance_m: float
    panic_distance_m: float
    alignment_threshold: float
    drive_distance_ease_factor: float
    block_lookahead_m: float
    role_swap_margin: float
    role_swap_cooldown_sec: float
    min_robot_separation_m: float
    role_cost_turn_weight: float
    diffusion_rate: float
    decay_factor: float
    grid_origin_x_m: float = 0.0
    grid_origin_y_m: float = 0.0


@dataclass
class Observation:
    """One control cycle's worth of sensor input, in plain Python/numpy types."""
    target_measurement: np.ndarray | None
    robot1_pos: np.ndarray
    robot2_pos: np.ndarray
    robot1_heading: np.ndarray
    robot2_heading: np.ndarray
    occupancy: np.ndarray | None
    sim_time_sec: float
    dt: float


@dataclass
class HerdingOutput:
    """Everything herding_node.py needs to publish for one control cycle."""
    robot1_goal: np.ndarray
    robot2_goal: np.ndarray
    fsm_state: FSMState
    driver_id: int
    blocker_id: int
    target_position: np.ndarray
    target_velocity: np.ndarray
    escape_top3: list = field(default_factory=list)
    escape_directions: np.ndarray | None = None
    escape_probabilities: np.ndarray | None = None
    latency_ms: float = 0.0
    panic: bool = False
    role_swapped: bool = False


class HerdingCore:
    """Wires grid, estimator, escape model, planner, role assigner, FSM, and occlusion grid together."""

    def __init__(self, config: HerdingConfig) -> None:
        self.config = config
        self.goal_pos = np.array([config.capture_zone_x_m, config.capture_zone_y_m], dtype=float)
        self.grid_map = GridMap(GridConfig(
            resolution_m=config.grid_resolution_m, width_cells=config.grid_width_cells,
            height_cells=config.grid_height_cells, origin_x_m=config.grid_origin_x_m,
            origin_y_m=config.grid_origin_y_m,
        ))
        self.estimator = TargetEstimator(EstimatorConfig(
            process_noise=config.kf_process_noise, measurement_noise=config.kf_measurement_noise,
            occlusion_timeout_sec=config.occlusion_timeout_sec,
        ))
        self.escape_model = EscapeModel(EscapeModelConfig(
            wall_follow_p=config.markov_wall_follow_p, wall_hug_p=config.markov_wall_hug_p,
            center_p=config.markov_center_p, momentum_weight=config.momentum_weight,
            robot_repulsion_weight=config.robot_repulsion_weight,
            wall_detect_radius_cells=config.wall_detect_radius_cells,
            escape_route_top_k=config.escape_route_top_k,
        ), self.grid_map)
        self.planner_config = PlannerConfig(
            drive_distance_m=config.drive_distance_m, panic_distance_m=config.panic_distance_m,
            alignment_threshold=config.alignment_threshold,
            drive_distance_ease_factor=config.drive_distance_ease_factor,
            block_lookahead_m=config.block_lookahead_m,
        )
        self.role_assigner_config = RoleAssignerConfig(
            role_swap_margin=config.role_swap_margin, role_swap_cooldown_sec=config.role_swap_cooldown_sec,
            min_robot_separation_m=config.min_robot_separation_m,
            role_cost_turn_weight=config.role_cost_turn_weight,
        )
        self.role_assigner = RoleAssigner(self.role_assigner_config)
        self.fsm = HerdingStateMachine()
        self.occlusion_grid = OcclusionGrid(
            OcclusionGridConfig(diffusion_rate=config.diffusion_rate, decay_factor=config.decay_factor),
            self.grid_map,
        )
        self._last_known_cell: tuple[int, int] | None = None
        self._last_known_point: np.ndarray | None = None
        self._occlusion_seeded = False
        self._first_observation_seen = False
        self._roles_ever_assigned = False

    # ------------------------------------------------------------------ #
    # Grid helpers                                                        #
    # ------------------------------------------------------------------ #

    def _cell_or_none(self, position: np.ndarray) -> tuple[int, int] | None:
        """Cell index for a world point, or None if it is off-grid / non-finite."""
        try:
            return self.grid_map.world_to_cell(float(position[0]), float(position[1]))
        except (ValueError, TypeError):
            return None

    def _clamped_cell_or_none(self, position: np.ndarray) -> tuple[int, int] | None:
        """Cell index for a world point, clamped into the grid; None if non-finite."""
        grid = self.grid_map.config
        point = np.asarray(position, dtype=float)
        if point.shape != (2,) or not np.all(np.isfinite(point)):
            return None
        half = grid.resolution_m * 0.5
        x_lo = grid.origin_x_m + half
        x_hi = grid.origin_x_m + grid.width_cells * grid.resolution_m - half
        y_lo = grid.origin_y_m + half
        y_hi = grid.origin_y_m + grid.height_cells * grid.resolution_m - half
        x = float(np.clip(point[0], x_lo, x_hi))
        y = float(np.clip(point[1], y_lo, y_hi))
        return self._cell_or_none(np.array([x, y]))

    def _current_roles(self) -> tuple[int, int]:
        """The role assignment currently latched in the RoleAssigner."""
        driver_id = self.role_assigner._driver_id
        return driver_id, (2 if driver_id == 1 else 1)

    def _nominal_driving_point(self, target_pos: np.ndarray, target_vel: np.ndarray) -> np.ndarray:
        """The driving point ignoring panic-retreat, used as a robot-neutral role-assignment candidate.

        compute_driving_point() returns a *retreat* point hugging robot_pos when that robot is
        inside panic_distance_m. Feeding the role assigner a candidate computed from one specific
        robot would therefore bias the assignment toward whichever robot happened to be close.
        Evaluating from a reference position guaranteed to be outside panic distance yields the
        geometric driving point, which is identical for both robots.
        """
        away = target_pos - self.goal_pos
        norm = float(np.linalg.norm(away))
        away = away / norm if norm > 1e-6 else np.array([1.0, 0.0])
        reference = target_pos + away * (self.config.panic_distance_m + self.config.drive_distance_m + 1.0)
        return compute_driving_point(
            target_pos, target_vel, self.goal_pos, reference, self.planner_config
        ).point

    def _reset_occlusion_memory(self) -> None:
        """Forget the LOST-episode seed so the next episode seeds fresh."""
        self._last_known_cell = None
        self._last_known_point = None
        self._occlusion_seeded = False

    def _search_point(self, fallback: np.ndarray) -> np.ndarray:
        """Best-guess re-search point from the occlusion belief grid."""
        if float(self.occlusion_grid.belief.max()) > 0.0:
            row, col = self.occlusion_grid.best_guess_cell()
            return np.array(self.grid_map.cell_to_world(row, col), dtype=float)
        # Belief mass decayed to zero (or the seed cell was an obstacle): argmax
        # would return cell (0, 0), an arbitrary grid corner. Search the last
        # known target location instead.
        if self._last_known_point is not None:
            return self._last_known_point.copy()
        return np.asarray(fallback, dtype=float).copy()

    # ------------------------------------------------------------------ #
    # Main cycle                                                          #
    # ------------------------------------------------------------------ #

    def step(self, observation: Observation) -> HerdingOutput:
        """Run one full control cycle and return goals + telemetry."""
        start = time.perf_counter()

        if observation.occupancy is not None:
            expected = (self.config.grid_height_cells, self.config.grid_width_cells)
            if tuple(observation.occupancy.shape) == expected:
                self.grid_map.set_obstacle_mask_from_occupancy(observation.occupancy)
            else:
                logger.warning(
                    "ignoring occupancy grid with shape %s, expected %s",
                    tuple(observation.occupancy.shape), expected,
                )

        target_observed = observation.target_measurement is not None
        if target_observed:
            self.estimator.predict(observation.dt)
            self.estimator.update(observation.target_measurement)
            self._first_observation_seen = True
        elif self._first_observation_seen:
            self.estimator.predict(observation.dt)

        target_state = self.estimator.get_state()
        kf_converged = self._first_observation_seen and not target_state.is_lost

        # The escape model indexes the grid at the target's cell, so it can only run
        # once the estimator holds a real, on-grid estimate. Before the first
        # observation the KF state is all-zero and meaningless.
        escape_estimate = None
        if kf_converged and self._cell_or_none(target_state.position) is not None:
            escape_estimate = self.escape_model.compute(
                target_state.position, target_state.velocity,
                [observation.robot1_pos, observation.robot2_pos],
            )

        distance_to_goal = float(np.linalg.norm(target_state.position - self.goal_pos)) \
            if self._first_observation_seen else float("inf")
        escape_concentrated = bool(
            escape_estimate is not None
            and escape_estimate.probabilities.max() >= self.config.escape_concentration_threshold
        )

        fsm_state = self.fsm.step(FSMInputs(
            target_observed=target_observed, kf_converged=kf_converged,
            distance_to_goal_m=distance_to_goal, capture_radius_m=self.config.capture_radius_m,
            escape_prob_concentrated=escape_concentrated,
            occlusion_elapsed_sec=target_state.time_since_observation,
            occlusion_timeout_sec=self.config.occlusion_timeout_sec,
            capture_hold_required_sec=self.config.capture_hold_sec, dt=observation.dt,
        ))

        panic = False
        role_swapped = False

        if fsm_state == FSMState.LOST:
            # The occlusion grid is used ONLY in LOST: no escape model, planner, or
            # role assignment runs here, the two robots just sweep the belief peak.
            if not self._occlusion_seeded:
                self._last_known_point = np.asarray(target_state.position, dtype=float).copy()
                self._last_known_cell = self._clamped_cell_or_none(target_state.position)
                if self._last_known_cell is not None:
                    self.occlusion_grid.seed(*self._last_known_cell)
                    self._occlusion_seeded = True
            self.occlusion_grid.step(observation.dt)
            search_point = self._search_point(target_state.position)
            driver_id, blocker_id = self._current_roles()
            # Both robots converge on the same belief peak; offset the second goal so
            # they do not fight for one point.
            offset_point = resolve_separation(search_point, search_point, self.role_assigner_config)
            if driver_id == 1:
                robot1_goal, robot2_goal = search_point, offset_point
            else:
                robot1_goal, robot2_goal = offset_point, search_point
        else:
            self._reset_occlusion_memory()
            if fsm_state in (FSMState.HERD, FSMState.CORNER):
                candidate = self._nominal_driving_point(target_state.position, target_state.velocity)
                previous_driver = self.role_assigner._driver_id if self._roles_ever_assigned else None
                driver_id, blocker_id = self.role_assigner.assign(
                    observation.robot1_pos, observation.robot2_pos,
                    observation.robot1_heading, observation.robot2_heading,
                    candidate, observation.sim_time_sec,
                )
                # The very first assign() bootstraps the driver rather than swapping
                # away from a previously computed assignment, so it is not a swap.
                role_swapped = previous_driver is not None and driver_id != previous_driver
                self._roles_ever_assigned = True

                driver_pos = observation.robot1_pos if driver_id == 1 else observation.robot2_pos
                blocker_pos = observation.robot2_pos if driver_id == 1 else observation.robot1_pos
                driving = compute_driving_point(
                    target_state.position, target_state.velocity, self.goal_pos,
                    driver_pos, self.planner_config,
                )
                panic = driving.is_panic

                if escape_estimate is not None:
                    blocking_point = compute_blocking_point(
                        target_state.position, self.goal_pos, escape_estimate,
                        self.grid_map, self.planner_config,
                    )
                    blocking_point = resolve_separation(
                        driving.point, blocking_point, self.role_assigner_config
                    )
                else:
                    # Target estimate is off-grid, so there is no escape distribution and
                    # no meaningful blocking point. Keep driving, hold the blocker.
                    blocking_point = np.asarray(blocker_pos, dtype=float).copy()

                if driver_id == 1:
                    robot1_goal, robot2_goal = driving.point, blocking_point
                else:
                    robot1_goal, robot2_goal = blocking_point, driving.point
            else:
                # IDLE / SEARCH / TRACK / CAPTURED: hold position.
                robot1_goal = np.asarray(observation.robot1_pos, dtype=float).copy()
                robot2_goal = np.asarray(observation.robot2_pos, dtype=float).copy()
                driver_id, blocker_id = self._current_roles()

        latency_ms = (time.perf_counter() - start) * 1000.0
        return HerdingOutput(
            robot1_goal=robot1_goal, robot2_goal=robot2_goal, fsm_state=fsm_state,
            driver_id=driver_id, blocker_id=blocker_id, target_position=target_state.position,
            target_velocity=target_state.velocity,
            escape_top3=list(escape_estimate.top_k_routes) if escape_estimate else [],
            escape_directions=escape_estimate.directions if escape_estimate else None,
            escape_probabilities=escape_estimate.probabilities if escape_estimate else None,
            latency_ms=latency_ms, panic=panic, role_swapped=role_swapped,
        )
