# herding_controller/herding_controller/herding_core.py
"""모든 herding 하위 모듈을 결합하는 순수 Python 퍼사드(facade).

강한 제약: 이 모듈과 이 모듈이 가져오는 모든 모듈은, ROS 설치 없이도 전체
알고리즘을 오프라인에서 구동할 수 있도록 어떠한 ROS2 의존성도 가져서는 안 된다.
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
    """config/herding_params.yaml의 ros__parameters 블록을 평평하게 옮긴 것."""
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

    def __post_init__(self) -> None:
        """herd를 교착 상태에 빠뜨리는 것으로 알려진 파라미터 조합을 거부한다.

        Driver는 자신의 driving point로 수렴하여 그곳에 멈춘다. 타겟이 목표
        지점과 정렬되어 있으면 planner는 drive_distance_ease_factor만큼 그
        지점을 바깥쪽으로 완화하므로, Driver의 최근접 거리는
        drive_distance_m * drive_distance_ease_factor가 된다. 이 값이 타겟의
        반응 반경(flee_reaction_distance_m)을 초과하면 타겟은 결코 도주하지
        않고 전체 시스템은 전혀 진행되지 않는다 -- 측정된 성공률이 약 83%에서
        2.5%로 붕괴한다 (config/herding_params.yaml 참고). 이는 시스템을
        떠받치는 불변 조건이므로, 실행 중에 조용히 성능이 저하되는 대신
        설정 생성 시점에 여기서 즉시 크게 실패하도록 한다.
        """
        eased = self.drive_distance_m * self.drive_distance_ease_factor
        if eased >= self.flee_reaction_distance_m:
            raise ValueError(
                "drive_distance_m * drive_distance_ease_factor must be < "
                "flee_reaction_distance_m, otherwise the Driver stops outside the "
                "target's reaction radius and the target never flees; got "
                f"drive_distance_m={self.drive_distance_m} * "
                f"drive_distance_ease_factor={self.drive_distance_ease_factor} = {eased} "
                f">= flee_reaction_distance_m={self.flee_reaction_distance_m}"
            )


@dataclass
class Observation:
    """한 제어 주기 분량의 센서 입력을, 순수 Python/numpy 타입으로 표현한 것."""
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
    """herding_node.py가 한 제어 주기 동안 발행하는 데 필요한 모든 것."""
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
    """grid, estimator, escape model, planner, role assigner, FSM, occlusion grid를 서로 연결한다."""

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
    # 그리드 헬퍼                                                          #
    # ------------------------------------------------------------------ #

    def _cell_or_none(self, position: np.ndarray) -> tuple[int, int] | None:
        """월드 좌표에 대응하는 셀 인덱스, 그리드 밖이거나 유한하지 않으면 None."""
        try:
            return self.grid_map.world_to_cell(float(position[0]), float(position[1]))
        except (ValueError, TypeError):
            return None

    def _clamped_cell_or_none(self, position: np.ndarray) -> tuple[int, int] | None:
        """월드 좌표에 대응하는 셀 인덱스를 그리드 안으로 클램프한 값; 유한하지 않으면 None."""
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
        """RoleAssigner에 현재 고정(latch)되어 있는 역할 배정."""
        driver_id = self.role_assigner._driver_id
        return driver_id, (2 if driver_id == 1 else 1)

    def _nominal_driving_point(self, target_pos: np.ndarray, target_vel: np.ndarray) -> np.ndarray:
        """panic-retreat를 무시한 driving point로, 로봇에 무관한 역할 배정 후보로 사용된다.

        compute_driving_point()는 해당 로봇이 panic_distance_m 이내에 있을 때
        robot_pos에 밀착하는 *retreat* 지점을 반환한다. 따라서 특정 로봇을
        기준으로 계산한 후보를 role assigner에 넘기면, 우연히 가까이 있던
        로봇 쪽으로 배정이 편향될 것이다. panic distance 밖에 있음이 보장된
        기준 위치에서 평가하면 두 로봇 모두에게 동일한 기하학적 driving
        point를 얻을 수 있다.
        """
        away = target_pos - self.goal_pos
        norm = float(np.linalg.norm(away))
        away = away / norm if norm > 1e-6 else np.array([1.0, 0.0])
        reference = target_pos + away * (self.config.panic_distance_m + self.config.drive_distance_m + 1.0)
        return compute_driving_point(
            target_pos, target_vel, self.goal_pos, reference, self.planner_config
        ).point

    def _reset_occlusion_memory(self) -> None:
        """LOST 에피소드의 시드를 초기화하여 다음 에피소드가 새로 시드되도록 한다."""
        self._last_known_cell = None
        self._last_known_point = None
        self._occlusion_seeded = False

    def _search_point(self, fallback: np.ndarray) -> np.ndarray:
        """occlusion belief grid로부터 얻은 최선 추정 재탐색 지점."""
        if float(self.occlusion_grid.belief.max()) > 0.0:
            row, col = self.occlusion_grid.best_guess_cell()
            return np.array(self.grid_map.cell_to_world(row, col), dtype=float)
        # belief 질량이 0으로 감쇠했거나(또는 시드 셀이 장애물이었던 경우):
        # argmax는 임의의 그리드 모서리인 셀 (0, 0)을 반환하게 된다. 그 대신
        # 마지막으로 알려진 타겟 위치를 탐색한다.
        if self._last_known_point is not None:
            return self._last_known_point.copy()
        return np.asarray(fallback, dtype=float).copy()

    # ------------------------------------------------------------------ #
    # 메인 사이클                                                          #
    # ------------------------------------------------------------------ #

    def step(self, observation: Observation) -> HerdingOutput:
        """제어 주기 전체를 한 번 실행하고 목표점과 텔레메트리를 반환한다."""
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

        # escape model은 타겟이 위치한 셀을 기준으로 그리드를 조회하므로,
        # estimator가 그리드 위의 실제 추정값을 가지고 있을 때만 실행할 수
        # 있다. 최초 관측 이전에는 KF 상태가 전부 0이며 의미가 없다.
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
            # occlusion grid는 오직 LOST 상태에서만 사용된다: 여기서는 escape
            # model, planner, role assignment 어느 것도 실행되지 않으며, 두
            # 로봇은 그저 belief의 최고점을 훑고 지나간다.
            if not self._occlusion_seeded:
                self._last_known_point = np.asarray(target_state.position, dtype=float).copy()
                self._last_known_cell = self._clamped_cell_or_none(target_state.position)
                if self._last_known_cell is not None:
                    self.occlusion_grid.seed(*self._last_known_cell)
                    self._occlusion_seeded = True
            self.occlusion_grid.step(observation.dt)
            search_point = self._search_point(target_state.position)
            driver_id, blocker_id = self._current_roles()
            # 두 로봇 모두 동일한 belief 최고점으로 수렴하므로, 하나의 지점을
            # 두고 다투지 않도록 두 번째 목표를 오프셋한다.
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
                # 최초의 assign() 호출은 이전에 계산된 배정으로부터 교체하는
                # 것이 아니라 driver를 부트스트랩하는 것이므로 swap이 아니다.
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
                    # 타겟 추정치가 그리드 밖에 있어 escape distribution이
                    # 없고 의미 있는 blocking point도 없다. driving은
                    # 계속하고 blocker는 제자리를 유지한다.
                    blocking_point = np.asarray(blocker_pos, dtype=float).copy()

                if driver_id == 1:
                    robot1_goal, robot2_goal = driving.point, blocking_point
                else:
                    robot1_goal, robot2_goal = blocking_point, driving.point
            else:
                # IDLE / SEARCH / TRACK / CAPTURED: 위치를 유지한다.
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
