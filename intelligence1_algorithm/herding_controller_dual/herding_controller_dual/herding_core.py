# herding_controller_dual/herding_controller_dual/herding_core.py
"""모든 herding 하위 모듈을 결합하는 순수 Python 퍼사드(facade).

강한 제약: 이 모듈과 이 모듈이 가져오는 모든 모듈은, ROS 설치 없이도 전체
알고리즘을 오프라인에서 구동할 수 있도록 어떠한 ROS2 의존성도 가져서는 안 된다.
"""
import logging
import time
from dataclasses import dataclass, field

import numpy as np

from herding_controller_dual.escape_model import EscapeModel, EscapeModelConfig
from herding_controller_dual.geodesic_field import GeodesicField
from herding_controller_dual.grid_map import GridConfig, GridMap
from herding_controller_dual.herding_planner import (
    PlannerConfig,
    compute_blocking_point,
    compute_driving_point,
    compute_pressure_pair,
    compute_shaping_pair,
    compute_guard_point,
    compute_endgame_pincer,
)
from herding_controller_dual.occlusion_grid import OcclusionGrid, OcclusionGridConfig
from herding_controller_dual.role_assigner import RoleAssigner, RoleAssignerConfig, resolve_separation
from herding_controller_dual.state_machine import FSMInputs, FSMState, HerdingStateMachine
from herding_controller_dual.target_estimator import EstimatorConfig, TargetEstimator

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
    min_robot_separation_m: float
    diffusion_rate: float
    decay_factor: float
    grid_origin_x_m: float = 0.0
    grid_origin_y_m: float = 0.0
    # Blocking Point 이력현상(hysteresis) 유지 시간 -- _stabilize_blocking_point()
    # 참고. 트러블슈팅 노트 10-6에서 flicker(매 프레임 요동)를 막으려고
    # 도입했다가, 10-7에서 "1.0초 고정이 이 방 크기 기준 오히려 너무 길어
    # Blocker가 계속 도착 전에 목표가 바뀌는 지연(lag)을 만든다"는 게 밝혀져
    # 하드코딩에서 이 필드로 뺐다. 최적값은 run_real_map_algo_suite 스윕으로 정한다.
    blocking_point_commit_sec: float = 1.0
    # 교착(deadlock) 감지 -- 트러블슈팅 노트 11-8 참고. Driver와 Blocker가
    # 서로 다른 각도에서 밀다가, 지정된 포획존이 아닌 방 안의 다른 벽
    # 구석에 표적을 우연히 핀서(pincer)로 가둬버리는 경우가 있다. 표적이
    # 이 시간(초) 이상 거의 안 움직이는데도 아직 포획존 근처가 아니면,
    # "몰이 성공"이 아니라 "우연히 낀 것"으로 판단한다.
    deadlock_stall_sec: float = 3.0
    # 표적이 이 반경(m) 안에 머무르면 "그 자리에 갇혀 있다"고 본다. 순간
    # 속도가 아니라 반경으로 판정하는 이유: 벽 충돌회피 물리엔진이 갇힌
    # 상태에서도 스텝마다 미세하게 위치를 밀어내는데(실측 0.01~0.03m/스텝),
    # 이게 순간속도 기준으로는 매번 "움직였다"로 잡혀 정지 누적이 리셋돼
    # 버린다(트러블슈팅 노트 11-8) -- 그래서 "기준점에서 이 반경을 벗어났나"
    # 로만 리셋을 판단한다.
    deadlock_drift_radius_m: float = 0.1
    # 교착 판정 시, Blocker를 표적으로부터 이 거리(m)만큼 뒤로 물러나게 해
    # 협공 각도를 풀어준다. flee_reaction_distance_m보다 확실히 커야
    # 표적의 반응 범위 밖으로 완전히 빠져나간다.
    deadlock_release_distance_m: float = 1.0
    # Blocker(로봇 2)의 escape_model 반발항 활성화 거리(m) -- 트러블슈팅
    # 노트 11-9/11-10/12 참고. 표적이 Blocker로부터 이 거리보다 멀면
    # Blocker의 존재를 EscapeModel._robot_repulsion() 계산에서 아예
    # 제외한다. 기본값 inf는 게이팅 없음(기존 동작과 완전히 동일) --
    # 최적값은 experiments/blocker_contribution_ablation.py 스윕으로 정한다.
    robot_repulsion_activation_distance_m: float = float("inf")
    # (플랜 B 전용) 동적 Driver/Blocker 역할 배정 이력현상 -- role_assigner.py::RoleAssigner
    # 참고. 두 로봇의 비용(거리 + 회전각*role_cost_turn_weight) 차이가
    # role_swap_margin 이상이면서 마지막 스왑 후 role_swap_cooldown_sec
    # 이상 지났을 때만 Driver를 교체한다. 옛 10x10m 아레나 검증값을 그대로
    # 이식(algorithm/dual-robot-herding 브랜치, 4abf472) -- 이 방(5.3x7.35m)
    # 기준으로 재튜닝은 안 됨.
    role_swap_margin: float = 0.5
    role_swap_cooldown_sec: float = 2.0
    role_cost_turn_weight: float = 0.3
    # --- 압박 선분 모드 (2026-08-08, 트러블슈팅 노트 16번 항목) ---
    # True면 HERD/CORNER에서 Driver/Blocker를 따로 계산하지 않고, 두 로봇
    # 목표점을 compute_pressure_pair()로 **같이** 뽑는다. 목적은 성공률
    # 상승이 아니라 "로봇 B가 빠지면 진로의 절반이 열린다"는 구조를 만들어
    # 기여를 지표로 증명하는 것이다. 기본값 False로 기존 동작 유지.
    pressure_mode_enabled: bool = False
    # 로봇/표적의 물리 치수 -- 압박 선분 배치에만 쓴다 (TurtleBot 4 실측).
    robot_radius_m: float = 0.171
    robot_wall_clearance_m: float = 0.03
    # 압박 선분에서 두 로봇을 목표 반대편 기준 좌우로 얼마나 벌릴지(도).
    # 크면 진로를 넓게 덮지만 미는 힘(cos 성분)이 줄어드는 트레이드오프.
    pressure_half_angle_deg: float = 45.0
    # 도주 분포 직접 최적화 모드 (2026-08-08, 트러블슈팅 노트 16번 항목).
    # True면 두 로봇을 '표적이 포획존 쪽으로 갈 확률이 최대가 되는' 자리에
    # 같이 놓는다 -- Driver/Blocker 구분 없음. 기본값 False로 기존 동작 유지.
    shaping_mode_enabled: bool = False
    # 수비-쓸기 모드 (2026-08-08, 트러블슈팅 노트 16번 항목). True면 한 로봇은
    # 표적을 쫓지 않고 '도주로의 가장 좁은 길목'에 미리 가서 지키고, 다른
    # 로봇이 표적을 트랩 쪽으로 민다 -- lion-and-man 2인 추격 전략을 몰이에
    # 적용한 것. 지금까지 방식은 전부 두 로봇이 표적을 쫓아서, 표적보다 느린
    # 로봇이 영원히 뒤따라가기만 했다. 기본값 False로 기존 동작 유지.
    guard_mode_enabled: bool = False
    # 수비 지점을 이만큼 이동해야 갱신한다(이력현상). 매 주기 길목이 바뀌면
    # 수비수가 도착도 못 하고 계속 옮겨다닌다(10-6의 flicker와 같은 문제).
    guard_commit_distance_m: float = 0.5
    # 엔드게임 협공 (2026-08-09, 트러블슈팅 노트 16-15). 표적이 트랩
    # endgame_trigger_radius_m 안에 들어오면, 두 로봇이 트랩 반대편 좌우를
    # 막아 표적이 트랩 쪽으로밖에 못 가게 한다. 실패 분석에서 구석회피
    # 표적의 실패 92건 중 91건이 트랩 0.6m 안까지 왔다가 마지막 7cm를
    # 못 넘고 옆으로 빠진 것이 확인돼 만든 모드다.
    endgame_pincer_enabled: bool = False
    endgame_trigger_radius_m: float = 0.8
    endgame_half_angle_deg: float = 90.0
    # 표적이 트리거 반경 안에 이 시간(초) 이상 머무는데도 포획이 안 되면
    # 그때 협공으로 전환한다. 0이면 즉시 발동. 얌전한 표적은 그냥 밀면
    # 바로 들어가는데 협공하느라 두 로봇이 벌어지면 미는 힘이 약해져
    # 오히려 나빠진다(실측: reactive_flee 90.7%->72.7%) -- 그래서 '먼저
    # 밀어보고 안 들어가면 협공'으로 조건을 건다.
    endgame_stall_sec: float = 2.0

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
    deadlock_release: bool = False
    # 압박 선분 모드에서만 채워진다: 표적 진로 단면 중 두 로봇이 덮은 비율.
    pressure_coverage: float | None = None
    # 도주 분포 최적화 모드에서만: 두 대 배치 시 목표방향 확률과, 같은 조건에서
    # 한 대만 썼을 때의 확률 (두 번째 로봇의 순기여를 그대로 보여준다).
    shaping_goal_prob: float | None = None
    shaping_goal_prob_single: float | None = None
    # 수비-쓸기 모드에서만: 수비수가 지키는 길목의 통로 폭(좁을수록 잘 막힘).
    guard_corridor_width_m: float | None = None


class HerdingCore:
    """grid, estimator, escape model, planner, FSM, occlusion grid를 서로 연결한다.

    (플랜 B) 로봇 역할은 매 제어 주기 동적으로 재배정될 수 있다: 두 로봇 중
    어느 쪽이 미는 역할(Driver)을 할지는 `RoleAssigner`가 거리+회전비용으로
    비교해서 결정하고, margin+cooldown 이력현상으로 진동(매 주기 역할이
    뒤집히는 것)을 막는다. "처음에 누가 표적을 발견해서 뛰어드는가"는 여전히
    상위 fleet 시스템의 몫이지만(그 배정은 RoleAssigner의 최초 호출에서
    비용 최적값으로 그대로 받아들여진다), 일단 두 로봇이 필드에서 몰이 중일
    때는 이 클래스가 실시간으로 Driver/Blocker를 다시 정한다. 이 클래스가
    실제로 계산해서 두 로봇 모두에 명령한다(플랜 A인 `herding_controller`는
    반대로 역할을 에피소드 내내 고정하고 로봇 1을 조종하지 않는다 -- 실제
    운용 제약 때문에 그렇게 결정됐고, 거기서는 여전히 유효하다. 자세한
    배경은 herding_controller_트러블슈팅_노트.md 10번/13번 항목 참고).
    """

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
            robot_repulsion_activation_distance_m=config.robot_repulsion_activation_distance_m,
        ), self.grid_map)
        self.planner_config = PlannerConfig(
            drive_distance_m=config.drive_distance_m, panic_distance_m=config.panic_distance_m,
            alignment_threshold=config.alignment_threshold,
            drive_distance_ease_factor=config.drive_distance_ease_factor,
            block_lookahead_m=config.block_lookahead_m,
        )
        self.fsm = HerdingStateMachine()
        self.role_assigner = RoleAssigner(RoleAssignerConfig(
            role_swap_margin=config.role_swap_margin,
            role_swap_cooldown_sec=config.role_swap_cooldown_sec,
            role_cost_turn_weight=config.role_cost_turn_weight,
        ))
        self._roles_ever_assigned = False
        self.occlusion_grid = OcclusionGrid(
            OcclusionGridConfig(diffusion_rate=config.diffusion_rate, decay_factor=config.decay_factor),
            self.grid_map,
        )
        self._last_known_cell: tuple[int, int] | None = None
        self._last_known_point: np.ndarray | None = None
        self._occlusion_seeded = False
        self._first_observation_seen = False
        # 벽을 고려한 목표 방향 필드 (geodesic_field.py). goal_pos는 설정값으로
        # 미션 내내 고정이므로, 맵이 실제로 도착한 뒤 딱 한 번만 계산해서
        # 캐싱한다 -- 제어 주기(5Hz)마다 다익스트라를 다시 돌리면 지연시간
        # 예산을 넘긴다. `_geodesic_ready`는 "맵이 아직 안 왔을 때"와 "맵은
        # 왔는데 goal_pos가 격자 밖이라 계산에 실패했을 때"를 구분해서, 후자를
        # 매 주기 재시도하지 않게 한다.
        self._geodesic_field: GeodesicField | None = None
        self._geodesic_ready = False
        # 압박 선분 배치용 벽까지 거리장(미터). 맵이 온 뒤 1회만 계산해 캐싱한다.
        self._clearance_m = None
        # 수비 지점 이력현상 상태 -- guard_commit_distance_m 참고.
        self._committed_guard_point = None
        # 엔드게임: 표적이 트리거 반경 안에 처음 들어온 시각.
        self._endgame_entered_sec = None
        # Blocking Point 이력현상(hysteresis) 상태 -- 아래 _stabilize_blocking_point() 참고.
        self._committed_blocking_point: np.ndarray | None = None
        self._committed_blocking_time: float = 0.0
        # 교착 감지 상태 -- 아래 _update_deadlock_state() 참고.
        self._deadlock_anchor_position: np.ndarray | None = None
        self._deadlock_anchor_time_sec: float = 0.0

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

    def _ensure_geodesic_field(self) -> None:
        """맵이 도착했다면, goal_pos 기준 geodesic 필드를 1회 계산해 캐싱한다."""
        if self._geodesic_ready:
            return
        if not self.grid_map.obstacle_mask.any():
            return  # 아직 실제 맵을 못 받음 (전부 False) -- 다음 주기에 재시도
        try:
            goal_row, goal_col = self.grid_map.world_to_cell(*self.goal_pos)
        except ValueError:
            # goal_pos가 격자 밖: 설정 오류이며 재시도해도 달라지지 않는다.
            self._geodesic_ready = True
            return
        self._geodesic_field = GeodesicField(self.grid_map, goal_row, goal_col)
        self._geodesic_ready = True

    def _ensure_clearance_field(self):
        """벽까지 거리장(미터)을 맵 도착 후 1회만 계산해 캐싱한다.

        압박 선분(compute_pressure_pair)이 "로봇이 여기 설 수 있나"를 판정할 때
        쓴다. geodesic 필드와 같은 이유로 제어 주기마다 다시 계산할 수 없다.
        """
        if self._clearance_m is None and self.grid_map.obstacle_mask.any():
            from scipy import ndimage
            self._clearance_m = (
                ndimage.distance_transform_edt(~self.grid_map.obstacle_mask)
                * self.grid_map.config.resolution_m
            )
        return self._clearance_m

    # 국소 기울기 한 지점이 아니라 실제 경로를 따라 앞을 내다보는 거리.
    # block_lookahead_m과 같은 자릿수로 맞춰, "한 코너 정도는 미리 본다"는
    # 감각에 맞춘 값 -- 별도 config 파라미터로 빼지 않은 이유는 이게
    # 알고리즘 내부 구현 디테일(geodesic 경로 추적의 해상도)이지, 배포
    # 환경마다 바뀌어야 하는 값이 아니기 때문이다.
    _WAYPOINT_LOOKAHEAD_M = 1.5

    def _direction_goal(self, position: np.ndarray) -> np.ndarray:
        """compute_driving_point/compute_blocking_point에 goal_pos 대신 넘길 목표 방향 기준점.

        geodesic 필드가 준비되어 있으면, 실제 경로를 따라
        `_WAYPOINT_LOOKAHEAD_M`만큼 앞서 내다본 지점(코너를 미리 반영)을
        반환한다. 실패하면(그리드 밖 등) 국소 기울기 한 번만 본 값으로,
        그것도 실패하면 실제 goal_pos로 폴백한다 -- 두 함수 모두 이 값과
        target_pos의 차이에서 "방향"만 뽑아 쓰므로 안전한 폴백 체인이다.
        """
        if self._geodesic_field is not None:
            waypoint = self._geodesic_field.waypoint_ahead(position, lookahead_m=self._WAYPOINT_LOOKAHEAD_M)
            if waypoint is not None:
                return waypoint
            virtual_goal = self._geodesic_field.virtual_goal_point(position)
            if virtual_goal is not None:
                return virtual_goal
        return self.goal_pos

    def _stabilize_blocking_point(self, candidate: np.ndarray, now_sec: float) -> np.ndarray:
        """새 Blocking Point 후보를 즉시 채택하지 않고, 이전 값을 일정 시간 유지한다.

        compute_blocking_point()는 "block_lookahead_m(1.8m) 앞의 지점이
        장애물인가"로 후보 방향을 고르는데, 표적이 문턱 근처에 있으면 몇 cm만
        움직여도 그 앞 지점이 벽 반대편으로 넘어가버려서 후보가 완전히 다른
        방향으로 바뀔 수 있다 -- 도주확률 예측 자체는 안정적인데도(트러블슈팅
        노트 10-6 항목 실측: argmax 방향은 그대로, 목표 좌표만 1.3m 넘게
        순간이동) 목표점만 매 제어 주기(0.2초)마다 요동쳐서, 로봇이 어느
        쪽으로도 진행하지 못하고 제자리에서 맴돌게 만든다. role_assigner.py가
        예전에 역할 교체 진동을 막던 것과 같은 종류의 문제라, 같은
        해법(이력현상)을 쓴다.

        `config.blocking_point_commit_sec` 미만이면 이전에 커밋한 점을 그대로
        반환하고, 그 이상 지났으면(또는 아직 커밋된 값이 없으면) 새 후보를
        채택하고 그 시각을 기록한다. 매 주기 값 자체는 계속 다시 계산되지만
        (그래야 표적이 실제로 멀리 움직였을 때 반영되므로), 화면에 나가는/
        로봇에 명령되는 값은 이 함수를 거친 안정화된 값이다.

        주의(트러블슈팅 노트 10-7): 이 값을 너무 크게 두면 반대로 "목표가
        멀리 튄 뒤 다음 커밋까지 로봇이 도착도 못 하고 또 바뀌는" 지연
        문제가 생긴다 -- flicker(너무 잦은 갱신)와 lag(너무 드문 갱신) 사이의
        트레이드오프이므로, 이 상수를 바꿀 때는 반드시
        test/run_validation.py::run_real_map_algo_suite()로 재검증한다.
        """
        if self._committed_blocking_point is None:
            self._committed_blocking_point = candidate
            self._committed_blocking_time = now_sec
            return candidate
        if now_sec - self._committed_blocking_time >= self.config.blocking_point_commit_sec:
            self._committed_blocking_point = candidate
            self._committed_blocking_time = now_sec
            return candidate
        return self._committed_blocking_point

    def _update_deadlock_state(self, target_pos: np.ndarray, now_sec: float) -> bool:
        """표적이 한 자리(반경 deadlock_drift_radius_m 안)에 오래 머물렀는지
        갱신하고, 교착 판정 여부를 반환한다.

        기준점(anchor)을 두고, 표적이 그 반경을 벗어날 때만 기준점을
        새로 잡는다 -- 매 스텝의 순간 이동량으로 판단하면 벽 충돌회피
        물리엔진의 미세한 스텝별 흔들림에도 매번 리셋돼버려 실제로 갇혀
        있어도 절대 누적되지 않는다(트러블슈팅 노트 11-8, seed=1200008로
        실측).
        """
        if self._deadlock_anchor_position is None:
            self._deadlock_anchor_position = np.asarray(target_pos, dtype=float).copy()
            self._deadlock_anchor_time_sec = now_sec
            return False
        drift = float(np.linalg.norm(target_pos - self._deadlock_anchor_position))
        if drift > self.config.deadlock_drift_radius_m:
            self._deadlock_anchor_position = np.asarray(target_pos, dtype=float).copy()
            self._deadlock_anchor_time_sec = now_sec
            return False
        return (now_sec - self._deadlock_anchor_time_sec) >= self.config.deadlock_stall_sec

    def _reset_deadlock_state(self) -> None:
        self._deadlock_anchor_position = None
        self._deadlock_anchor_time_sec = 0.0

    def _deadlock_release_point(self, target_pos: np.ndarray, blocker_pos: np.ndarray) -> np.ndarray:
        """Blocker를 표적으로부터 deadlock_release_distance_m만큼 뒤로 물러나게 한다.

        표적 위치에서 (현재 Blocker 역할인 로봇의) *현재* 위치를 지나는
        직선 방향으로 물러나므로, Blocker가 어느 각도에서 핀서에 가담하고
        있었든 그 각도 그대로 벌어지기만 한다 (엉뚱한 새 방향으로 튀지
        않음). 플랜 A와 달리 이 로봇이 매 주기 로봇 1일 수도 로봇 2일 수도
        있다 -- 호출부(step())가 그 주기의 실제 Blocker 위치를 넘긴다.
        """
        away = blocker_pos - target_pos
        norm = np.linalg.norm(away)
        away = away / norm if norm > 1e-6 else np.array([1.0, 0.0])
        return target_pos + away * self.config.deadlock_release_distance_m

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

    def _current_roles(self) -> tuple[int, int]:
        """RoleAssigner에 현재 고정(latch)되어 있는 역할 배정."""
        driver_id = self.role_assigner._driver_id
        return driver_id, (2 if driver_id == 1 else 1)

    def _nominal_driving_point(
        self, target_pos: np.ndarray, target_vel: np.ndarray, direction_goal: np.ndarray
    ) -> np.ndarray:
        """panic-retreat를 무시한 driving point로, 로봇에 무관한 역할 배정 후보로 사용된다.

        compute_driving_point()는 해당 로봇이 panic_distance_m 이내에 있을 때
        robot_pos에 밀착하는 *retreat* 지점을 반환한다. 따라서 특정 로봇을
        기준으로 계산한 후보를 role assigner에 넘기면, 우연히 가까이 있던
        로봇 쪽으로 배정이 편향될 것이다. panic distance 밖에 있음이 보장된
        기준 위치에서 평가하면 두 로봇 모두에게 동일한 기하학적 driving
        point를 얻을 수 있다.
        """
        away = target_pos - direction_goal
        norm = float(np.linalg.norm(away))
        away = away / norm if norm > 1e-6 else np.array([1.0, 0.0])
        reference = target_pos + away * (self.config.panic_distance_m + self.config.drive_distance_m + 1.0)
        return compute_driving_point(
            target_pos, target_vel, direction_goal, reference, self.planner_config
        ).point

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

        self._ensure_geodesic_field()

        target_observed = observation.target_measurement is not None
        if target_observed:
            self.estimator.predict(observation.dt)
            self.estimator.update(observation.target_measurement)
            self._first_observation_seen = True
        elif self._first_observation_seen:
            self.estimator.predict(observation.dt)

        target_state = self.estimator.get_state()
        kf_converged = self._first_observation_seen and not target_state.is_lost

        # escape_model.compute()가 FSM 상태 판단(escape_concentrated)보다
        # 먼저 실행돼야 하므로, 이 시점엔 이번 주기의 역할이 아직 새로
        # 계산되지 않았다 -- 직전 주기까지 확정(latch)된 역할을 그대로
        # 쓴다. HERD/CORNER 분기에서 role_assigner.assign()이 이번 주기
        # 역할을 다시 정하면 아래 driver_id/blocker_id를 덮어쓴다(반영이
        # 한 제어 주기(0.2초) 늦어질 수 있지만, 스왑이 실제로 일어나는
        # 순간에만 해당하는 미세한 차이다).
        driver_id, blocker_id = self._current_roles()

        # escape model은 타겟이 위치한 셀을 기준으로 그리드를 조회하므로,
        # estimator가 그리드 위의 실제 추정값을 가지고 있을 때만 실행할 수
        # 있다. 최초 관측 이전에는 KF 상태가 전부 0이며 의미가 없다.
        escape_estimate = None
        if kf_converged and self._cell_or_none(target_state.position) is not None:
            escape_estimate = self.escape_model.compute(
                target_state.position, target_state.velocity,
                [observation.robot1_pos, observation.robot2_pos],
                blocker_index=blocker_id - 1,
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
        deadlock_release = False

        if fsm_state == FSMState.LOST:
            # occlusion grid는 오직 LOST 상태에서만 사용된다: 여기서는 escape
            # model, planner, role assignment 어느 것도 실행되지 않으며, 두
            # 로봇은 그저 belief의 최고점을 훑고 지나간다. 역할은 재배정하지
            # 않고(위에서 이미 읽은 latched 값을 그대로 쓰고) 마지막으로
            # 확정된 배정을 그대로 읽기만 한다.
            if not self._occlusion_seeded:
                self._last_known_point = np.asarray(target_state.position, dtype=float).copy()
                self._last_known_cell = self._clamped_cell_or_none(target_state.position)
                if self._last_known_cell is not None:
                    self.occlusion_grid.seed(*self._last_known_cell)
                    self._occlusion_seeded = True
            self.occlusion_grid.step(observation.dt)
            search_point = self._search_point(target_state.position)
            # 두 로봇 모두 동일한 belief 최고점으로 수렴하므로, 하나의 지점을
            # 두고 다투지 않도록 두 번째 목표를 오프셋한다.
            offset_point = resolve_separation(search_point, search_point, self.config.min_robot_separation_m)
            if driver_id == 1:
                robot1_goal, robot2_goal = search_point, offset_point
            else:
                robot1_goal, robot2_goal = offset_point, search_point
            # 재관측 후 HERD로 돌아왔을 때 LOST 이전의 낡은 목표점을
            # 이어받지 않도록 이력현상 상태를 지운다.
            self._committed_blocking_point = None
            self._committed_guard_point = None
            self._endgame_entered_sec = None
            self._reset_deadlock_state()
        else:
            self._reset_occlusion_memory()
            if fsm_state in (FSMState.HERD, FSMState.CORNER):
                # compute_driving_point/compute_blocking_point에는 실제
                # goal_pos 대신 direction_goal(벽을 고려한 가상 목표점,
                # geodesic_field.py)을 넘긴다: 두 함수 모두 이 값과
                # target_pos의 차이에서 방향만 뽑아 쓰므로, 직선이 벽을
                # 가로지르는 상황에서도 실제로 갈 수 있는 방향을 가리키게
                # 된다 (geodesic 필드가 아직 없으면 goal_pos 그대로 폴백).
                direction_goal = self._direction_goal(target_state.position)

                # 어느 로봇이 Driver를 할지 매 주기 다시 판단한다: 역할무관
                # 후보 지점(_nominal_driving_point)을 기준으로 두 로봇의
                # 비용(거리+회전)을 비교해서 RoleAssigner가 margin+cooldown
                # 이력현상을 거쳐 결정한다.
                candidate = self._nominal_driving_point(
                    target_state.position, target_state.velocity, direction_goal
                )
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

                clearance = self._ensure_clearance_field()
                if self.config.endgame_pincer_enabled and clearance is not None:
                    # 트리거 반경 안에 머문 시간을 누적한다 -- 벗어나면 리셋.
                    if distance_to_goal <= self.config.endgame_trigger_radius_m:
                        if self._endgame_entered_sec is None:
                            self._endgame_entered_sec = observation.sim_time_sec
                    else:
                        self._endgame_entered_sec = None
                    stalled = (self._endgame_entered_sec is not None
                               and observation.sim_time_sec - self._endgame_entered_sec
                               >= self.config.endgame_stall_sec)
                if self.config.endgame_pincer_enabled and clearance is not None and stalled:
                    pincer = compute_endgame_pincer(
                        target_state.position, self.goal_pos, self.grid_map, clearance,
                        self.config.robot_radius_m + self.config.robot_wall_clearance_m,
                        stand_distance_m=self.config.drive_distance_m,
                        trigger_radius_m=self.config.endgame_trigger_radius_m,
                        half_angle_rad=np.radians(self.config.endgame_half_angle_deg),
                    )
                    if pincer is not None:
                        straight = (float(np.linalg.norm(observation.robot1_pos - pincer.point_a))
                                    + float(np.linalg.norm(observation.robot2_pos - pincer.point_b)))
                        crossed = (float(np.linalg.norm(observation.robot1_pos - pincer.point_b))
                                   + float(np.linalg.norm(observation.robot2_pos - pincer.point_a)))
                        if straight <= crossed:
                            robot1_goal, robot2_goal = pincer.point_a, pincer.point_b
                        else:
                            robot1_goal, robot2_goal = pincer.point_b, pincer.point_a
                        self._committed_blocking_point = None
                        self._reset_deadlock_state()
                        latency_ms = (time.perf_counter() - start) * 1000.0
                        return HerdingOutput(
                            robot1_goal=robot1_goal, robot2_goal=robot2_goal, fsm_state=fsm_state,
                            driver_id=driver_id, blocker_id=blocker_id,
                            target_position=target_state.position,
                            target_velocity=target_state.velocity,
                            escape_top3=list(escape_estimate.top_k_routes) if escape_estimate else [],
                            escape_directions=escape_estimate.directions if escape_estimate else None,
                            escape_probabilities=escape_estimate.probabilities if escape_estimate else None,
                            latency_ms=latency_ms, panic=False, role_swapped=role_swapped,
                            deadlock_release=False,
                        )

                if self.config.guard_mode_enabled and clearance is not None:
                    guard = compute_guard_point(
                        target_state.position, direction_goal - target_state.position,
                        self.grid_map, clearance,
                        self.config.robot_radius_m + self.config.robot_wall_clearance_m,
                    )
                    if guard is not None:
                        # 이력현상: 새 길목이 충분히 멀어졌을 때만 갱신한다.
                        if (self._committed_guard_point is None
                                or float(np.linalg.norm(guard.point - self._committed_guard_point))
                                > self.config.guard_commit_distance_m):
                            self._committed_guard_point = guard.point.copy()
                        guard_point = self._committed_guard_point
                        # 쓸기 담당은 기존 검증된 Driving Point 그대로.
                        sweep = compute_driving_point(
                            target_state.position, target_state.velocity, direction_goal,
                            driver_pos, self.planner_config,
                        )
                        panic = sweep.is_panic
                        # 배정: 각자 가까운 쪽을 맡아 서로 경로가 꼬이지 않게 한다.
                        straight = (float(np.linalg.norm(observation.robot1_pos - sweep.point))
                                    + float(np.linalg.norm(observation.robot2_pos - guard_point)))
                        crossed = (float(np.linalg.norm(observation.robot1_pos - guard_point))
                                   + float(np.linalg.norm(observation.robot2_pos - sweep.point)))
                        if straight <= crossed:
                            robot1_goal, robot2_goal = sweep.point, guard_point
                        else:
                            robot1_goal, robot2_goal = guard_point, sweep.point
                        self._committed_blocking_point = None
                        self._reset_deadlock_state()
                        latency_ms = (time.perf_counter() - start) * 1000.0
                        return HerdingOutput(
                            robot1_goal=robot1_goal, robot2_goal=robot2_goal, fsm_state=fsm_state,
                            driver_id=driver_id, blocker_id=blocker_id,
                            target_position=target_state.position,
                            target_velocity=target_state.velocity,
                            escape_top3=list(escape_estimate.top_k_routes) if escape_estimate else [],
                            escape_directions=escape_estimate.directions if escape_estimate else None,
                            escape_probabilities=escape_estimate.probabilities if escape_estimate else None,
                            latency_ms=latency_ms, panic=panic, role_swapped=role_swapped,
                            deadlock_release=False,
                            guard_corridor_width_m=guard.corridor_width_m,
                        )

                if self.config.shaping_mode_enabled and clearance is not None:
                    shaping = compute_shaping_pair(
                        target_state.position, target_state.velocity,
                        direction_goal - target_state.position, self.escape_model,
                        self.grid_map, clearance,
                        self.config.robot_radius_m, self.config.robot_wall_clearance_m,
                        stand_radius_m=self.config.drive_distance_m,
                    )
                    straight = (float(np.linalg.norm(observation.robot1_pos - shaping.point_a))
                                + float(np.linalg.norm(observation.robot2_pos - shaping.point_b)))
                    crossed = (float(np.linalg.norm(observation.robot1_pos - shaping.point_b))
                               + float(np.linalg.norm(observation.robot2_pos - shaping.point_a)))
                    if straight <= crossed:
                        robot1_goal, robot2_goal = shaping.point_a, shaping.point_b
                    else:
                        robot1_goal, robot2_goal = shaping.point_b, shaping.point_a
                    self._committed_blocking_point = None
                    self._reset_deadlock_state()
                    latency_ms = (time.perf_counter() - start) * 1000.0
                    return HerdingOutput(
                        robot1_goal=robot1_goal, robot2_goal=robot2_goal, fsm_state=fsm_state,
                        driver_id=driver_id, blocker_id=blocker_id,
                        target_position=target_state.position,
                        target_velocity=target_state.velocity,
                        escape_top3=list(escape_estimate.top_k_routes) if escape_estimate else [],
                        escape_directions=escape_estimate.directions if escape_estimate else None,
                        escape_probabilities=escape_estimate.probabilities if escape_estimate else None,
                        latency_ms=latency_ms, panic=False, role_swapped=role_swapped,
                        deadlock_release=False,
                        shaping_goal_prob=shaping.goal_prob,
                        shaping_goal_prob_single=shaping.goal_prob_single,
                    )

                if self.config.pressure_mode_enabled and clearance is not None:
                    # 압박 선분 모드: 두 로봇 목표를 하나의 기하학에서 같이 뽑는다.
                    # goal_direction은 표적 -> 목표 방향이어야 하므로
                    # direction_goal - target_pos를 쓴다(direction_goal은 벽을
                    # 고려한 가상 목표점이다).
                    pressure = compute_pressure_pair(
                        target_state.position, direction_goal - target_state.position,
                        self.grid_map, clearance,
                        self.config.robot_radius_m, self.config.robot_wall_clearance_m,
                        self.config.flee_reaction_distance_m, self.config.drive_distance_m,
                        half_angle_rad=np.radians(self.config.pressure_half_angle_deg),
                    )
                    pressure_coverage = pressure.coverage_fraction
                    # 두 끝점을 두 로봇에 배정한다: 서로 경로가 교차하지
                    # 않도록(=이동거리 합이 작도록) 가까운 쪽끼리 짝짓는다.
                    straight = (float(np.linalg.norm(observation.robot1_pos - pressure.point_a))
                                + float(np.linalg.norm(observation.robot2_pos - pressure.point_b)))
                    crossed = (float(np.linalg.norm(observation.robot1_pos - pressure.point_b))
                               + float(np.linalg.norm(observation.robot2_pos - pressure.point_a)))
                    if straight <= crossed:
                        robot1_goal, robot2_goal = pressure.point_a, pressure.point_b
                    else:
                        robot1_goal, robot2_goal = pressure.point_b, pressure.point_a
                    # 압박 모드는 Driver/Blocker 구분이 없으므로 이력현상/교착
                    # 상태를 들고 있을 이유가 없다 -- 모드가 꺼졌을 때 낡은
                    # 값을 이어받지 않도록 지운다.
                    self._committed_blocking_point = None
                    self._reset_deadlock_state()
                    latency_ms = (time.perf_counter() - start) * 1000.0
                    return HerdingOutput(
                        robot1_goal=robot1_goal, robot2_goal=robot2_goal, fsm_state=fsm_state,
                        driver_id=driver_id, blocker_id=blocker_id,
                        target_position=target_state.position,
                        target_velocity=target_state.velocity,
                        escape_top3=list(escape_estimate.top_k_routes) if escape_estimate else [],
                        escape_directions=escape_estimate.directions if escape_estimate else None,
                        escape_probabilities=escape_estimate.probabilities if escape_estimate else None,
                        latency_ms=latency_ms, panic=False, role_swapped=role_swapped,
                        deadlock_release=False, pressure_coverage=pressure_coverage,
                    )

                driving = compute_driving_point(
                    target_state.position, target_state.velocity, direction_goal,
                    driver_pos, self.planner_config,
                )
                panic = driving.is_panic

                if escape_estimate is not None:
                    raw_blocking_point = compute_blocking_point(
                        target_state.position, direction_goal, escape_estimate,
                        self.grid_map, self.planner_config,
                        geodesic_field=self._geodesic_field,
                        previous_point=self._committed_blocking_point,
                    )
                    # 방금 계산한 후보를 즉시 채택하지 않고 이력현상을 거친다
                    # -- _stabilize_blocking_point() 참고. resolve_separation은
                    # (안정화된 값이든 아니든) 매 주기 다시 적용해서, Driver가
                    # 그사이 움직였어도 최소 이격 거리는 항상 보장한다. 이
                    # 커밋된 지점 자체는 로봇 위치와 무관하게(target/goal/
                    # escape 분포로만) 계산되므로, 역할이 스왑돼도 새로
                    # 초기화할 필요가 없다 -- 물리적으로 "가야 할 지점"은
                    # 그대로이고 그 지점을 맡을 로봇만 바뀐다.
                    stable_blocking_point = self._stabilize_blocking_point(
                        raw_blocking_point, observation.sim_time_sec
                    )
                    blocking_point = resolve_separation(
                        driving.point, stable_blocking_point, self.config.min_robot_separation_m
                    )
                else:
                    # 타겟 추정치가 그리드 밖에 있어 escape distribution이
                    # 없고 의미 있는 blocking point도 없다. blocker는
                    # 제자리를 유지하고, 다음에 다시 의미 있는 추정치가
                    # 생기면 완전히 새로 커밋하도록 이력현상 상태를 지운다.
                    blocking_point = np.asarray(blocker_pos, dtype=float).copy()
                    self._committed_blocking_point = None

                # 교착(deadlock) 감지: Driver+Blocker가 서로 다른 각도에서
                # 밀다가 지정된 포획존이 아닌 곳에서 표적을 우연히 가둘 수
                # 있다 (트러블슈팅 노트 11-8). 표적이 오래 안 움직이는데
                # 아직 포획존 근처가 아니면 "몰이 성공"이 아니라 "우연히
                # 낀 것"이므로, Blocker를 뒤로 물러나게 해 협공 각도를
                # 풀어준다. 표적이 다시 움직이면(_update_deadlock_state가
                # 정지 누적을 리셋) 자동으로 정상 블로킹으로 복귀한다.
                in_deadlock = self._update_deadlock_state(target_state.position, observation.sim_time_sec)
                deadlock_release = in_deadlock and distance_to_goal > self.config.capture_radius_m
                if deadlock_release:
                    blocking_point = self._deadlock_release_point(
                        target_state.position, blocker_pos
                    )
                    self._committed_blocking_point = None

                if driver_id == 1:
                    robot1_goal, robot2_goal = driving.point, blocking_point
                else:
                    robot1_goal, robot2_goal = blocking_point, driving.point
            else:
                # IDLE / SEARCH / TRACK / CAPTURED: 위치를 유지한다. 새
                # 에피소드가 HERD로 다시 들어갈 때 지난 목표점을 이어받지
                # 않도록 이력현상 상태도 같이 지운다.
                self._committed_blocking_point = None
                self._reset_deadlock_state()
                robot1_goal = np.asarray(observation.robot1_pos, dtype=float).copy()
                robot2_goal = np.asarray(observation.robot2_pos, dtype=float).copy()

        latency_ms = (time.perf_counter() - start) * 1000.0
        return HerdingOutput(
            robot1_goal=robot1_goal, robot2_goal=robot2_goal, fsm_state=fsm_state,
            driver_id=driver_id, blocker_id=blocker_id, target_position=target_state.position,
            target_velocity=target_state.velocity,
            escape_top3=list(escape_estimate.top_k_routes) if escape_estimate else [],
            escape_directions=escape_estimate.directions if escape_estimate else None,
            escape_probabilities=escape_estimate.probabilities if escape_estimate else None,
            latency_ms=latency_ms, panic=panic, role_swapped=role_swapped,
            deadlock_release=deadlock_release,
        )
