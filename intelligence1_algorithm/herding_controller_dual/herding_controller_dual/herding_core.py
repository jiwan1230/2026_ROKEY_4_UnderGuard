# herding_controller_dual/herding_controller_dual/herding_core.py
"""칼만 필터, 마르코프 도주 모델, FSM, 역할 배정, 목표점 계산 — 이 모든 부품을 한데 모아 돌리는 파일.

지켜야 할 규칙: 이 파일과 여기서 불러오는 모든 파일은 ROS가 설치 안 된
컴퓨터에서도 그냥 파이썬으로 돌아가야 한다(ROS 기능을 아예 안 씀). 그래야
로봇 없이 노트북에서도 알고리즘 전체를 테스트할 수 있다.
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
    """config/herding_params.yaml에 적힌 설정값들을 파이썬 객체로 옮긴 것."""
    frame_id: str                      # 좌표계 이름(보통 "map")
    control_rate_hz: float             # 1초에 몇 번 계산할지 — herding_node.py의 타이머 주기
    capture_zone_x_m: float            # 덫 x좌표
    capture_zone_y_m: float            # 덫 y좌표
    capture_radius_m: float            # 이 거리 안이면 "덫 근처"
    capture_hold_sec: float            # 덫 근처에 이만큼 계속 있어야 "잡았다"고 확정
    grid_resolution_m: float           # 지도 한 칸의 실제 크기(m) — grid_map.py
    grid_width_cells: int              # 지도 가로 칸 수
    grid_height_cells: int             # 지도 세로 칸 수
    kf_process_noise: float            # 칼만필터: 예측을 얼마나 못 미더워할지 — target_estimator.py
    kf_measurement_noise: float        # 칼만필터: 카메라를 얼마나 못 미더워할지
    occlusion_timeout_sec: float       # 이 시간 넘게 못 보면 LOST
    markov_wall_follow_p: float        # 도주모델: 벽 따라가기 기본확률 — escape_model.py
    markov_wall_hug_p: float           # 도주모델: 벽에 딱 붙기 기본확률
    markov_center_p: float             # 도주모델: 방 한가운데로 갈 기본확률
    momentum_weight: float             # 도주모델: 가던 방향 관성 가중치
    robot_repulsion_weight: float      # 도주모델: 로봇 무서워하는 정도 가중치
    wall_detect_radius_cells: int      # 벽이 있는지 확인하는 범위(칸)
    escape_route_top_k: int            # 화면에 보여줄 도주경로 개수
    escape_concentration_threshold: float  # 이 이상이면 "도망갈 방향이 한쪽으로 쏠림"(CORNER 조건)
    drive_distance_m: float            # 미는 로봇이 쥐 뒤에서 유지하는 거리 — herding_planner.py
    flee_reaction_distance_m: float    # 쥐가 로봇을 보고 도망가기 시작하는 거리
    panic_distance_m: float            # 이보다 가까우면 미는 로봇이 놀라서 뒤로 물러남
    alignment_threshold: float         # 이 이상 방향이 맞으면 살살 밀기(easing)로 전환
    drive_distance_ease_factor: float  # 살살 밀 때 거리를 몇 배로 늘릴지
    block_lookahead_m: float           # 막는 로봇 목표점을 계산할 때 얼마나 앞을 내다볼지
    min_robot_separation_m: float      # 로봇 두 대가 유지해야 하는 최소 거리
    diffusion_rate: float              # 놓쳤을 때(LOST) 짐작이 퍼지는 속도 — occlusion_grid.py
    decay_factor: float                # 놓쳤을 때 짐작이 옅어지는 속도
    grid_origin_x_m: float = 0.0
    grid_origin_y_m: float = 0.0
    # 막는 로봇 목표점을 한번 정하면 잠깐 그대로 유지하는 시간 --
    # _stabilize_blocking_point() 참고. 왜 필요하냐면, 목표점이 매 순간
    # 홱홱 바뀌면(트러블슈팅 노트 10-6) 로봇이 어디로도 못 가고 제자리서
    # 맴돈다. 그렇다고 너무 길게 유지하면(10-7) 이번엔 로봇이 도착하기도
    # 전에 다음 목표로 또 바뀌는 반대 문제가 생긴다. 딱 맞는 값은
    # run_real_map_algo_suite로 직접 재보고 정했다.
    blocking_point_commit_sec: float = 1.0
    # 교착(deadlock) 감지 -- 트러블슈팅 노트 11-8 참고. 미는 로봇과 막는
    # 로봇이 서로 다른 각도에서 밀다가, 원래 목표인 덫이 아니라 방 안 다른
    # 구석에 쥐를 우연히 가둬버릴 때가 있다. 쥐가 이 시간(초) 이상 거의 안
    # 움직이는데 아직 덫 근처도 아니면, "몰이에 성공한 것"이 아니라 "우연히
    # 낀 것"으로 본다.
    deadlock_stall_sec: float = 3.0
    # 쥐가 이 반경(m) 안에서만 움직이면 "한 자리에 갇혀 있다"고 본다.
    # 순간순간의 움직임量이 아니라 반경으로 재는 이유: 벽에 부딪힐 때마다
    # 물리엔진이 쥐를 아주 살짝씩(0.01~0.03m) 밀어내는데, 이걸 "방금
    # 움직였다"로 치면 매번 정지 시간이 리셋돼서 실제로 갇혀 있어도 절대
    # 갇힌 걸로 안 잡힌다(트러블슈팅 노트 11-8).
    deadlock_drift_radius_m: float = 0.1
    # 교착으로 판정되면, 막는 로봇을 쥐로부터 이 거리(m)만큼 뒤로
    # 물러나게 해서 압박을 풀어준다. flee_reaction_distance_m보다 확실히
    # 커야 쥐가 "이제 안 무섭다"고 느낄 범위 밖으로 완전히 나간다.
    deadlock_release_distance_m: float = 1.0
    # 막는 로봇이 쥐로부터 이 거리(m)보다 멀면, "로봇이 무서워서 도망감"
    # 계산(escape_model.py)에서 막는 로봇은 아예 빼고 계산한다 --
    # 트러블슈팅 노트 11-9/11-10/12 참고. 기본값 inf는 이 기능을 끈
    # 것(원래 동작 그대로)과 같다. 최적값은 experiments/blocker_contribution_ablation.py로 찾는다.
    robot_repulsion_activation_distance_m: float = float("inf")
    # (이 패키지만의 기능) 미는 역할을 누가 맡을지 매 순간 다시 정하는
    # 로직의 "자주 안 바뀌게 하는 자물쇠" -- role_assigner.py::RoleAssigner
    # 참고. 두 로봇의 수고(거리 + 회전각*role_cost_turn_weight) 차이가
    # role_swap_margin 이상 나고, 마지막으로 바꾼 지 role_swap_cooldown_sec
    # 이상 지났을 때만 실제로 역할을 바꾼다. 예전에 다른 방(10x10m)에서
    # 검증했던 값을 그대로 가져왔고, 지금 이 방(5.3x7.35m) 기준으로 다시
    # 맞추지는 않았다.
    role_swap_margin: float = 0.5
    role_swap_cooldown_sec: float = 2.0
    role_cost_turn_weight: float = 0.3
    # --- 압박 선분 모드 (실험용, 기본은 꺼짐) ---
    # 켜면 미는 로봇/막는 로봇을 따로 안 정하고, 두 로봇 목표점을
    # compute_pressure_pair()로 한 번에 같이 정한다. 목적은 성공률을
    # 더 올리는 게 아니라, "로봇 B가 빠지면 진로의 절반이 뻥 뚫린다"는
    # 구조를 만들어서 로봇 B가 진짜 쓸모 있다는 걸 숫자로 보여주기
    # 위한 것이다.
    pressure_mode_enabled: bool = False
    # 로봇/쥐의 실제 크기 -- 압박 선분 배치에만 쓴다 (TurtleBot 4 실측값).
    robot_radius_m: float = 0.175
    robot_wall_clearance_m: float = 0.03
    # 압박 선분에서 두 로봇을 덫 반대편 기준 좌우로 얼마나 벌릴지(도).
    # 크게 벌리면 넓은 구간을 막을 수 있지만 미는 힘은 약해지는
    # 트레이드오프가 있다.
    pressure_half_angle_deg: float = 45.0
    # 도주 분포 직접 최적화 모드 (실험용, 기본은 꺼짐). 켜면 "쥐가 덫
    # 쪽으로 갈 확률이 제일 높아지는" 자리에 두 로봇을 같이 놓는다 --
    # 미는/막는 구분이 없다.
    shaping_mode_enabled: bool = False
    # 수비-쓸기 모드 (실험용, 기본은 꺼짐). 켜면 한 로봇은 쥐를 직접
    # 쫓지 않고 "도망갈 수 있는 가장 좁은 길목"에 미리 가서 지키고, 다른
    # 로봇이 쥐를 덫 쪽으로 민다. 지금까지 다른 방식은 전부 두 로봇이
    # 쥐를 쫓아다니기만 했는데, 쥐보다 느린 로봇은 영원히 뒤만 쫓게
    # 되는 문제가 있어서 시도해본 방식이다.
    guard_mode_enabled: bool = False
    # 수비 지점을 이만큼 옮겨야만 실제로 갱신한다(너무 자주 안 바뀌게).
    guard_commit_distance_m: float = 0.5
    # 엔드게임 협공 (이 프로젝트의 핵심 기능). 쥐가 덫
    # endgame_trigger_radius_m 안까지 들어오면, 두 로봇이 덫 반대편
    # 좌우로 갈라져서 쥐가 덫 쪽으로밖에 못 가게 만든다. "실패 사례를
    # 다 뜯어봤더니 92건 중 91건이 덫 0.6m 안까지 왔다가 마지막 7cm를
    # 못 넘고 옆으로 빠져나간 것"이라는 걸 발견하고 만든 기능이다.
    endgame_pincer_enabled: bool = False
    endgame_trigger_radius_m: float = 0.8
    endgame_half_angle_deg: float = 60.0
    # 쥐가 트리거 반경 안에 이 시간(초) 이상 있었는데도 안 잡히면 그때야
    # 협공으로 전환한다. 0으로 하면 반경에 들어오자마자 바로 협공한다.
    # 순순히 잘 밀리는 쥐는 그냥 밀기만 해도 바로 잡히는데, 거기다 대고
    # 매번 협공부터 하면 오히려 두 로봇이 갈라지느라 미는 힘이 약해져서
    # 더 나빠진다(실측: 90.7%->72.7%) -- 그래서 "일단 밀어보고, 안 되면
    # 그때 협공"으로 조건을 걸었다.
    endgame_stall_sec: float = 3.0

    def __post_init__(self) -> None:
        """설정값 조합이 몰이를 아예 멈춰버리는 경우를 미리 걸러낸다.

        미는 로봇은 자기 목표점(driving point)에 도착하면 거기 멈춘다.
        쥐가 이미 덫 쪽을 보고 있으면 살살 밀기(easing)가 걸려서 로봇이
        더 멀리서 멈추는데, 그 거리가 drive_distance_m *
        drive_distance_ease_factor다. 이 거리가 "쥐가 로봇을 보고
        도망가기 시작하는 거리"(flee_reaction_distance_m)보다 크거나
        같으면 어떻게 될까? 로봇이 멈춘 자리가 쥐를 무섭게 하기엔 너무
        멀어서, 쥐는 평생 도망을 안 가고 알고리즘 전체가 멈춰버린다 --
        실제로 재봤더니 성공률이 83%에서 2.5%로 폭락했다. 그래서 이
        조합은 실행 중에 조용히 성능만 나빠지게 두지 않고, 설정을 만드는
        바로 이 순간에 크게 에러를 내서 바로 알아챌 수 있게 한다.
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
    """이번 한 순간에 로봇/카메라가 알려준 정보 전부."""
    target_measurement: np.ndarray | None  # 이번에 본 쥐 위치 (x,y), 못 봤으면 None
    robot1_pos: np.ndarray
    robot2_pos: np.ndarray
    robot1_heading: np.ndarray
    robot2_heading: np.ndarray
    occupancy: np.ndarray | None   # 지도(OccupancyGrid) 배열, 아직 안 왔으면 None
    sim_time_sec: float             # 지금까지 흐른 총 시간 (여러 타이머 판정에 씀)
    dt: float                       # 이번 한 번의 시간 간격(초)


@dataclass
class HerdingOutput:
    """이번 한 순간 계산 결과 — herding_node.py가 이대로 로봇에 명령을 내린다."""
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
    # 압박 선분 모드에서만 값이 들어온다: 쥐가 지나갈 길 중 두 로봇이 막고 있는 비율.
    pressure_coverage: float | None = None
    # 도주 분포 최적화 모드에서만: 로봇 두 대로 막았을 때와 한 대만 있었다면
    # 어땠을지를 비교한 확률 (두 번째 로봇이 실제로 얼마나 도움이 됐는지 보여줌).
    shaping_goal_prob: float | None = None
    shaping_goal_prob_single: float | None = None
    # 수비-쓸기 모드에서만: 수비 로봇이 지키는 길목의 폭(좁을수록 잘 막힘).
    guard_corridor_width_m: float | None = None
    # 이번 순간 목표점이 엔드게임 협공으로 나온 것인지. 협공은 마지막
    # 순간에만 켜지므로 대부분은 False다. 화면에 "지금 협공 중"이라고
    # 보여주거나, 검증할 때 실제로 몇 번 발동했는지 세는 데 쓴다.
    pincer_active: bool = False


class HerdingCore:
    """지도, 칼만 필터, 도주 모델, 목표점 계산, 상태표, 재탐색 그리드를 전부 연결하는 조종실.

    (이 패키지만의 기능) 어느 로봇이 미는 역할(Driver)을 할지는 매 순간
    다시 정해질 수 있다: `RoleAssigner`가 "거리 + 돌아야 하는 각도"를 계산해서
    비교하고, 너무 자주 안 바뀌게 자물쇠(margin+cooldown)를 걸어둔다.
    "처음에 누가 쥐를 발견해서 뛰어드는가"는 여전히 상위 시스템이 정하지만
    (그 처음 배정을 RoleAssigner가 그대로 받아들인다), 일단 둘 다 몰이를
    시작하면 이 클래스가 실시간으로 역할을 다시 정한다. 이 클래스는 로봇
    두 대 모두에게 직접 명령한다 (구버전인 `herding_controller`는 반대로
    역할을 처음부터 끝까지 고정하고 로봇 1은 아예 조종하지 않는다 -- 실제
    운용 사정 때문에 그렇게 정해졌고 지금도 그대로 쓰인다. 자세한 배경은
    herding_controller_트러블슈팅_노트.md 10번/13번 항목 참고).
    """

    def __init__(self, config: HerdingConfig) -> None:
        self.config = config
        self.goal_pos = np.array([config.capture_zone_x_m, config.capture_zone_y_m], dtype=float)  # 덫 좌표
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
        # 벽을 피해서 목표 방향을 알려주는 지도 (geodesic_field.py). 덫
        # 위치는 설정값이라 미션 내내 안 바뀌므로, 실제 지도가 도착한 뒤
        # 딱 한 번만 계산해서 저장해둔다 -- 매번(1초에 5번) 이 계산을 새로
        # 하면 시간이 너무 오래 걸린다. `_geodesic_ready`는 "아직 지도가
        # 안 왔을 때"와 "지도는 왔는데 덫 좌표가 지도 밖이라 계산에
        # 실패했을 때"를 구분해서, 후자는 매번 다시 시도하지 않게 한다.
        self._geodesic_field: GeodesicField | None = None
        self._geodesic_ready = False
        # 압박 선분 배치용, 각 칸에서 가장 가까운 벽까지의 거리(미터)를
        # 저장해둔 지도. 실제 지도가 온 뒤 한 번만 계산한다.
        self._clearance_m = None
        # 수비 지점을 얼마나 유지할지 상태 -- guard_commit_distance_m 참고.
        self._committed_guard_point = None
        # 엔드게임: 쥐가 트리거 반경 안에 처음 들어온 시각을 기록해둔다.
        self._endgame_entered_sec = None
        # 막는 로봇 목표점을 얼마나 유지할지 상태 -- 아래 _stabilize_blocking_point() 참고.
        self._committed_blocking_point: np.ndarray | None = None
        self._committed_blocking_time: float = 0.0
        # 교착(deadlock) 감지 상태 -- 아래 _update_deadlock_state() 참고.
        self._deadlock_anchor_position: np.ndarray | None = None
        self._deadlock_anchor_time_sec: float = 0.0

    # ------------------------------------------------------------------ #
    # 지도 관련 도우미 함수들                                                #
    # ------------------------------------------------------------------ #

    def _cell_or_none(self, position: np.ndarray) -> tuple[int, int] | None:
        """실제 좌표(m)를 지도 칸 번호로 바꾼다. 지도 밖이거나 이상한 값이면 None."""
        try:
            return self.grid_map.world_to_cell(float(position[0]), float(position[1]))
        except (ValueError, TypeError):
            return None

    def _clamped_cell_or_none(self, position: np.ndarray) -> tuple[int, int] | None:
        """실제 좌표를 지도 칸 번호로 바꾸되, 지도 밖으로 나가면 지도 가장자리로 끌어당긴다."""
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
        """실제 지도가 도착했으면, 덫까지 가는 길 안내 지도를 딱 한 번만 계산해서 저장한다."""
        if self._geodesic_ready:
            return
        if not self.grid_map.obstacle_mask.any():
            return  # 아직 진짜 지도를 못 받음 (전부 빈칸) -- 다음 순간에 다시 시도
        try:
            goal_row, goal_col = self.grid_map.world_to_cell(*self.goal_pos)
        except ValueError:
            # 덫 좌표가 지도 밖 -- 설정이 잘못된 것이라 다시 시도해도 똑같이 실패한다.
            self._geodesic_ready = True
            return
        self._geodesic_field = GeodesicField(self.grid_map, goal_row, goal_col)
        self._geodesic_ready = True

    def _ensure_clearance_field(self):
        """"각 칸에서 가장 가까운 벽까지 거리"를 담은 지도를, 실제 지도가 온 뒤 한 번만 계산한다.

        압박 선분(compute_pressure_pair)이 "여기 로봇이 서도 되나"를
        판단할 때 쓴다. 위 geodesic 지도와 같은 이유로 매번 다시 계산하지
        않는다.
        """
        if self._clearance_m is None and self.grid_map.obstacle_mask.any():
            from scipy import ndimage
            self._clearance_m = (
                ndimage.distance_transform_edt(~self.grid_map.obstacle_mask)
                * self.grid_map.config.resolution_m
            )
        return self._clearance_m

    # 딱 한 지점의 방향만 보는 게 아니라, 실제 갈 수 있는 길을 따라 이만큼
    # 앞을 미리 내다본다. block_lookahead_m과 비슷한 크기로 맞춰서 "코너
    # 하나 정도는 미리 본다"는 느낌에 맞췄다 -- 배포 환경마다 바뀔 값이
    # 아니라 알고리즘 내부 구현 디테일이라 설정 파일로 빼지 않았다.
    _WAYPOINT_LOOKAHEAD_M = 1.5

    def _direction_goal(self, position: np.ndarray) -> np.ndarray:
        """"어느 쪽으로 밀지" 계산할 때 실제 덫 좌표 대신 쓸, 벽을 고려한 기준점.

        길 안내 지도가 준비돼 있으면, 실제 갈 수 있는 길을 따라
        `_WAYPOINT_LOOKAHEAD_M`만큼 앞선 지점(코너를 미리 반영한 지점)을
        준다. 그게 안 되면(지도 밖 등) 그 자리에서 방향만 본 값을 쓰고,
        그것도 안 되면 그냥 진짜 덫 좌표를 쓴다 -- 미는/막는 로봇 계산은
        둘 다 이 값과 쥐 위치의 "차이"에서 방향만 뽑아 쓰므로 이렇게
        단계적으로 물러나도 안전하다.
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
        """막는 로봇의 새 목표점 후보를 바로 받아들이지 않고, 잠깐은 이전 값을 그대로 유지한다.

        compute_blocking_point()는 "1.8m 앞 지점이 벽인가 아닌가"로
        어느 방향을 지킬지 정하는데, 쥐가 그 경계선 바로 근처에 있으면
        몇 cm만 움직여도 앞 지점이 벽 반대편으로 넘어가면서 완전히 다른
        방향이 골라져 버릴 수 있다. 도망 확률 예측 자체는 안정적인데도
        (트러블슈팅 노트 10-6: 제일 유력한 방향은 그대로인데 목표 좌표만
        1.3m 넘게 순간이동) 목표점만 계속 요동치면, 로봇이 어느 쪽으로도
        못 가고 제자리서 맴돈다. role_assigner.py에서 역할이 자꾸
        바뀌던 것과 똑같은 문제라 같은 해법(자물쇠)을 쓴다.

        `config.blocking_point_commit_sec`이 아직 안 지났으면 이전에
        정했던 점을 그대로 돌려주고, 그 이상 지났으면(또는 아직 한 번도
        안 정했으면) 새 후보를 받아들이고 시각을 기록한다. 계산 자체는
        매 순간 계속 새로 하지만(그래야 쥐가 진짜로 멀리 움직였을 때
        반영되니까), 실제로 로봇에 전달되는 값은 이 함수를 거쳐서
        안정화된 값이다.

        주의(트러블슈팅 노트 10-7): 이 유지 시간을 너무 길게 잡으면
        반대로 "목표가 멀리 바뀐 뒤 로봇이 도착하기도 전에 또 바뀌는"
        지연 문제가 생긴다 -- 너무 자주 바뀌는 것과 너무 안 바뀌는 것
        사이의 균형이므로, 이 값을 바꿀 땐 반드시
        test/run_validation.py::run_real_map_algo_suite()로 다시 검증한다.
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
        """쥐가 한 자리(반경 deadlock_drift_radius_m 안)에 오래 머물렀는지 확인한다.

        기준점을 하나 잡아두고, 쥐가 그 반경을 실제로 벗어날 때만 기준점을
        새로 잡는다 -- 매 순간의 아주 작은 움직임으로 판단하면, 벽에
        부딪혀 물리엔진이 매번 살짝씩 밀어내는 것까지 "움직였다"고 쳐서
        절대 갇힌 것으로 안 잡힌다(트러블슈팅 노트 11-8, 시드 1200008로
        직접 확인함).
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
        """막는 로봇을 쥐로부터 deadlock_release_distance_m만큼 뒤로 물러나게 한다.

        쥐 위치에서 (지금 막는 역할인 로봇의) 지금 위치를 잇는 직선
        방향으로 물러나므로, 어느 각도에서 압박하고 있었든 그 각도
        그대로 벌어지기만 한다(엉뚱한 새 방향으로 안 튐). 이 패키지는
        역할이 매 순간 바뀔 수 있어서, 부르는 쪽(step())이 그 순간 실제
        막는 로봇의 위치를 넘겨준다.
        """
        away = blocker_pos - target_pos
        norm = np.linalg.norm(away)
        away = away / norm if norm > 1e-6 else np.array([1.0, 0.0])
        return target_pos + away * self.config.deadlock_release_distance_m

    def _reset_occlusion_memory(self) -> None:
        """쥐를 놓쳤던 기록을 지워서, 다음에 또 놓치면 처음부터 새로 찾기 시작하게 한다."""
        self._last_known_cell = None
        self._last_known_point = None
        self._occlusion_seeded = False

    def _search_point(self, fallback: np.ndarray) -> np.ndarray:
        """쥐를 놓쳤을 때, "여기 있을 것 같다"는 짐작 지도에서 제일 유력한 지점을 고른다."""
        if float(self.occlusion_grid.belief.max()) > 0.0:
            row, col = self.occlusion_grid.best_guess_cell()
            return np.array(self.grid_map.cell_to_world(row, col), dtype=float)
        # 짐작이 다 옅어졌거나(시간이 너무 지남) 처음 심은 자리가 벽이었던
        # 경우: 그냥 놔두면 지도 구석(0,0)을 가리키게 된다. 대신 마지막으로
        # 실제로 봤던 위치를 다시 살펴본다.
        if self._last_known_point is not None:
            return self._last_known_point.copy()
        return np.asarray(fallback, dtype=float).copy()

    def _current_roles(self) -> tuple[int, int]:
        """지금 확정돼 있는 역할 배정을 그대로 읽어온다."""
        driver_id = self.role_assigner._driver_id
        return driver_id, (2 if driver_id == 1 else 1)

    def _nominal_driving_point(
        self, target_pos: np.ndarray, target_vel: np.ndarray, direction_goal: np.ndarray
    ) -> np.ndarray:
        """"어느 로봇이 미는 역할을 맡을지" 비교할 때 쓸, 특정 로봇에 치우치지 않은 기준 목표점.

        compute_driving_point()는 그 로봇이 너무 가까이 있으면(panic
        거리 안) "일단 물러나기" 지점을 돌려준다. 그런데 역할을 정하기
        위해 특정 로봇 위치로 미리 계산해버리면, 우연히 가까이 있던
        로봇 쪽으로 배정이 쏠려버린다. 그래서 panic 거리 밖이 확실한
        가상의 기준 위치에서 계산해서, 두 로봇 모두에게 공평하게 같은
        기하학적 목표점을 준다.
        """
        away = target_pos - direction_goal   # 덫 → 쥐 방향, "미는 로봇이 서는 쪽"
        norm = float(np.linalg.norm(away))
        away = away / norm if norm > 1e-6 else np.array([1.0, 0.0])  # 방향만 남기고 길이는 1로
        # panic 거리 + 평소 미는 거리 + 1.0m 만큼 떨어뜨려, 확실히 panic 반경 밖의
        # 가상 위치를 만든다(그래야 compute_driving_point가 물러나기 대신 평소 계산을 함).
        reference = target_pos + away * (self.config.panic_distance_m + self.config.drive_distance_m + 1.0)
        return compute_driving_point(
            target_pos, target_vel, direction_goal, reference, self.planner_config
        ).point

    # ------------------------------------------------------------------ #
    # 메인 루프 — 1초에 5번(control_rate_hz) 이 함수가 통째로 다시 불린다      #
    # ------------------------------------------------------------------ #

    def step(self, observation: Observation) -> HerdingOutput:
        """이번 한 순간의 계산을 전부 실행하고, 두 로봇이 가야 할 목표점을 돌려준다."""
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
            self.estimator.predict(observation.dt)      # 먼저 "아까 가던 대로" 밀어보고
            self.estimator.update(observation.target_measurement)  # 방금 본 실제 위치로 고친다
            self._first_observation_seen = True
        elif self._first_observation_seen:
            self.estimator.predict(observation.dt)  # 이번엔 못 봤음 — 예측만 (한 번이라도 본 뒤부터 의미 있음)

        target_state = self.estimator.get_state()
        kf_converged = self._first_observation_seen and not target_state.is_lost  # 본 적 있고 최근에도 봤음

        # escape_model.compute()가 FSM 판단(도망 방향이 쏠렸는지)보다
        # 먼저 실행돼야 하는데, 이 시점엔 이번 순간의 역할이 아직 새로
        # 계산되지 않았다 -- 그래서 일단 직전 순간까지 정해져 있던 역할을
        # 그대로 쓴다. 아래 HERD/CORNER 부분에서 역할을 다시 정하면
        # driver_id/blocker_id가 새 값으로 덮어써진다(역할이 실제로
        # 바뀌는 그 찰나에만 한 박자(0.2초) 늦게 반영되는 정도의 사소한 차이다).
        driver_id, blocker_id = self._current_roles()

        # 도주 모델은 쥐가 있는 지도 칸을 기준으로 계산하므로, 칼만
        # 필터가 지도 위의 진짜 위치를 갖고 있을 때만 돌릴 수 있다. 한
        # 번도 못 본 상태에서는 칼만 필터 값이 전부 0이라 의미가 없다.
        escape_estimate = None
        if kf_converged and self._cell_or_none(target_state.position) is not None:
            escape_estimate = self.escape_model.compute(
                target_state.position, target_state.velocity,
                [observation.robot1_pos, observation.robot2_pos],
                blocker_index=blocker_id - 1,
            )

        distance_to_goal = float(np.linalg.norm(target_state.position - self.goal_pos)) \
            if self._first_observation_seen else float("inf")  # 한 번도 못 봤으면 "무한히 멀다"고 침
        escape_concentrated = bool(
            escape_estimate is not None
            and escape_estimate.probabilities.max() >= self.config.escape_concentration_threshold
        )  # 8방향 중 제일 높은 확률이 기준치 이상 = 도망갈 방향이 한쪽으로 쏠려 있음

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
            # LOST일 때는 재탐색 지도만 쓴다: 도주 모델도, 목표점 계산도,
            # 역할 재배정도 전혀 안 한다. 두 로봇은 그냥 "짐작 지도에서
            # 제일 유력한 곳"을 훑고 지나갈 뿐이다. 역할은 새로 안 정하고
            # (위에서 이미 읽어둔 값을 그대로 쓰고) 마지막으로 확정됐던
            # 배정만 그대로 읽는다.
            if not self._occlusion_seeded:
                self._last_known_point = np.asarray(target_state.position, dtype=float).copy()
                self._last_known_cell = self._clamped_cell_or_none(target_state.position)
                if self._last_known_cell is not None:
                    self.occlusion_grid.seed(*self._last_known_cell)
                    self._occlusion_seeded = True
            self.occlusion_grid.step(observation.dt)
            search_point = self._search_point(target_state.position)
            # 두 로봇이 똑같은 지점으로 수렴하니까, 서로 안 겹치게 두 번째
            # 로봇의 목표는 살짝 옆으로 밀어준다.
            offset_point = resolve_separation(search_point, search_point, self.config.min_robot_separation_m)
            if driver_id == 1:
                robot1_goal, robot2_goal = search_point, offset_point
            else:
                robot1_goal, robot2_goal = offset_point, search_point
            # 다시 발견해서 HERD로 돌아갈 때 LOST 이전의 낡은 목표점을
            # 이어받지 않도록, 유지해두던 값들을 여기서 지운다.
            self._committed_blocking_point = None
            self._committed_guard_point = None
            self._endgame_entered_sec = None
            self._reset_deadlock_state()
        else:
            self._reset_occlusion_memory()
            if fsm_state in (FSMState.HERD, FSMState.CORNER):
                # 목표점을 계산할 때 진짜 덫 좌표 대신 direction_goal(벽을
                # 고려한 가상의 기준점, geodesic_field.py)을 넘긴다: 두
                # 계산 함수 모두 이 값과 쥐 위치의 "차이"에서 방향만
                # 뽑아 쓰므로, 직선이 벽을 뚫고 지나가는 상황에서도 실제로
                # 갈 수 있는 방향을 가리키게 된다 (아직 이 지도가 없으면
                # 그냥 진짜 덫 좌표를 쓴다).
                direction_goal = self._direction_goal(target_state.position)

                # 이번 순간 누가 미는 역할을 할지 다시 정한다: 특정
                # 로봇에 치우치지 않은 기준 목표점(_nominal_driving_point)을
                # 놓고 두 로봇의 수고(거리+회전)를 비교해서, 역할이
                # 자주 안 바뀌게 자물쇠를 건 채로 RoleAssigner가 결정한다.
                candidate = self._nominal_driving_point(
                    target_state.position, target_state.velocity, direction_goal
                )
                previous_driver = self.role_assigner._driver_id if self._roles_ever_assigned else None
                driver_id, blocker_id = self.role_assigner.assign(
                    observation.robot1_pos, observation.robot2_pos,
                    observation.robot1_heading, observation.robot2_heading,
                    candidate, observation.sim_time_sec,
                )
                # 맨 처음 한 번은 "역할을 바꾼 것"이 아니라 "역할을 처음
                # 정한 것"이므로 role_swapped로 안 친다.
                role_swapped = previous_driver is not None and driver_id != previous_driver
                self._roles_ever_assigned = True

                driver_pos = observation.robot1_pos if driver_id == 1 else observation.robot2_pos
                blocker_pos = observation.robot2_pos if driver_id == 1 else observation.robot1_pos

                clearance = self._ensure_clearance_field()
                if self.config.endgame_pincer_enabled and clearance is not None:
                    # 트리거 반경 안에 머문 시간을 계속 더한다 -- 벗어나면 처음부터 다시.
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
                        # 두 로봇을 point_a/point_b에 그대로(straight) 보낼지
                        # 서로 바꿔서(crossed) 보낼지는, 둘이 이동해야 할
                        # 거리의 합이 더 작은 쪽으로 정한다 — 그래야 두
                        # 로봇의 경로가 서로 엇갈리지 않는다 (아래
                        # guard/shaping/pressure 부분도 다 같은 방식).
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
                            deadlock_release=False, pincer_active=True,
                        )

                if self.config.guard_mode_enabled and clearance is not None:
                    guard = compute_guard_point(
                        target_state.position, direction_goal - target_state.position,
                        self.grid_map, clearance,
                        self.config.robot_radius_m + self.config.robot_wall_clearance_m,
                    )
                    if guard is not None:
                        # 자물쇠: 새 길목이 충분히 멀어졌을 때만 갱신한다.
                        if (self._committed_guard_point is None
                                or float(np.linalg.norm(guard.point - self._committed_guard_point))
                                > self.config.guard_commit_distance_m):
                            self._committed_guard_point = guard.point.copy()
                        guard_point = self._committed_guard_point
                        # 쓸어 미는 역할은 이미 검증된 미는 로봇 계산을 그대로 쓴다.
                        sweep = compute_driving_point(
                            target_state.position, target_state.velocity, direction_goal,
                            driver_pos, self.planner_config,
                        )
                        panic = sweep.is_panic
                        # 배정: 각자 가까운 쪽을 맡아서 경로가 안 엇갈리게 한다.
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
                    # (위 협공 부분과 같은 방식) 이동거리 합이 더 작은 배정을 고른다
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
                    # 압박 선분 모드: 두 로봇 목표를 하나의 계산에서 같이 뽑는다.
                    # goal_direction은 "쥐 → 목표" 방향이어야 하므로
                    # direction_goal - target_pos를 쓴다(direction_goal은
                    # 벽을 고려한 가상 목표점).
                    pressure = compute_pressure_pair(
                        target_state.position, direction_goal - target_state.position,
                        self.grid_map, clearance,
                        self.config.robot_radius_m, self.config.robot_wall_clearance_m,
                        self.config.flee_reaction_distance_m, self.config.drive_distance_m,
                        half_angle_rad=np.radians(self.config.pressure_half_angle_deg),
                    )
                    pressure_coverage = pressure.coverage_fraction
                    # 두 끝점을 두 로봇에 배정한다: 경로가 안 엇갈리게
                    # (=이동거리 합이 작게) 가까운 쪽끼리 짝짓는다.
                    straight = (float(np.linalg.norm(observation.robot1_pos - pressure.point_a))
                                + float(np.linalg.norm(observation.robot2_pos - pressure.point_b)))
                    crossed = (float(np.linalg.norm(observation.robot1_pos - pressure.point_b))
                               + float(np.linalg.norm(observation.robot2_pos - pressure.point_a)))
                    if straight <= crossed:
                        robot1_goal, robot2_goal = pressure.point_a, pressure.point_b
                    else:
                        robot1_goal, robot2_goal = pressure.point_b, pressure.point_a
                    # 압박 모드는 미는/막는 구분이 없으니 이력 상태를 들고
                    # 있을 이유가 없다 -- 모드를 껐을 때 낡은 값을 이어받지
                    # 않도록 지운다.
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

                driving = compute_driving_point(   # 지금 미는 역할인 로봇(driver_id)의 목표점
                    target_state.position, target_state.velocity, direction_goal,
                    driver_pos, self.planner_config,
                )
                panic = driving.is_panic  # 너무 가까워져서 미는 로봇이 뒤로 물러나는 중인지

                if escape_estimate is not None:
                    raw_blocking_point = compute_blocking_point(
                        target_state.position, direction_goal, escape_estimate,
                        self.grid_map, self.planner_config,
                        geodesic_field=self._geodesic_field,
                        previous_point=self._committed_blocking_point,
                    )
                    # 방금 계산한 후보를 바로 안 쓰고 자물쇠를 거친다 --
                    # _stabilize_blocking_point() 참고. resolve_separation은
                    # (자물쇠를 거쳤든 아니든) 매 순간 다시 적용해서, 미는
                    # 로봇이 그사이 움직였어도 최소 거리는 항상 지킨다. 이
                    # 목표점 자체는 로봇 위치와 상관없이(쥐/덫/도주확률로만)
                    # 계산되므로, 역할이 바뀌어도 새로 계산할 필요가 없다 --
                    # "가야 할 자리"는 그대로고 그 자리를 맡을 로봇만 바뀐다.
                    stable_blocking_point = self._stabilize_blocking_point(
                        raw_blocking_point, observation.sim_time_sec
                    )
                    blocking_point = resolve_separation(
                        driving.point, stable_blocking_point, self.config.min_robot_separation_m
                    )
                else:
                    # 쥐 추정 위치가 지도 밖이라 도주 확률도 없고 의미 있는
                    # 막는 로봇 목표점도 없다. 막는 로봇은 일단 제자리를
                    # 지키고, 다음에 다시 의미 있는 값이 생기면 완전히
                    # 새로 정하도록 자물쇠 상태를 지운다.
                    blocking_point = np.asarray(blocker_pos, dtype=float).copy()
                    self._committed_blocking_point = None

                # 교착(deadlock) 감지: 미는 로봇과 막는 로봇이 서로 다른
                # 각도에서 밀다가, 지정된 덫이 아닌 다른 곳에서 쥐를 우연히
                # 가둬버릴 수 있다 (트러블슈팅 노트 11-8). 쥐가 오래 안
                # 움직이는데 아직 덫 근처도 아니면 "몰이 성공"이 아니라
                # "우연히 낀 것"이므로, 막는 로봇을 뒤로 물러나게 해서
                # 압박을 풀어준다. 쥐가 다시 움직이면(_update_deadlock_state가
                # 정지 시간을 리셋) 자동으로 평소 방식으로 돌아온다.
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
                # IDLE / SEARCH / TRACK / CAPTURED: 그냥 지금 자리를 지킨다.
                # 다음에 새로 HERD에 들어갈 때 이전 목표점을 이어받지 않도록
                # 자물쇠 상태도 같이 지운다.
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
