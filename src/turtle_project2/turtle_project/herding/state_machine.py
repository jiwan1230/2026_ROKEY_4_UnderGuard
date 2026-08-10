# herding_controller/herding_controller/state_machine.py
"""6단계 herding FSM: IDLE -> SEARCH -> TRACK -> HERD -> CORNER -> CAPTURED, LOST 복구 포함."""
from dataclasses import dataclass
from enum import Enum, auto


class FSMState(Enum):
    """herding controller의 동작 모드."""
    IDLE = auto()
    SEARCH = auto()
    TRACK = auto()
    HERD = auto()
    CORNER = auto()
    CAPTURED = auto()
    LOST = auto()


@dataclass
class FSMInputs:
    """FSM이 이번 주기의 다음 전이를 결정하는 데 필요한 신호들."""
    target_observed: bool
    kf_converged: bool
    distance_to_goal_m: float
    capture_radius_m: float
    escape_prob_concentrated: bool
    occlusion_elapsed_sec: float
    occlusion_timeout_sec: float
    capture_hold_required_sec: float
    dt: float


class HerdingStateMachine:
    """주기마다의 센서/estimator 신호를 받아 herding FSM 상태를 진행시킨다."""

    def __init__(self) -> None:
        self._state = FSMState.IDLE
        self._capture_hold_elapsed_sec = 0.0

    @property
    def state(self) -> FSMState:
        """현재 FSM 상태."""
        return self._state

    def step(self, inputs: FSMInputs) -> FSMState:
        """이번 제어 주기의 다음 FSM 상태를 계산하여 저장한다."""
        state = self._state

        # 1. 상태별 "전진" 전이. 여기서는 상태를 뒤로 되돌리는 로직(관측 끊김 ->
        # LOST, 재관측 -> CAPTURED 취소 등)을 넣지 않는다: 그건 모두 아래
        # 2번/3번 블록에서 상태와 무관하게 한 번에 처리한다. 그래야 "관측이
        # 끊기면 어떤 상태에서도 LOST로 간다"는 규칙을 상태 개수만큼
        # 중복해서 적을 필요가 없다.
        if state == FSMState.IDLE:
            # IDLE은 노드가 막 켜졌을 때의 부트스트랩 값일 뿐, 대기해야 할
            # 이유가 없다. 다음 주기에 바로 SEARCH로 넘어간다.
            state = FSMState.SEARCH
        elif state == FSMState.SEARCH:
            if inputs.target_observed:
                state = FSMState.TRACK
        elif state == FSMState.TRACK:
            # 타겟이 "관측"되는 것과 칼만 필터가 "수렴"하는 것은 별개다:
            # 첫 관측 직후에는 속도 추정치가 아직 신뢰할 수 없으므로,
            # planner에게 이 상태를 넘기기 전에 kf_converged까지 함께 요구한다.
            if inputs.target_observed and inputs.kf_converged:
                state = FSMState.HERD
        elif state == FSMState.HERD:
            # CORNER로 넘어가려면 "포획 반경 안"이라는 위치 조건뿐 아니라
            # "도주 확률이 한쪽 방향에 집중되어 있다"는 조건도 함께 필요하다.
            # 위치만 보고 CORNER로 넘어가면, 반경 안에 있어도 여전히 사방으로
            # 도망칠 여지가 있는 타겟을 성급하게 "구석에 몰렸다"고 오판하게 된다.
            in_capture_zone = inputs.distance_to_goal_m <= inputs.capture_radius_m
            if in_capture_zone and inputs.escape_prob_concentrated:
                state = FSMState.CORNER
        elif state == FSMState.CORNER:
            # 위 조건 중 하나라도 깨지면(반경을 벗어나거나, 도주 확률이 다시
            # 퍼지면) 즉시 HERD로 되돌아간다 -- CORNER 전용 별도 하강 조건이
            # 없는 이유는, "구석에 몰림"이 HERD 판정 조건의 부정으로 정확히
            # 정의되기 때문이다.
            in_capture_zone = inputs.distance_to_goal_m <= inputs.capture_radius_m
            if not (in_capture_zone and inputs.escape_prob_concentrated):
                state = FSMState.HERD
        elif state == FSMState.LOST:
            # LOST에서는 재관측되자마자 곧장 HERD가 아니라 TRACK으로 돌아간다:
            # 오랫동안 관측이 끊겨 있었으므로 KF가 다시 수렴할 시간이
            # 필요하고, 그건 TRACK의 kf_converged 게이트가 담당한다.
            if inputs.target_observed:
                state = FSMState.TRACK

        # 2. 관측 끊김 감시 (상태 무관). TRACK/HERD/CORNER 중이었다면, 그
        # 사이에 위에서 이미 다른 상태로 전이했더라도 occlusion timeout이
        # 지났으면 LOST로 덮어쓴다 -- 시야 차단은 "어디까지 진행했었는지"와
        # 무관하게 최우선으로 처리해야 하는 안전 조건이다.
        if state in (FSMState.TRACK, FSMState.HERD, FSMState.CORNER):
            if inputs.occlusion_elapsed_sec > inputs.occlusion_timeout_sec:
                state = FSMState.LOST

        # 3. 포획 유지 타이머 (상태 무관). "반경 안에 몇 초 연속으로
        # 있었는가"로 판정해야, 타겟이 반경 경계를 스쳐 지나가는 순간의
        # 잡음성 포획을 걸러낸다. HERD/CORNER를 벗어나 있는 동안(예: LOST로
        # 막 전이한 주기)은 타이머를 리셋해서, 끊겼다 이어진 체류 시간이
        # 합산되지 않도록 한다.
        if state in (FSMState.HERD, FSMState.CORNER):
            if inputs.distance_to_goal_m <= inputs.capture_radius_m:
                self._capture_hold_elapsed_sec += inputs.dt
                if self._capture_hold_elapsed_sec >= inputs.capture_hold_required_sec:
                    state = FSMState.CAPTURED
            else:
                self._capture_hold_elapsed_sec = 0.0
        elif state != FSMState.CAPTURED:
            self._capture_hold_elapsed_sec = 0.0

        self._state = state
        return state
