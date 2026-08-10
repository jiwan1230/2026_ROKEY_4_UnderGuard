# herding_controller_dual/herding_controller_dual/state_machine.py
""""지금 뭘 하고 있나"를 관리하는 상태표. IDLE -> SEARCH -> TRACK -> HERD -> CORNER -> CAPTURED 순서로 진행하고, 놓치면 LOST로 갔다가 다시 찾는다."""
from dataclasses import dataclass
from enum import Enum, auto


class FSMState(Enum):
    """로봇들이 지금 어떤 동작 모드인지."""
    IDLE = auto()      # 아직 시작 전
    SEARCH = auto()    # 쥐를 찾는 중
    TRACK = auto()     # 쥐를 봤는데 아직 속도를 잘 모름
    HERD = auto()      # 쥐를 덫 쪽으로 몰고 있음
    CORNER = auto()    # 거의 다 몰아서 구석에 몰림
    CAPTURED = auto()  # 잡았음
    LOST = auto()      # 놓쳐서 다시 찾는 중


@dataclass
class FSMInputs:
    """이번 순간에 상태를 바꿀지 말지 결정하는 데 필요한 정보들."""
    target_observed: bool          # 이번에 쥐가 실제로 보였는가
    kf_converged: bool             # 칼만 필터가 속도까지 믿을 만하게 알아냈는가
    distance_to_goal_m: float      # 쥐 추정 위치 <-> 덫까지 거리
    capture_radius_m: float        # 이 거리 안이면 "덫 근처"로 침
    escape_prob_concentrated: bool  # 도망갈 방향이 한쪽으로 쏠려 있는가 (escape_model.py)
    occlusion_elapsed_sec: float   # 마지막으로 본 뒤 몇 초 지났는가
    occlusion_timeout_sec: float   # 이 시간 넘게 못 보면 LOST로 감
    capture_hold_required_sec: float  # 덫 근처에 이만큼 계속 있어야 CAPTURED로 확정
    dt: float                      # 이번 한 번의 시간 간격(초)


class HerdingStateMachine:
    """매 순간 들어오는 정보를 보고 지금 상태를 다음 상태로 넘겨준다."""

    def __init__(self) -> None:
        self._state = FSMState.IDLE
        self._capture_hold_elapsed_sec = 0.0

    @property
    def state(self) -> FSMState:
        """지금 상태."""
        return self._state

    def step(self, inputs: FSMInputs) -> FSMState:
        """이번 순간 정보를 보고 다음 상태를 정해서 저장한다."""
        state = self._state

        # 1. 각 상태에서 "앞으로 진행"하는 조건들. 여기서는 뒤로 돌아가는
        # 경우(못 보면 LOST로, 다시 보이면 CAPTURED 취소 등)는 안 다룬다 —
        # 그건 아래 2번/3번에서 상태가 뭐든 상관없이 한 번에 처리한다.
        # 그래야 "못 보면 무조건 LOST로 간다"는 규칙을 상태마다 따로
        # 안 적어도 된다.
        if state == FSMState.IDLE:
            # IDLE은 그냥 막 켜졌을 때의 시작 상태일 뿐, 딱히 기다릴 이유가
            # 없다. 바로 다음 순간에 SEARCH로 넘어간다.
            state = FSMState.SEARCH
        elif state == FSMState.SEARCH:
            if inputs.target_observed:
                state = FSMState.TRACK
        elif state == FSMState.TRACK:
            # "쥐가 보였다"와 "칼만 필터가 속도까지 믿을 만하게 안다"는
            # 다른 얘기다: 방금 막 처음 봤을 땐 속도 추정이 아직
            # 못 미더우니까, HERD로 넘기기 전에 kf_converged까지
            # 같이 확인한다.
            if inputs.target_observed and inputs.kf_converged:
                state = FSMState.HERD
        elif state == FSMState.HERD:
            # CORNER로 가려면 "덫 가까이 왔다"는 것뿐 아니라 "도망갈
            # 방향이 한쪽으로 쏠려 있다"는 조건도 같이 필요하다. 위치만
            # 보고 CORNER라고 판단하면, 덫 근처에 왔어도 여전히 사방으로
            # 도망갈 수 있는 쥐를 성급하게 "구석에 몰렸다"고 착각하게 된다.
            in_capture_zone = inputs.distance_to_goal_m <= inputs.capture_radius_m  # 덫 근처에 왔는가
            if in_capture_zone and inputs.escape_prob_concentrated:
                state = FSMState.CORNER
        elif state == FSMState.CORNER:
            # 위 두 조건 중 하나라도 깨지면(덫에서 멀어지거나, 도망갈
            # 방향이 다시 여러 곳으로 퍼지면) 바로 HERD로 돌아간다 —
            # CORNER를 벗어나는 별도 조건이 따로 없는 이유는, "구석에
            # 몰림"이 정확히 "HERD 조건의 반대"로 정의돼 있기 때문이다.
            in_capture_zone = inputs.distance_to_goal_m <= inputs.capture_radius_m  # 덫 근처에 왔는가
            if not (in_capture_zone and inputs.escape_prob_concentrated):
                state = FSMState.HERD
        elif state == FSMState.LOST:
            # LOST에서는 다시 보이자마자 바로 HERD로 안 가고 TRACK부터
            # 다시 거친다: 한참 못 봤으니 칼만 필터가 다시 믿을 만해질
            # 시간이 필요하고, 그건 TRACK의 kf_converged 조건이 맡는다.
            if inputs.target_observed:
                state = FSMState.TRACK

        # 2. 놓쳤는지 감시 (어느 상태든 상관없이 확인). TRACK/HERD/CORNER
        # 중이었다면, 방금 위에서 다른 상태로 넘어갔더라도 너무 오래
        # 못 봤으면 LOST로 덮어쓴다 — "쥐를 놓쳤다"는 건 지금까지
        # 어디까지 진행했었는지와 상관없이 제일 먼저 처리해야 하는
        # 안전장치이기 때문이다.
        if state in (FSMState.TRACK, FSMState.HERD, FSMState.CORNER):
            if inputs.occlusion_elapsed_sec > inputs.occlusion_timeout_sec:
                state = FSMState.LOST

        # 3. 잡았는지 확인하는 타이머 (어느 상태든 상관없이 확인). "덫
        # 근처에 몇 초 동안 계속 있었는가"로 판단해야, 쥐가 덫 경계를
        # 살짝 스쳐 지나가는 것까지 "잡았다"고 착각하지 않는다.
        # HERD/CORNER를 벗어나 있는 동안(예: 막 LOST로 넘어간 순간)은
        # 타이머를 리셋해서, 끊겼다 이어진 시간이 합쳐지지 않게 한다.
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
