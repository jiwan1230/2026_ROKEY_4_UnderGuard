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

        if state == FSMState.IDLE:
            state = FSMState.SEARCH
        elif state == FSMState.SEARCH:
            if inputs.target_observed:
                state = FSMState.TRACK
        elif state == FSMState.TRACK:
            if inputs.target_observed and inputs.kf_converged:
                state = FSMState.HERD
        elif state == FSMState.HERD:
            in_capture_zone = inputs.distance_to_goal_m <= inputs.capture_radius_m
            if in_capture_zone and inputs.escape_prob_concentrated:
                state = FSMState.CORNER
        elif state == FSMState.CORNER:
            in_capture_zone = inputs.distance_to_goal_m <= inputs.capture_radius_m
            if not (in_capture_zone and inputs.escape_prob_concentrated):
                state = FSMState.HERD
        elif state == FSMState.LOST:
            if inputs.target_observed:
                state = FSMState.TRACK

        if state in (FSMState.TRACK, FSMState.HERD, FSMState.CORNER):
            if inputs.occlusion_elapsed_sec > inputs.occlusion_timeout_sec:
                state = FSMState.LOST

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
