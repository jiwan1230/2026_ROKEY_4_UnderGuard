# herding_controller/test/test_state_machine.py
from herding_controller.state_machine import FSMInputs, FSMState, HerdingStateMachine


def base_inputs(**overrides):
    defaults = dict(
        target_observed=False, kf_converged=False, distance_to_goal_m=10.0, capture_radius_m=0.5,
        escape_prob_concentrated=False, occlusion_elapsed_sec=0.0, occlusion_timeout_sec=3.0,
        capture_hold_required_sec=3.0, dt=0.2,
    )
    defaults.update(overrides)
    return FSMInputs(**defaults)


def test_full_forward_path_idle_to_captured():
    fsm = HerdingStateMachine()
    assert fsm.step(base_inputs()) == FSMState.SEARCH
    assert fsm.step(base_inputs(target_observed=True)) == FSMState.TRACK
    assert fsm.step(base_inputs(target_observed=True, kf_converged=True)) == FSMState.HERD
    assert fsm.step(base_inputs(
        target_observed=True, kf_converged=True, distance_to_goal_m=0.3, escape_prob_concentrated=True
    )) == FSMState.CORNER
    # hold inside capture radius for capture_hold_required_sec (dt=0.2, need >=15 steps for 3.0s)
    state = FSMState.CORNER
    for _ in range(16):
        state = fsm.step(base_inputs(
            target_observed=True, kf_converged=True, distance_to_goal_m=0.3, escape_prob_concentrated=True
        ))
    assert state == FSMState.CAPTURED


def test_occlusion_timeout_from_herd_transitions_to_lost_then_back_to_track():
    fsm = HerdingStateMachine()
    fsm.step(base_inputs())
    fsm.step(base_inputs(target_observed=True))
    fsm.step(base_inputs(target_observed=True, kf_converged=True))  # now in HERD
    lost_state = fsm.step(base_inputs(
        target_observed=False, kf_converged=True, occlusion_elapsed_sec=3.5
    ))
    assert lost_state == FSMState.LOST
    recovered = fsm.step(base_inputs(target_observed=True, kf_converged=True, occlusion_elapsed_sec=0.0))
    assert recovered == FSMState.TRACK
