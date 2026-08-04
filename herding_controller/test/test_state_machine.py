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


def test_corner_falls_back_to_herd_when_entry_invariant_breaks():
    # Regression test: CORNER requires (in capture zone AND escape_prob_concentrated) to
    # enter. If that invariant later breaks (e.g. the target flees the capture zone), the
    # FSM must fall back to HERD rather than staying latched in CORNER forever -- otherwise
    # downstream control code keyed on `.state` would keep running cornering maneuvers on a
    # target that is no longer near the goal.
    fsm = HerdingStateMachine()
    fsm.step(base_inputs())
    fsm.step(base_inputs(target_observed=True))
    fsm.step(base_inputs(target_observed=True, kf_converged=True))  # HERD
    entered = fsm.step(base_inputs(
        target_observed=True, kf_converged=True, distance_to_goal_m=0.3, escape_prob_concentrated=True
    ))
    assert entered == FSMState.CORNER

    # Target flees the capture zone and escape probability disperses.
    fell_back = fsm.step(base_inputs(
        target_observed=True, kf_converged=True, distance_to_goal_m=10.0, escape_prob_concentrated=False
    ))
    assert fell_back == FSMState.HERD

    # It stays HERD on subsequent steps too (doesn't oscillate/re-enter without cause).
    still_herd = fsm.step(base_inputs(
        target_observed=True, kf_converged=True, distance_to_goal_m=10.0, escape_prob_concentrated=False
    ))
    assert still_herd == FSMState.HERD

    # And can re-enter CORNER again once the invariant is re-established.
    re_entered = fsm.step(base_inputs(
        target_observed=True, kf_converged=True, distance_to_goal_m=0.3, escape_prob_concentrated=True
    ))
    assert re_entered == FSMState.CORNER
