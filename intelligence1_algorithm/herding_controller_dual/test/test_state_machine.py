# herding_controller_dual/test/test_state_machine.py
from herding_controller_dual.state_machine import FSMInputs, FSMState, HerdingStateMachine


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
    # capture_hold_required_sec 동안 capture 반경 안에서 유지 (dt=0.2, 3.0초를 위해 >=15 스텝 필요)
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
    fsm.step(base_inputs(target_observed=True, kf_converged=True))  # 이제 HERD 상태
    lost_state = fsm.step(base_inputs(
        target_observed=False, kf_converged=True, occlusion_elapsed_sec=3.5
    ))
    assert lost_state == FSMState.LOST
    recovered = fsm.step(base_inputs(target_observed=True, kf_converged=True, occlusion_elapsed_sec=0.0))
    assert recovered == FSMState.TRACK


def test_corner_falls_back_to_herd_when_entry_invariant_breaks():
    # 회귀 테스트: CORNER에 진입하려면 (capture zone 안에 있음 AND escape_prob_concentrated)
    # 조건이 필요하다. 이 불변식이 나중에 깨지면(예: 타겟이 capture zone을 벗어남),
    # FSM은 CORNER에 영원히 고착되어 있지 않고 HERD로 되돌아가야 한다 -- 그렇지 않으면
    # `.state`를 기준으로 동작하는 하위 제어 코드가 더 이상 goal 근처에 있지 않은
    # 타겟에 대해 계속 코너링 기동을 실행하게 된다.
    fsm = HerdingStateMachine()
    fsm.step(base_inputs())
    fsm.step(base_inputs(target_observed=True))
    fsm.step(base_inputs(target_observed=True, kf_converged=True))  # HERD
    entered = fsm.step(base_inputs(
        target_observed=True, kf_converged=True, distance_to_goal_m=0.3, escape_prob_concentrated=True
    ))
    assert entered == FSMState.CORNER

    # 타겟이 capture zone을 벗어나고 escape 확률이 흩어진다.
    fell_back = fsm.step(base_inputs(
        target_observed=True, kf_converged=True, distance_to_goal_m=10.0, escape_prob_concentrated=False
    ))
    assert fell_back == FSMState.HERD

    # 이후 스텝에서도 계속 HERD를 유지한다(원인 없이 진동하거나 재진입하지 않음).
    still_herd = fsm.step(base_inputs(
        target_observed=True, kf_converged=True, distance_to_goal_m=10.0, escape_prob_concentrated=False
    ))
    assert still_herd == FSMState.HERD

    # 그리고 불변식이 다시 성립되면 CORNER에 재진입할 수 있다.
    re_entered = fsm.step(base_inputs(
        target_observed=True, kf_converged=True, distance_to_goal_m=0.3, escape_prob_concentrated=True
    ))
    assert re_entered == FSMState.CORNER
