# herding_controller/test/test_role_assigner.py
import numpy as np

from herding_controller.role_assigner import RoleAssigner, RoleAssignerConfig, resolve_separation


def make_config(margin=0.5, cooldown=2.0, separation=0.6):
    return RoleAssignerConfig(
        role_swap_margin=margin, role_swap_cooldown_sec=cooldown,
        min_robot_separation_m=separation, role_cost_turn_weight=0.3,
    )


def test_closer_robot_is_assigned_driver_initially():
    assigner = RoleAssigner(make_config())
    driving_point = np.array([5.0, 5.0])
    driver, blocker = assigner.assign(
        robot1_pos=np.array([4.9, 5.0]), robot2_pos=np.array([0.0, 0.0]),
        robot1_heading=np.array([1.0, 0.0]), robot2_heading=np.array([1.0, 0.0]),
        driving_point_candidate=driving_point, current_time_sec=0.0,
    )
    assert driver == 1
    assert blocker == 2


def test_farther_robot1_is_not_bootstrapped_as_driver_on_first_call():
    # 회귀 테스트: 최초의 assign() 호출은 margin이나 cooldown 게이팅 없이
    # 비용이 최적인 후보로부터 driver를 바로 부트스트랩해야 한다 -- 아직
    # 보호해야 할 이전 "swap"이 존재하지 않기 때문이다. robot1이 기본값
    # self._driver_id로 하드코딩되어 있으므로, 이 시나리오(nonzero cooldown
    # 상태에서 최초 호출 시 robot2가 명백히 비용 최적인 경우)만이
    # "올바르게 계산됨"과 "우연히 하드코딩된 기본값과 일치함"을 구별할 수 있다.
    assigner = RoleAssigner(make_config(margin=0.5, cooldown=2.0))
    driving_point = np.array([5.0, 5.0])
    heading = np.array([1.0, 0.0])
    driver, blocker = assigner.assign(
        robot1_pos=np.array([100.0, 100.0]), robot2_pos=np.array([5.0, 5.0]),
        robot1_heading=heading, robot2_heading=heading,
        driving_point_candidate=driving_point, current_time_sec=0.0,
    )
    assert driver == 2  # t=0 부트스트랩 기본값에 고착되면 안 됨
    assert blocker == 1


def test_role_does_not_swap_within_cooldown():
    assigner = RoleAssigner(make_config(margin=0.01, cooldown=2.0))
    driving_point = np.array([5.0, 5.0])
    heading = np.array([1.0, 0.0])
    # robot1이 처음에 더 가까움 -> t=0에서 driver=1
    assigner.assign(np.array([4.9, 5.0]), np.array([0.0, 0.0]), heading, heading, driving_point, 0.0)
    # 이제 robot2가 더 가까워지고 margin을 크게 넘지만, 겨우 0.5초 후(< cooldown)
    driver, _ = assigner.assign(
        np.array([10.0, 10.0]), np.array([4.9, 5.0]), heading, heading, driving_point, 0.5
    )
    assert driver == 1  # 아직 swap되지 않았어야 함


def test_role_swaps_after_cooldown_elapses():
    assigner = RoleAssigner(make_config(margin=0.01, cooldown=2.0))
    driving_point = np.array([5.0, 5.0])
    heading = np.array([1.0, 0.0])
    assigner.assign(np.array([4.9, 5.0]), np.array([0.0, 0.0]), heading, heading, driving_point, 0.0)
    driver, _ = assigner.assign(
        np.array([10.0, 10.0]), np.array([4.9, 5.0]), heading, heading, driving_point, 2.5
    )
    assert driver == 2


def test_resolve_separation_pushes_blocker_away():
    config = make_config(separation=0.6)
    driving_point = np.array([5.0, 5.0])
    blocking_point = np.array([5.1, 5.0])  # 0.1m밖에 떨어지지 않아 min_robot_separation_m 위반
    adjusted = resolve_separation(driving_point, blocking_point, config)
    assert np.linalg.norm(adjusted - driving_point) >= config.min_robot_separation_m


def test_resolve_separation_handles_coincident_points():
    # driving_point == blocking_point (거리 0): NaN/쓰레기 값을 만들어서는 안 되며,
    # 최소 분리 요구사항도 여전히 만족해야 한다.
    config = make_config(separation=0.6)
    point = np.array([5.0, 5.0])
    adjusted = resolve_separation(point, point.copy(), config)
    assert not np.isnan(adjusted).any()
    assert np.linalg.norm(adjusted - point) >= config.min_robot_separation_m


def test_role_cost_turn_weight_breaks_ties_between_equidistant_robots():
    # 두 로봇 모두 driving point로부터 정확히 같은 거리에 있으므로, 거리만으로는
    # 동점이다. role_cost_turn_weight가 무시되거나 잘못 연결되었다면, heading과
    # 무관하게 동점 처리(driver는 기본값 1)가 될 것이다. turn weight가 적용되면,
    # (이미 driving point를 향하고 있는) robot2가 (그 반대를 향한) robot1보다
    # 비용이 낮아 Driver로 선호되어야 한다.
    config = make_config(margin=0.0001, cooldown=0.0)
    assigner = RoleAssigner(config)
    driving_point = np.array([5.0, 0.0])
    robot1_pos = np.array([0.0, 0.0])
    robot2_pos = np.array([0.0, 0.0])
    robot1_heading = np.array([-1.0, 0.0])  # driving_point 반대 방향을 향함
    robot2_heading = np.array([1.0, 0.0])   # driving_point 쪽을 향함
    driver, blocker = assigner.assign(
        robot1_pos, robot2_pos, robot1_heading, robot2_heading, driving_point, 0.0
    )
    assert driver == 2
    assert blocker == 1

    # 정합성 확인: turn weight를 0으로 만들면, 동일한 기하 구조는 순수한
    # 거리 동점이 되어 동점 처리가 기본 driver(1)로 되돌아간다.
    zero_turn_config = make_config(margin=0.0001, cooldown=0.0)
    zero_turn_config.role_cost_turn_weight = 0.0
    tie_assigner = RoleAssigner(zero_turn_config)
    driver_tie, _ = tie_assigner.assign(
        robot1_pos, robot2_pos, robot1_heading, robot2_heading, driving_point, 0.0
    )
    assert driver_tie == 1
