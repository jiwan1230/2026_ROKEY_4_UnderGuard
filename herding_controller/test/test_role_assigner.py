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


def test_role_does_not_swap_within_cooldown():
    assigner = RoleAssigner(make_config(margin=0.01, cooldown=2.0))
    driving_point = np.array([5.0, 5.0])
    heading = np.array([1.0, 0.0])
    # robot1 starts closer -> driver=1 at t=0
    assigner.assign(np.array([4.9, 5.0]), np.array([0.0, 0.0]), heading, heading, driving_point, 0.0)
    # now robot2 becomes closer, well past the margin, but only 0.5s later (< cooldown)
    driver, _ = assigner.assign(
        np.array([10.0, 10.0]), np.array([4.9, 5.0]), heading, heading, driving_point, 0.5
    )
    assert driver == 1  # must not have swapped yet


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
    blocking_point = np.array([5.1, 5.0])  # only 0.1m away, violates min_robot_separation_m
    adjusted = resolve_separation(driving_point, blocking_point, config)
    assert np.linalg.norm(adjusted - driving_point) >= config.min_robot_separation_m


def test_resolve_separation_handles_coincident_points():
    # driving_point == blocking_point (zero distance): must not produce NaN/garbage,
    # and must still satisfy the minimum separation requirement.
    config = make_config(separation=0.6)
    point = np.array([5.0, 5.0])
    adjusted = resolve_separation(point, point.copy(), config)
    assert not np.isnan(adjusted).any()
    assert np.linalg.norm(adjusted - point) >= config.min_robot_separation_m


def test_role_cost_turn_weight_breaks_ties_between_equidistant_robots():
    # Both robots are exactly equidistant from the driving point, so distance
    # alone is a tie. If role_cost_turn_weight were ignored or mis-wired, the
    # tie-break would stay a tie (driver defaults to 1) regardless of heading.
    # With turn weight applied, robot2 (already facing the driving point)
    # should have lower cost than robot1 (facing directly away from it) and
    # be preferred as Driver.
    config = make_config(margin=0.0001, cooldown=0.0)
    assigner = RoleAssigner(config)
    driving_point = np.array([5.0, 0.0])
    robot1_pos = np.array([0.0, 0.0])
    robot2_pos = np.array([0.0, 0.0])
    robot1_heading = np.array([-1.0, 0.0])  # facing away from driving_point
    robot2_heading = np.array([1.0, 0.0])   # facing toward driving_point
    driver, blocker = assigner.assign(
        robot1_pos, robot2_pos, robot1_heading, robot2_heading, driving_point, 0.0
    )
    assert driver == 2
    assert blocker == 1

    # Sanity check: with turn weight zeroed out, the same geometry is a pure
    # distance tie and the tie-break falls back to the default driver (1).
    zero_turn_config = make_config(margin=0.0001, cooldown=0.0)
    zero_turn_config.role_cost_turn_weight = 0.0
    tie_assigner = RoleAssigner(zero_turn_config)
    driver_tie, _ = tie_assigner.assign(
        robot1_pos, robot2_pos, robot1_heading, robot2_heading, driving_point, 0.0
    )
    assert driver_tie == 1
