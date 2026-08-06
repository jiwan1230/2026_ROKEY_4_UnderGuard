# herding_controller/test/test_role_assigner.py
import numpy as np

from herding_controller.role_assigner import resolve_separation


def test_resolve_separation_pushes_blocker_away():
    driving_point = np.array([5.0, 5.0])
    blocking_point = np.array([5.1, 5.0])  # 0.1m밖에 떨어지지 않아 min_separation_m 위반
    adjusted = resolve_separation(driving_point, blocking_point, min_separation_m=0.6)
    assert np.linalg.norm(adjusted - driving_point) >= 0.6


def test_resolve_separation_handles_coincident_points():
    # driving_point == blocking_point (거리 0): NaN/쓰레기 값을 만들어서는 안 되며,
    # 최소 분리 요구사항도 여전히 만족해야 한다.
    point = np.array([5.0, 5.0])
    adjusted = resolve_separation(point, point.copy(), min_separation_m=0.6)
    assert not np.isnan(adjusted).any()
    assert np.linalg.norm(adjusted - point) >= 0.6


def test_resolve_separation_leaves_already_separated_points_untouched():
    driving_point = np.array([0.0, 0.0])
    blocking_point = np.array([5.0, 0.0])  # 이미 min_separation_m보다 훨씬 멀리 떨어져 있음
    adjusted = resolve_separation(driving_point, blocking_point, min_separation_m=0.6)
    np.testing.assert_array_equal(adjusted, blocking_point)
