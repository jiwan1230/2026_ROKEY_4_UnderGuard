import numpy as np

from herding_controller_dual.target_estimator import EstimatorConfig, TargetEstimator


def make_estimator():
    return TargetEstimator(EstimatorConfig(process_noise=0.1, measurement_noise=0.05, occlusion_timeout_sec=1.0))


def test_converges_to_constant_velocity_track():
    est = make_estimator()
    true_vel = np.array([0.2, 0.0])
    pos = np.array([0.0, 0.0])
    dt = 0.1
    for _ in range(50):
        pos = pos + true_vel * dt
        est.predict(dt)
        est.update(pos)
    state = est.get_state()
    assert np.allclose(state.position, pos, atol=0.05)
    assert np.allclose(state.velocity, true_vel, atol=0.1)
    assert state.is_lost is False


def test_occlusion_triggers_lost_after_timeout():
    est = make_estimator()
    est.update(np.array([1.0, 1.0]))
    est.predict(0.5)
    assert est.get_state().is_lost is False
    est.predict(0.6)
    assert est.get_state().is_lost is True
