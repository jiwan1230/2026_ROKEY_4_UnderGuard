"""Constant-velocity Kalman filter for target position/velocity estimation."""
from dataclasses import dataclass

import numpy as np


@dataclass
class EstimatorConfig:
    """Kalman filter tuning and occlusion handling."""
    process_noise: float
    measurement_noise: float
    occlusion_timeout_sec: float


@dataclass
class TargetState:
    """Current best estimate of the target's map-frame state."""
    position: np.ndarray
    velocity: np.ndarray
    covariance: np.ndarray
    is_lost: bool
    time_since_observation: float


class TargetEstimator:
    """Tracks a target's [x, y, vx, vy] state with a constant-velocity KF."""

    def __init__(self, config: EstimatorConfig) -> None:
        self.config = config
        self._x = np.zeros(4)
        self._P = np.eye(4) * 1e3
        self._initialized = False
        self._time_since_obs = 0.0

    def predict(self, dt: float) -> None:
        """Advance the filter state by dt seconds with no new measurement."""
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
        Q = np.eye(4) * self.config.process_noise * dt
        self._x = F @ self._x
        self._P = F @ self._P @ F.T + Q
        self._time_since_obs += dt

    def update(self, measurement: np.ndarray) -> None:
        """Fuse a new (x, y) position observation into the filter."""
        if not self._initialized:
            self._x[:2] = measurement
            self._initialized = True
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        R = np.eye(2) * self.config.measurement_noise
        innovation = measurement - H @ self._x
        S = H @ self._P @ H.T + R
        K = self._P @ H.T @ np.linalg.inv(S)
        self._x = self._x + K @ innovation
        self._P = (np.eye(4) - K @ H) @ self._P
        self._time_since_obs = 0.0

    def get_state(self) -> TargetState:
        """Return the current position/velocity estimate and LOST status."""
        is_lost = self._time_since_obs > self.config.occlusion_timeout_sec
        return TargetState(
            position=self._x[:2].copy(),
            velocity=self._x[2:].copy(),
            covariance=self._P.copy(),
            is_lost=is_lost,
            time_since_observation=self._time_since_obs,
        )
