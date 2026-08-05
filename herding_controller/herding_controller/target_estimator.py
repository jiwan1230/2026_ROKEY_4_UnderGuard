"""타겟 위치/속도 추정을 위한 등속도(constant-velocity) 칼만 필터."""
from dataclasses import dataclass

import numpy as np


@dataclass
class EstimatorConfig:
    """칼만 필터 튜닝 및 occlusion 처리."""
    process_noise: float
    measurement_noise: float
    occlusion_timeout_sec: float


@dataclass
class TargetState:
    """타겟의 맵 프레임 상태에 대한 현재 최선의 추정치."""
    position: np.ndarray
    velocity: np.ndarray
    covariance: np.ndarray
    is_lost: bool
    time_since_observation: float


class TargetEstimator:
    """등속도 KF로 타겟의 [x, y, vx, vy] 상태를 추적한다."""

    def __init__(self, config: EstimatorConfig) -> None:
        self.config = config
        self._x = np.zeros(4)
        self._P = np.eye(4) * 1e3
        self._initialized = False
        self._time_since_obs = 0.0

    def predict(self, dt: float) -> None:
        """새로운 측정값 없이 필터 상태를 dt초만큼 전진시킨다."""
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
        Q = np.eye(4) * self.config.process_noise * dt
        self._x = F @ self._x
        self._P = F @ self._P @ F.T + Q
        self._time_since_obs += dt

    def update(self, measurement: np.ndarray) -> None:
        """새로운 (x, y) 위치 관측값을 필터에 융합한다."""
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
        """현재 위치/속도 추정치와 LOST 상태를 반환한다."""
        is_lost = self._time_since_obs > self.config.occlusion_timeout_sec
        return TargetState(
            position=self._x[:2].copy(),
            velocity=self._x[2:].copy(),
            covariance=self._P.copy(),
            is_lost=is_lost,
            time_since_observation=self._time_since_obs,
        )
