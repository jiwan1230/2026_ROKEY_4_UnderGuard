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
        self._x = np.zeros(4)          # 상태벡터 [px, py, vx, vy], 첫 관측 전엔 원점+정지로 가정
        self._P = np.eye(4) * 1e3      # 공분산: 초깃값은 "전혀 모른다"는 뜻으로 크게 잡음
        self._initialized = False      # 첫 update() 호출 여부 (원점 초깃값을 실제 위치로 덮어쓸지 판단)
        self._time_since_obs = 0.0     # 마지막 관측 이후 경과 시간(occlusion 판정용)

    def predict(self, dt: float) -> None:
        """새로운 측정값 없이 필터 상태를 dt초만큼 전진시킨다.

        표준 칼만 필터의 예측(predict) 단계. 상태벡터는 x = [px, py, vx, vy]
        (위치 2 + 속도 2). 등속도(constant-velocity) 모델이므로 상태 전이
        행렬 F는 "위치 += 속도 * dt, 속도는 그대로 유지"를 인코딩한다:

            [px]   [1 0 dt 0] [px]
            [py] = [0 1 0 dt] [py]
            [vx]   [0 0 1  0] [vx]
            [vy]   [0 0 0  1] [vy]

        Q(process noise)는 "등속도 가정이 완벽하지 않다"는 불확실성을
        공분산 P에 매 스텝 더해 넣는 항이다 — 타겟이 실제로는 가속/방향전환을
        하므로, Q가 없으면 필터가 과신(overconfident)해서 급격한 도주
        방향전환에 둔감하게 반응한다. dt에 비례시키는 이유는 예측 구간이
        길수록(관측이 뜸할수록) 그만큼 불확실성이 더 쌓여야 하기 때문이다.
        LOST 상태에서도 이 함수는 계속 호출된다(관측 없이 dt만 흘려보냄) —
        그래야 재관측 시 위치가 아니라 "얼마나 오래 못 봤는지"만으로
        occlusion_timeout_sec 판정이 가능해진다.
        """
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])  # 상태전이행렬 (위 docstring 수식)
        Q = np.eye(4) * self.config.process_noise * dt  # 과정 잡음 공분산 — dt에 비례해 누적
        self._x = F @ self._x                # 상태 예측: x_pred = F @ x
        self._P = F @ self._P @ F.T + Q       # 공분산 예측: P_pred = F P F^T + Q
        self._time_since_obs += dt            # 못 본 시간 누적

    def update(self, measurement: np.ndarray) -> None:
        """새로운 (x, y) 위치 관측값을 필터에 융합한다.

        표준 칼만 필터의 갱신(update) 단계. 관측 모델 H = [[1,0,0,0],[0,1,0,0]]는
        "센서는 위치(px, py)만 직접 관측하고 속도는 관측하지 못한다"는 것을
        인코딩한다 — 속도는 오직 위치가 시간에 따라 어떻게 변하는지로부터
        간접적으로(P의 위치-속도 교차 공분산을 통해) 추정된다.

        innovation(= measurement - H@x)은 "예측이 틀린 정도"이고, 칼만 이득
        K는 그 오차를 상태에 얼마나 반영할지를 결정한다: P(현재 예측이
        얼마나 불확실한가)와 R(measurement_noise, 센서가 얼마나 믿을만한가)의
        상대적 크기에 따라 자동으로 정해진다 — P가 크면(불확실하면) K가
        커져 관측을 더 신뢰하고, R이 크면(센서가 노이즈가 많으면) K가
        작아져 예측을 더 신뢰한다. 첫 번째 update() 호출 시에는 위치를
        측정값으로 직접 초기화한다 — 초기 상태가 원점(0,0)에 임의로 고정돼
        있어 첫 innovation이 비현실적으로 크게 튀는 것을 방지한다.
        """
        if not self._initialized:
            self._x[:2] = measurement       # 원점(0,0) 초깃값 대신 첫 실측 위치로 바로 시작
            self._initialized = True
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])   # 관측행렬 — 위치(px,py)만 관측, 속도는 못 봄
        R = np.eye(2) * self.config.measurement_noise  # 관측 잡음 공분산 (센서가 얼마나 못 믿을만한가)
        innovation = measurement - H @ self._x    # 잔차 = 실제 관측 - 예측했던 관측(H@x)
        S = H @ self._P @ H.T + R                 # 잔차 공분산 (예측 불확실성 + 관측 불확실성)
        K = self._P @ H.T @ np.linalg.inv(S)      # 칼만 이득 — 잔차를 상태에 얼마나 반영할지
        self._x = self._x + K @ innovation        # 상태 갱신: 예측 + 이득*잔차
        self._P = (np.eye(4) - K @ H) @ self._P   # 공분산 갱신 — 관측을 반영해 불확실성 감소
        self._time_since_obs = 0.0                # 방금 관측했으므로 경과시간 리셋

    def get_state(self) -> TargetState:
        """현재 위치/속도 추정치와 LOST 상태를 반환한다."""
        is_lost = self._time_since_obs > self.config.occlusion_timeout_sec  # timeout 넘게 못 봤으면 LOST
        return TargetState(
            position=self._x[:2].copy(),   # 상태벡터 앞 2개 = 위치. .copy()로 호출부가 내부 상태를 못 건드리게
            velocity=self._x[2:].copy(),   # 뒤 2개 = 속도
            covariance=self._P.copy(),
            is_lost=is_lost,
            time_since_observation=self._time_since_obs,
        )
