"""쥐(표적)가 지금 어디 있는지 추측하는 칼만 필터.

카메라로 매번 정확히 보이는 게 아니니까, "아까 이 속도로 가고 있었으니
지금쯤 여기 있겠다"는 예측과 "방금 실제로 여기서 봤다"는 관측을 계속
섞어서 제일 그럴듯한 위치를 만든다. 이렇게 두 추측을 섞는 방법이 칼만
필터(Kalman filter)다.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class EstimatorConfig:
    """칼만 필터를 얼마나 조심스럽게 믿을지 정하는 값들 + 놓쳤을 때 처리 시간."""
    process_noise: float
    measurement_noise: float
    occlusion_timeout_sec: float


@dataclass
class TargetState:
    """지금 이 순간 "쥐가 여기 있을 것이다"라는 최선의 추측."""
    position: np.ndarray
    velocity: np.ndarray
    covariance: np.ndarray
    is_lost: bool
    time_since_observation: float


class TargetEstimator:
    """쥐의 위치(x, y)와 속도(vx, vy)를 계속 추적하는 칼만 필터."""

    def __init__(self, config: EstimatorConfig) -> None:
        self.config = config
        self._x = np.zeros(4)          # [위치x, 위치y, 속도x, 속도y]. 아직 한 번도 못 봤으면 원점에서 정지로 가정
        self._P = np.eye(4) * 1e3      # "내 추측이 얼마나 못 미더운지" 점수. 처음엔 "전혀 모른다"이므로 크게 잡음
        self._initialized = False      # 실제로 한 번이라도 봤는지 (처음 보면 원점 대신 그 자리로 바로 이동시키려고)
        self._time_since_obs = 0.0     # 마지막으로 본 뒤 몇 초 지났는지 (오래 못 보면 LOST 판정에 씀)

    def predict(self, dt: float) -> None:
        """새로 본 게 없어도, "아까 가던 대로 dt초만큼 더 갔겠지"라고 위치를 밀어준다.

        속도는 그대로 두고 위치만 (속도 x 시간)만큼 옮기는 아주 단순한
        가정이다 (등속도 모델):

            새 위치x = 위치x + 속도x * dt
            새 위치y = 위치y + 속도y * dt
            속도는 그대로

        문제는 쥐가 실제로는 이 가정처럼 얌전하지 않다는 것 — 갑자기
        방향을 튼다. 그래서 예측할 때마다 "내 추측이 얼마나 못 미더운지"
        점수(P)를 조금씩 올려준다(Q를 더함). 안 그러면 필터가 "내 예측이
        무조건 맞다"고 과신해서, 쥐가 갑자기 방향을 틀어도 한참 뒤에야
        알아챈다. 오래 못 볼수록(dt가 쌓일수록) 그만큼 못 미더워져야
        하므로 dt에 비례해서 올린다.

        쥐를 놓친 동안(LOST 상태)에도 이 함수는 계속 불린다 — 위치를
        계속 갱신하려는 게 아니라, "못 본 지 몇 초나 됐는지"만 정확히
        세기 위해서다.
        """
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])  # 위 설명의 "속도만큼 밀기"를 행렬로 쓴 것
        Q = np.eye(4) * self.config.process_noise * dt  # 예측이 못 미더워지는 정도 (시간이 지날수록 커짐)
        self._x = F @ self._x                # 위치/속도 추측을 한 칸 전진
        self._P = F @ self._P @ F.T + Q       # "못 미더운 정도"도 같이 갱신
        self._time_since_obs += dt            # 못 본 시간 누적

    def update(self, measurement: np.ndarray) -> None:
        """실제로 관측된 (x, y) 위치가 들어오면, 그 정보로 추측을 고친다.

        카메라는 위치만 보여주고 속도는 안 보여준다 — 속도는 "아까는
        여기, 방금은 저기"라는 위치 변화로부터 필터가 스스로 계산해낸다.

        예측했던 위치와 실제로 관측된 위치의 차이(오차)를 얼마나 반영할지
        정하는 값이 K(칼만 이득)다. 쉽게 말해 **저울**이다:
          - 내 예측이 못 미더우면(P가 크면) → 방금 본 값을 더 믿는다 (K가 커짐)
          - 카메라가 원래 좀 부정확하면(R이 크면) → 내 예측을 더 믿는다 (K가 작아짐)
        어느 쪽을 더 믿을지 사람이 정하는 게 아니라 이 저울이 매번 자동으로 정한다.

        맨 처음 한 번은 좀 특별하게 처리한다: 필터가 시작할 때 위치를
        원점(0,0)으로 임의로 잡아뒀으므로, 첫 관측이 들어오면 "고쳐나가는"
        대신 아예 그 자리로 텔레포트시킨다. 안 그러면 첫 오차가 너무 커서
        이상하게 튄다.
        """
        if not self._initialized:
            self._x[:2] = measurement       # 원점(0,0)이라는 엉터리 초깃값 대신, 첫 실측 위치로 바로 시작
            self._initialized = True
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])   # "카메라는 위치만 본다"는 규칙
        R = np.eye(2) * self.config.measurement_noise  # 카메라가 얼마나 부정확한지
        innovation = measurement - H @ self._x    # 오차 = 실제로 본 위치 - 예측했던 위치
        S = H @ self._P @ H.T + R                 # 이 오차 자체가 얼마나 못 미더운지 (예측 오차 + 카메라 오차)
        K = self._P @ H.T @ np.linalg.inv(S)      # 저울: 이 오차를 얼마나 반영할지
        self._x = self._x + K @ innovation        # 최종 추측 = 예측 + 저울만큼 반영한 오차
        self._P = (np.eye(4) - K @ H) @ self._P   # 방금 실제로 봤으니 "못 미더운 정도"는 줄어든다
        self._time_since_obs = 0.0                # 방금 봤으니 못 본 시간 다시 0부터

    def get_state(self) -> TargetState:
        """지금 이 순간 최선의 위치/속도 추측과, 너무 오래 못 봤는지(LOST)를 알려준다."""
        is_lost = self._time_since_obs > self.config.occlusion_timeout_sec  # 정해둔 시간보다 오래 못 봤으면 LOST
        return TargetState(
            position=self._x[:2].copy(),   # 앞 두 칸 = 위치. .copy()로 원본이 바깥에서 함부로 안 바뀌게
            velocity=self._x[2:].copy(),   # 뒤 두 칸 = 속도
            covariance=self._P.copy(),
            is_lost=is_lost,
            time_since_observation=self._time_since_obs,
        )
