# herding_controller/herding_controller/role_assigner.py
"""Driver/Blocker 최소 이격 거리 유지 유틸리티.

Driver(로봇 A) 역할은 이 패키지가 계산해서 배정하지 않는다: 실제 운용에서는
두 로봇이 배터리 상태에 따라 번갈아 순찰/충전하고, 순찰 중 표적을 발견한
로봇이 그 순간 Driver로 배정되며 그 배정은 해당 포획 에피소드가 끝날 때까지
고정된다 (배정 자체는 이 패키지 밖의 상위 시스템이 담당). 이 패키지는 항상
"로봇 1 = 이미 주어진 Driver, 로봇 2 = 우리가 목표점을 계산해서 알려주는
Blocker"로 취급하며, 비용을 비교해서 매 주기 역할을 재배정하는 로직은 두지
않는다 -- 그런 재배정은 애초에 이 패키지의 책임이 아니다.
"""
import numpy as np


def resolve_separation(driving_point: np.ndarray, blocking_point: np.ndarray,
                       min_separation_m: float) -> np.ndarray:
    """최소 이격 거리를 유지하기 위해 Blocker의 목표점을 Driver의 (실제/참고) 위치로부터 밀어낸다."""
    delta = blocking_point - driving_point
    dist = np.linalg.norm(delta)
    if dist >= min_separation_m:
        return blocking_point
    direction = delta / dist if dist > 1e-6 else np.array([1.0, 0.0])
    # 덧셈/뺄셈 왕복 과정의 부동소수점 반올림으로 인해 결과가
    # min_separation_m보다 미세하게 부족해지지 않도록, 임계값을 살짝
    # 넘겨서 밀어낸다 (float64에서 norm(a+b)는 정확히 ||b||가 아니다).
    push_distance = min_separation_m * (1.0 + 1e-9) + 1e-9
    return driving_point + direction * push_distance
