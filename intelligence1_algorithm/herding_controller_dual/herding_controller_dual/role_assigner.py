# herding_controller_dual/herding_controller_dual/role_assigner.py
"""스왑 이력현상(hysteresis)을 적용한 동적 Driver/Blocker 역할 배정 + 최소 이격 거리 유지.

플랜 A(herding_controller)와의 차이점: 플랜 A는 "Driver는 상위 시스템이 이미
배정한 값을 고정으로 받아들인다"는 전제였지만, 여기(플랜 B)는 두 로봇이 필드에
나와 몰이 중인 동안에는 상황(거리/방향)에 따라 어느 로봇이 미는 역할(Driver)을
할지 실시간으로 다시 판단한다 -- git 히스토리 `b35eb68`에 있던 원래 구현을
복원한 것이다(당시 6c04c5b 커밋에서 "로봇 A는 조종 안 함" 정정과 함께 같이
제거됐었다).
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class RoleAssignerConfig:
    """역할 교체를 위한 이력현상 임계값. 최소 이격 거리는 HerdingConfig 쪽에 따로 있다."""
    role_swap_margin: float
    role_swap_cooldown_sec: float
    role_cost_turn_weight: float


class RoleAssigner:
    """어느 로봇(1 또는 2)이 Driver인지 배정하며, 진동을 막기 위해 이력현상을 적용한다."""

    def __init__(self, config: RoleAssignerConfig) -> None:
        self.config = config
        self._driver_id = 1
        self._last_swap_time = None

    def assign(
        self,
        robot1_pos: np.ndarray,
        robot2_pos: np.ndarray,
        robot1_heading: np.ndarray,
        robot2_heading: np.ndarray,
        driving_point_candidate: np.ndarray,
        current_time_sec: float,
    ) -> tuple[int, int]:
        """(driver_id, blocker_id)를 반환하며, margin+cooldown 임계값을 넘을 때만 교체한다."""
        cost1 = self._cost(robot1_pos, robot1_heading, driving_point_candidate)
        cost2 = self._cost(robot2_pos, robot2_heading, driving_point_candidate)
        candidate_driver = 1 if cost1 <= cost2 else 2

        if self._last_swap_time is None:
            # 최초 호출: margin/cooldown 게이팅으로 보호해야 할 이전 "swap"이
            # 없고, 이력현상을 적용할 실제 이전 배정도 없다 -- 지금까지의
            # self._driver_id == 1은 실제로 계산된 결정이 아니라 그저 임의의
            # 부트스트랩 기본값이다. 비용이 최적인 후보를 그대로 채택한다.
            self._driver_id = candidate_driver
            self._last_swap_time = current_time_sec
        elif candidate_driver != self._driver_id:
            cost_diff = abs(cost1 - cost2)
            time_since_swap = current_time_sec - self._last_swap_time
            if cost_diff >= self.config.role_swap_margin and time_since_swap >= self.config.role_swap_cooldown_sec:
                self._driver_id = candidate_driver
                self._last_swap_time = current_time_sec

        blocker_id = 2 if self._driver_id == 1 else 1
        return self._driver_id, blocker_id

    def _cost(self, robot_pos: np.ndarray, robot_heading: np.ndarray, target_point: np.ndarray) -> float:
        """Driver 후보로서의 비용: 직선 거리 + (제자리 회전 각도 * 가중치).

        거리만 비교하면 이미 정확한 방향을 보고 있는 로봇보다, 더 가깝지만
        180도 돌아야 하는 로봇을 Driver로 뽑아버릴 수 있다. 회전 비용을
        더해 두 로봇의 "실제로 그 지점에 도달하는 데 걸리는 수고"를 더
        가깝게 근사한다.
        """
        distance = float(np.linalg.norm(target_point - robot_pos))
        desired = target_point - robot_pos
        norm = np.linalg.norm(desired)
        if norm < 1e-6:
            return distance
        desired = desired / norm
        cos_angle = float(np.clip(np.dot(desired, robot_heading), -1.0, 1.0))
        turn_cost = float(np.arccos(cos_angle))
        return distance + self.config.role_cost_turn_weight * turn_cost


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
