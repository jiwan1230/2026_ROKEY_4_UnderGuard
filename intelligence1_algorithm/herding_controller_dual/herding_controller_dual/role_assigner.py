# herding_controller_dual/herding_controller_dual/role_assigner.py
"""스왑 이력현상(hysteresis)을 적용한 동적 Driver/Blocker 역할 배정 + 최소 이격 거리 유지.

플랜 A(herding_controller)와의 차이점: 플랜 A는 "Driver는 상위 시스템이 이미
배정한 값을 고정으로 받아들인다"는 전제였지만, 여기(플랜 B)는 두 로봇이 필드에
나와 몰이 중인 동안에는 상황(거리/방향)에 따라 어느 로봇이 미는 역할(Driver)을
할지 실시간으로 다시 판단한다 -- git 히스토리 `b35eb68`에 있던 원래 구현을
복원한 것이다(당시 6c04c5b 커밋에서 "로봇 A는 조종 안 함" 정정과 함께 같이
제거됐었다). 원본은 `algorithm/dual-robot-herding` 브랜치의
`herding_controller_dual`에 있으며(2026-08-07 복원, 4abf472), 이번엔 그
당시 코드베이스(section 10-4까지)가 아니라 **최신 herding_controller**
(section 13까지의 모든 수정: 벽 반발 샘플링 반경, deadlock 감지·해제,
escape_model 반발항 게이팅 포함) 위에 다시 이식했다.
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
        self._driver_id = 1          # 임의의 부트스트랩 기본값 (첫 assign() 호출에서 실제로 결정됨)
        self._last_swap_time = None  # 마지막으로 역할이 바뀐 시각 (None = 아직 한 번도 배정 안 함)

    def assign(
        self,
        robot1_pos: np.ndarray,
        robot2_pos: np.ndarray,
        robot1_heading: np.ndarray,
        robot2_heading: np.ndarray,
        driving_point_candidate: np.ndarray,
        current_time_sec: float,
    ) -> tuple[int, int]:
        """두 로봇 중 누가 미는 역할(Driver)인지 정해서 (driver_id, blocker_id)로 돌려준다.

        너무 자주 바뀌면 로봇이 갈팡질팡하니까, "확실히 유리할 때"만
        바꾼다: 비용 차이가 margin보다 크게 나고 + 마지막으로 바꾼 지
        cooldown 시간이 지났을 때만 실제로 교체한다.
        """
        cost1 = self._cost(robot1_pos, robot1_heading, driving_point_candidate)  # 로봇1이 미는 역할을 맡으면 드는 수고
        cost2 = self._cost(robot2_pos, robot2_heading, driving_point_candidate)  # 로봇2가 미는 역할을 맡으면 드는 수고
        candidate_driver = 1 if cost1 <= cost2 else 2  # 지금 이 순간만 보면 수고가 더 적은 쪽

        if self._last_swap_time is None:
            # 맨 처음 부르는 경우: 지금까지 self._driver_id == 1은 실제로
            # 계산한 게 아니라 그냥 임시로 넣어둔 값이었다. 그러니 "확실히
            # 유리할 때만 바꾼다"는 규칙 없이, 지금 계산한 결과를 그냥
            # 바로 채택한다.
            self._driver_id = candidate_driver
            self._last_swap_time = current_time_sec
        elif candidate_driver != self._driver_id:  # 지금 맡고 있는 로봇과 다른 로봇이 더 유리해 보임 — 바꿀지 검토
            cost_diff = abs(cost1 - cost2)                              # 두 로봇 수고 차이 (차이가 작으면 굳이 안 바꿈)
            time_since_swap = current_time_sec - self._last_swap_time   # 마지막으로 바꾼 뒤 지난 시간
            if cost_diff >= self.config.role_swap_margin and time_since_swap >= self.config.role_swap_cooldown_sec:
                self._driver_id = candidate_driver   # 차이도 충분하고 시간도 지났음 — 이제 진짜로 바꾼다
                self._last_swap_time = current_time_sec

        blocker_id = 2 if self._driver_id == 1 else 1  # 막는 역할은 미는 역할이 아닌 나머지 로봇
        return self._driver_id, blocker_id

    def _cost(self, robot_pos: np.ndarray, robot_heading: np.ndarray, target_point: np.ndarray) -> float:
        """이 로봇이 미는 역할을 맡을 때 드는 수고 = 거리 + (돌아야 하는 각도 * 가중치).

        거리만 비교하면, 이미 목표 방향을 보고 있는 로봇보다 "더 가깝지만
        180도 뒤돌아야 하는" 로봇을 뽑아버릴 수 있다. 그래서 돌아야 하는
        각도도 같이 더해서, "실제로 그 자리까지 가는 데 드는 진짜 수고"에
        더 가깝게 맞춘다.
        """
        distance = float(np.linalg.norm(target_point - robot_pos))  # 목표점까지 직선 거리
        desired = target_point - robot_pos    # 로봇 → 목표점 방향
        norm = np.linalg.norm(desired)
        if norm < 1e-6:
            return distance  # 이미 목표점에 있음 — 방향이 없으니 회전 수고도 없음
        desired = desired / norm  # 방향만 남기고 길이는 1로
        cos_angle = float(np.clip(np.dot(desired, robot_heading), -1.0, 1.0))  # 지금 보는 방향과 목표 방향이 얼마나 비슷한지
        turn_cost = float(np.arccos(cos_angle))  # 그걸 "돌아야 하는 각도(라디안)"로 바꿈
        return distance + self.config.role_cost_turn_weight * turn_cost  # 거리 + 가중치를 곱한 회전 수고


def resolve_separation(driving_point: np.ndarray, blocking_point: np.ndarray,
                       min_separation_m: float) -> np.ndarray:
    """두 로봇이 너무 가까워지지 않도록, 막는 로봇의 목표점을 미는 로봇 쪽에서 밀어낸다."""
    delta = blocking_point - driving_point   # 미는 로봇 → 막는 로봇 방향
    dist = np.linalg.norm(delta)
    if dist >= min_separation_m:
        return blocking_point  # 이미 충분히 떨어져 있음 — 그대로 둔다
    direction = delta / dist if dist > 1e-6 else np.array([1.0, 0.0])  # 완전히 겹쳐 있으면(방향 없음) 그냥 오른쪽으로 밀어냄
    # 컴퓨터의 소수점 계산은 아주 살짝 오차가 생길 수 있어서, 밀어낸
    # 거리가 min_separation_m보다 미세하게 모자라는 걸 막으려고 아주
    # 조금 더 밀어낸다.
    push_distance = min_separation_m * (1.0 + 1e-9) + 1e-9
    return driving_point + direction * push_distance
