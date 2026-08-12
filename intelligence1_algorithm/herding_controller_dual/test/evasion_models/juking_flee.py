# herding_controller_dual/test/evasion_models/juking_flee.py
"""ReactiveFlee(기존 검증 모델)를 그대로 쓰되, 도망치는 도중 가끔 무작위로
옆으로 새는 구간을 섞는다 -- DecisiveFlee(전면 재설계)가 캡처 메커니즘
자체를 무력화시킨 것과 달리, "정직한 도주"라는 기본 골격은 유지한 채
예측 가능성만 살짝 깨서 로봇 B의 블로킹이 의미를 가지는지 보기 위한
최소 변경 모델이다.
"""
import numpy as np

from test.evasion_models.base import EvasionModel
from test.evasion_models.reactive_flee import ReactiveFlee


class JukingFlee(EvasionModel):
    """ReactiveFlee 명령을 따르다가, 확률적으로 잠깐 옆으로 새는 구간을 낀다."""

    def __init__(
        self,
        max_speed_mps: float,
        flee_reaction_distance_m: float,
        juke_probability_per_sec: float = 0.5,
        juke_duration_sec: float = 0.4,
        juke_angle_range: tuple = (np.pi / 4, np.pi / 2),
        rng: np.random.Generator | None = None,
    ) -> None:
        """기본 ReactiveFlee를 감싸고, 새는(juke) 빈도/길이/각도 범위를 저장한다.

        juke_probability_per_sec: 로봇으로부터 실제로 도망치는 중일 때, 초당
            이 확률로 새기 시작한다(로봇이 안 보이면 애초에 도망칠 게 없으므로
            새지 않는다 -- ALGO-008 우연 포획률 대조군인 RandomWalk와는 다른
            시나리오).
        juke_duration_sec: 한 번 새면 이 시간 동안 새 방향을 유지한다.
        juke_angle_range: 원래 도주 방향 기준 좌우로 새는 각도(라디안) 범위.
        """
        self._reactive = ReactiveFlee(max_speed_mps, flee_reaction_distance_m)
        self.max_speed_mps = max_speed_mps
        self.juke_probability_per_sec = juke_probability_per_sec
        self.juke_duration_sec = juke_duration_sec
        self.juke_angle_range = juke_angle_range
        self.rng = rng or np.random.default_rng()
        self._juke_remaining_sec = 0.0
        self._juke_heading = 0.0

    def step(self, target_state: np.ndarray, robot_positions: list, obstacle_map, dt: float) -> np.ndarray:
        if self._juke_remaining_sec > 0.0:
            self._juke_remaining_sec -= dt
            return self._juke_velocity()

        base_command = self._reactive.step(target_state, robot_positions, obstacle_map, dt)
        fleeing = np.linalg.norm(base_command) > 1e-9
        if fleeing and self.rng.random() < self.juke_probability_per_sec * dt:
            base_heading = np.arctan2(base_command[1], base_command[0])
            sign = self.rng.choice([-1.0, 1.0])
            offset = self.rng.uniform(*self.juke_angle_range)
            self._juke_heading = base_heading + sign * offset
            self._juke_remaining_sec = self.juke_duration_sec
            return self._juke_velocity()
        return base_command

    def _juke_velocity(self) -> np.ndarray:
        return np.array([np.cos(self._juke_heading), np.sin(self._juke_heading)]) * self.max_speed_mps
