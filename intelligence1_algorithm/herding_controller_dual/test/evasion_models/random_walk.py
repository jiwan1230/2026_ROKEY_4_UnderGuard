# herding_controller_dual/test/evasion_models/random_walk.py
"""대조군 기준 모델: 로봇을 완전히 무시하고 무작위로 이동한다."""
import numpy as np

from test.evasion_models.base import EvasionModel


class RandomWalk(EvasionModel):
    """로봇 위치를 무시한다. ALGO-008의 우연 포획률을 측정하는 데 사용된다."""

    def __init__(self, max_speed_mps: float, rng: np.random.Generator | None = None) -> None:
        """일정한 속도를 저장하고 무작위 초기 방향(heading)을 설정한다."""
        self.max_speed_mps = max_speed_mps
        self.rng = rng or np.random.default_rng()
        self._heading = self.rng.uniform(0, 2 * np.pi)

    def step(self, target_state: np.ndarray, robot_positions: list, obstacle_map, dt: float) -> np.ndarray:
        """방향(heading)을 무작위로 흔들며 그 방향으로 일정한 속도의 벡터를 반환한다."""
        self._heading += self.rng.normal(scale=0.5) * dt
        return np.array([np.cos(self._heading), np.sin(self._heading)]) * self.max_speed_mps
