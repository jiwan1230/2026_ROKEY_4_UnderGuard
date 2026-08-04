# herding_controller/test/evasion_models/random_walk.py
"""Control-baseline model: ignores robots entirely, walks randomly."""
import numpy as np

from test.evasion_models.base import EvasionModel


class RandomWalk(EvasionModel):
    """Ignores robot positions; used to measure chance capture rate for ALGO-008."""

    def __init__(self, max_speed_mps: float, rng: np.random.Generator | None = None) -> None:
        """Store the constant speed and seed a random initial heading."""
        self.max_speed_mps = max_speed_mps
        self.rng = rng or np.random.default_rng()
        self._heading = self.rng.uniform(0, 2 * np.pi)

    def step(self, target_state: np.ndarray, robot_positions: list, obstacle_map, dt: float) -> np.ndarray:
        """Drift the heading randomly and return a constant-speed velocity in that direction."""
        self._heading += self.rng.normal(scale=0.5) * dt
        return np.array([np.cos(self._heading), np.sin(self._heading)]) * self.max_speed_mps
