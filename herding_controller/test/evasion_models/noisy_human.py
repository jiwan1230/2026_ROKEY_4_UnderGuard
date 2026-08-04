# herding_controller/test/evasion_models/noisy_human.py
"""Approximates a human RC operator: WallHugger behavior with reaction delay and noise."""
import numpy as np

from test.evasion_models.base import EvasionModel
from test.evasion_models.wall_hugger import WallHugger


class NoisyHuman(EvasionModel):
    """The real-world-success predictor: delayed, noisy WallHugger commands."""

    def __init__(
        self,
        max_speed_mps: float,
        flee_reaction_distance_m: float,
        grid_map,
        reaction_delay_range: tuple = (0.3, 0.8),
        noise_std: float = 0.1,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.max_speed_mps = max_speed_mps
        self._wall_hugger = WallHugger(max_speed_mps, flee_reaction_distance_m, grid_map)
        self.reaction_delay_range = reaction_delay_range
        self.noise_std = noise_std
        self.rng = rng or np.random.default_rng()
        self._pending_delay_sec = self.rng.uniform(*reaction_delay_range)
        self._elapsed_since_command_sec = 0.0
        self._held_command = np.zeros(2)

    def step(self, target_state: np.ndarray, robot_positions: list, obstacle_map, dt: float) -> np.ndarray:
        self._elapsed_since_command_sec += dt
        if self._elapsed_since_command_sec >= self._pending_delay_sec:
            base_command = self._wall_hugger.step(target_state, robot_positions, obstacle_map, dt)
            noise = self.rng.normal(scale=self.noise_std, size=2)
            noisy_command = base_command + noise
            # Adding noise can push the commanded velocity past max_speed_mps, breaking the
            # simulator's hard-cap assumption; re-clip the magnitude while preserving direction.
            noisy_norm = np.linalg.norm(noisy_command)
            if noisy_norm > self.max_speed_mps:
                noisy_command = noisy_command / noisy_norm * self.max_speed_mps
            self._held_command = noisy_command
            self._elapsed_since_command_sec = 0.0
            self._pending_delay_sec = self.rng.uniform(*self.reaction_delay_range)
        return self._held_command
