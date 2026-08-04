# herding_controller/test/evasion_models/reactive_flee.py
"""Primary validation model: target flees directly away from the nearest robot."""
import numpy as np

from test.evasion_models.base import EvasionModel


class ReactiveFlee(EvasionModel):
    """Flees straight away from whichever robot is within flee_reaction_distance_m."""

    def __init__(self, max_speed_mps: float, flee_reaction_distance_m: float) -> None:
        """Store the speed cap and the distance within which a robot triggers fleeing."""
        self.max_speed_mps = max_speed_mps
        self.flee_reaction_distance_m = flee_reaction_distance_m

    def step(self, target_state: np.ndarray, robot_positions: list, obstacle_map, dt: float) -> np.ndarray:
        """Sum weighted away-from-robot vectors for every nearby robot and cap the result."""
        target_pos = target_state[:2]
        flee_dir = np.zeros(2)
        for robot_pos in robot_positions:
            away = target_pos - robot_pos
            dist = np.linalg.norm(away)
            if dist < self.flee_reaction_distance_m and dist > 1e-6:
                flee_dir += (away / dist) * (self.flee_reaction_distance_m - dist)
        norm = np.linalg.norm(flee_dir)
        if norm < 1e-9:
            return np.zeros(2)
        return (flee_dir / norm) * self.max_speed_mps
