# herding_controller/test/evasion_models/wall_hugger.py
"""Target prefers moving along walls; flees like ReactiveFlee when a robot is close."""
import numpy as np

from test.evasion_models.base import EvasionModel
from test.evasion_models.reactive_flee import ReactiveFlee


class WallHugger(EvasionModel):
    """Hugs the nearest wall when unthreatened, flees directly when a robot is close."""

    def __init__(self, max_speed_mps: float, flee_reaction_distance_m: float, grid_map) -> None:
        self.max_speed_mps = max_speed_mps
        self.flee_reaction_distance_m = flee_reaction_distance_m
        self.grid_map = grid_map
        self._flee = ReactiveFlee(max_speed_mps, flee_reaction_distance_m)

    def step(self, target_state: np.ndarray, robot_positions: list, obstacle_map, dt: float) -> np.ndarray:
        flee_velocity = self._flee.step(target_state, robot_positions, obstacle_map, dt)
        if np.linalg.norm(flee_velocity) > 1e-9:
            return flee_velocity

        target_pos = target_state[:2]
        wall_dir = self._nearest_wall_tangent(target_pos)
        if wall_dir is None:
            return np.zeros(2)
        return wall_dir * self.max_speed_mps * 0.5

    def _nearest_wall_tangent(self, target_pos: np.ndarray) -> np.ndarray | None:
        try:
            row, col = self.grid_map.world_to_cell(*target_pos)
        except ValueError:
            return None
        radius = 3
        row_lo, row_hi = max(0, row - radius), min(self.grid_map.config.height_cells, row + radius + 1)
        col_lo, col_hi = max(0, col - radius), min(self.grid_map.config.width_cells, col + radius + 1)
        window = self.grid_map.obstacle_mask[row_lo:row_hi, col_lo:col_hi]
        if not window.any():
            return None
        rows, cols = np.nonzero(window)
        offsets = np.stack([cols - (col - col_lo), rows - (row - row_lo)], axis=1).astype(float)
        nearest = offsets[np.argmin(np.linalg.norm(offsets, axis=1))]
        norm = np.linalg.norm(nearest)
        if norm < 1e-9:
            return None
        normal = nearest / norm
        return np.array([-normal[1], normal[0]])  # perpendicular = tangent along the wall
