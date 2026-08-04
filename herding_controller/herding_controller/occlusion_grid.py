# herding_controller/herding_controller/occlusion_grid.py
"""Bayesian belief grid for LOST-state re-search (diffusion + decay)."""
from dataclasses import dataclass

import numpy as np

from herding_controller.grid_map import GridMap


@dataclass
class OcclusionGridConfig:
    """Diffusion and decay rates for the LOST-recovery belief grid."""
    diffusion_rate: float
    decay_factor: float


class OcclusionGrid:
    """Tracks a decaying, diffusing belief of the target's location while LOST."""

    def __init__(self, config: OcclusionGridConfig, grid_map: GridMap) -> None:
        self.config = config
        self.grid_map = grid_map
        self.belief = np.zeros((grid_map.config.height_cells, grid_map.config.width_cells))

    def seed(self, row: int, col: int) -> None:
        """Reset the belief to a single point mass at the last known target cell."""
        self.belief = np.zeros_like(self.belief)
        self.belief[row, col] = 1.0

    def step(self, dt: float) -> None:
        """Diffuse belief to 4-neighbors, mask obstacles, and decay total mass."""
        # Clamp the diffusion weight itself, not just the result: 0.25 is the
        # standard stability bound for this explicit finite-difference
        # 4-neighbor stencil on a uniform grid. It guarantees
        # (1 - 4*weight) >= 0 unconditionally, so the diffusion step below is
        # mass-conserving (never inflates total belief) regardless of how
        # large diffusion_rate * dt gets -- e.g. from a stalled/delayed ROS2
        # timer callback producing an unusually large dt.
        weight = min(self.config.diffusion_rate * dt, 0.25)
        # Mass-conserving diffusion: each cell keeps (1 - 4*weight) of its own
        # belief and gains weight * neighbor for each of its 4-neighbors. Cells
        # on the grid boundary still lose the full 4*weight outflow but have
        # fewer than 4 neighbors to receive from, so mass "diffusing off the
        # map edge" is lost there (mirrors the obstacle case below).
        diffused = self.belief * (1.0 - 4.0 * weight)
        diffused[1:, :] += self.belief[:-1, :] * weight
        diffused[:-1, :] += self.belief[1:, :] * weight
        diffused[:, 1:] += self.belief[:, :-1] * weight
        diffused[:, :-1] += self.belief[:, 1:] * weight
        diffused[self.grid_map.obstacle_mask] = 0.0

        # Apply decay directly to the (already mass-conserving) diffused
        # belief. Do NOT renormalize back up to a total of 1.0 here: doing so
        # would erase every previous step's decay and pin the total mass at
        # exactly decay_factor forever instead of letting it fall off
        # geometrically (decay_factor ** n) as more steps elapse unobserved.
        self.belief = diffused * self.config.decay_factor

    def best_guess_cell(self) -> tuple[int, int]:
        """Return the (row, col) of the highest-belief cell as the re-search target."""
        row, col = np.unravel_index(np.argmax(self.belief), self.belief.shape)
        return int(row), int(col)
