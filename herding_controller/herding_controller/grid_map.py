"""Grid <-> map coordinate conversion and obstacle/wall masking."""
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class GridConfig:
    """Resolution and extent of the occupancy grid used by the herding core."""
    resolution_m: float
    width_cells: int
    height_cells: int
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0


class GridMap:
    """Converts between world (map frame) coordinates and grid cell indices."""

    def __init__(self, config: GridConfig) -> None:
        self.config = config
        self.obstacle_mask = np.zeros((config.height_cells, config.width_cells), dtype=bool)

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        """Convert a map-frame (x, y) in meters to a (row, col) cell index."""
        col = math.floor((x - self.config.origin_x_m) / self.config.resolution_m)
        row = math.floor((y - self.config.origin_y_m) / self.config.resolution_m)
        if not self.in_bounds(row, col):
            raise ValueError(f"world coordinate ({x}, {y}) is outside the grid bounds")
        return row, col

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        """Convert a (row, col) cell index to the map-frame (x, y) of its center."""
        x = self.config.origin_x_m + (col + 0.5) * self.config.resolution_m
        y = self.config.origin_y_m + (row + 0.5) * self.config.resolution_m
        return x, y

    def set_obstacle_mask_from_occupancy(self, occupancy: np.ndarray, threshold: int = 50) -> None:
        """Build the obstacle mask from a nav_msgs/OccupancyGrid-style array (>= threshold = occupied)."""
        self.obstacle_mask = occupancy >= threshold

    def is_obstacle(self, row: int, col: int) -> bool:
        """Return True if the given cell is occupied."""
        return bool(self.obstacle_mask[row, col])

    def in_bounds(self, row: int, col: int) -> bool:
        """Return True if (row, col) is within the grid extent."""
        return 0 <= row < self.config.height_cells and 0 <= col < self.config.width_cells
