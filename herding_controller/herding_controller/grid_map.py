"""그리드 <-> 맵 좌표 변환 및 장애물/벽 마스킹."""
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class GridConfig:
    """herding core에서 사용하는 점유 격자의 해상도와 범위."""
    resolution_m: float
    width_cells: int
    height_cells: int
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0


class GridMap:
    """월드(맵 프레임) 좌표와 그리드 셀 인덱스 사이를 변환한다."""

    def __init__(self, config: GridConfig) -> None:
        self.config = config
        self.obstacle_mask = np.zeros((config.height_cells, config.width_cells), dtype=bool)

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        """미터 단위의 맵 프레임 (x, y)를 (row, col) 셀 인덱스로 변환한다."""
        col = math.floor((x - self.config.origin_x_m) / self.config.resolution_m)
        row = math.floor((y - self.config.origin_y_m) / self.config.resolution_m)
        if not self.in_bounds(row, col):
            raise ValueError(f"world coordinate ({x}, {y}) is outside the grid bounds")
        return row, col

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        """(row, col) 셀 인덱스를 그 중심의 맵 프레임 (x, y)로 변환한다."""
        x = self.config.origin_x_m + (col + 0.5) * self.config.resolution_m
        y = self.config.origin_y_m + (row + 0.5) * self.config.resolution_m
        return x, y

    def set_obstacle_mask_from_occupancy(self, occupancy: np.ndarray, threshold: int = 50) -> None:
        """nav_msgs/OccupancyGrid 형식의 배열로부터 장애물 마스크를 생성한다 (>= threshold = 점유됨)."""
        self.obstacle_mask = occupancy >= threshold

    def is_obstacle(self, row: int, col: int) -> bool:
        """주어진 셀이 점유되어 있으면 True를 반환한다."""
        return bool(self.obstacle_mask[row, col])

    def in_bounds(self, row: int, col: int) -> bool:
        """(row, col)이 그리드 범위 내에 있으면 True를 반환한다."""
        return 0 <= row < self.config.height_cells and 0 <= col < self.config.width_cells
