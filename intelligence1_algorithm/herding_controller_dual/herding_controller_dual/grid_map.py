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
        # shape=(height, width) — world_to_cell/cell_to_world이 쓰는 (row, col)
        # 인덱싱과 순서를 맞춘 것. numpy 배열의 첫 축(행)이 row(=y), 둘째 축(열)이
        # col(=x)이라, 다른 곳에서처럼 여기서도 (x, y)가 아니라 (row, col) 순서로
        # 인덱싱해야 한다(예: is_obstacle(row, col), 반대로 하면 조용히 잘못된 셀을 가리킨다).
        self.obstacle_mask = np.zeros((config.height_cells, config.width_cells), dtype=bool)

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        """미터 단위의 맵 프레임 (x, y)를 (row, col) 셀 인덱스로 변환한다.

        row = ROS 맵 프레임의 y축(북쪽), col = x축(동쪽)에 대응한다 — 즉 이
        그리드는 (row, col)이 (y, x) 순서라는 점에 주의. `origin_*_m`을 뺀 뒤
        `resolution_m`으로 나누는 것은 "원점으로부터 몇 칸 떨어져 있는가"를
        계산하는 것이고, `floor()`를 쓰는 이유는 각 셀이 [셀 시작,
        셀 시작+resolution) 구간의 반열림 구간을 담당하기 때문이다 (예:
        resolution=0.25일 때 x=0.24와 x=0.0은 같은 셀 0, x=0.25는 다음 셀 1).
        """
        col = math.floor((x - self.config.origin_x_m) / self.config.resolution_m)
        row = math.floor((y - self.config.origin_y_m) / self.config.resolution_m)
        if not self.in_bounds(row, col):
            raise ValueError(f"world coordinate ({x}, {y}) is outside the grid bounds")
        return row, col

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        """(row, col) 셀 인덱스를 그 중심의 맵 프레임 (x, y)로 변환한다.

        `world_to_cell`의 역연산이지만 셀의 좌측/하단 경계가 아니라 **중심**을
        반환한다 (+0.5 오프셋) — occlusion grid의 재탐색 목표점처럼 "이 셀
        어딘가"를 로봇에게 구체적인 좌표로 줄 때, 셀 경계보다 중심이 더
        안전한 목표점이기 때문이다 (경계에 딱 붙으면 부동소수점 오차로
        옆 셀로 다시 튕겨 나갈 수 있음).
        """
        x = self.config.origin_x_m + (col + 0.5) * self.config.resolution_m
        y = self.config.origin_y_m + (row + 0.5) * self.config.resolution_m
        return x, y

    def set_obstacle_mask_from_occupancy(self, occupancy: np.ndarray, threshold: int = 50) -> None:
        """nav_msgs/OccupancyGrid 형식의 배열로부터 장애물 마스크를 생성한다 (>= threshold = 점유됨).

        ROS의 OccupancyGrid는 각 셀에 -1(unknown)~100(확실히 점유) 사이의
        점유 확률을 담는다. 기본 threshold=50은 "절반 이상의 확률로
        점유되어 있다고 보고되면 장애물로 취급"이라는 보수적인 기준이며,
        -1(unknown)은 50 미만이므로 자동으로 "장애물 아님"(자유공간과 동일)
        취급된다는 점에 유의 — 즉 미탐색 영역은 통행 가능하다고 낙관적으로
        가정한다.
        """
        self.obstacle_mask = occupancy >= threshold

    def is_obstacle(self, row: int, col: int) -> bool:
        """주어진 셀이 점유되어 있으면 True를 반환한다."""
        return bool(self.obstacle_mask[row, col])

    def in_bounds(self, row: int, col: int) -> bool:
        """(row, col)이 그리드 범위 내에 있으면 True를 반환한다."""
        return 0 <= row < self.config.height_cells and 0 <= col < self.config.width_cells
