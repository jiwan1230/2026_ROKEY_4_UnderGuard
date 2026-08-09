# herding_controller_dual/test/evasion_models/wall_hugger.py
"""타겟은 벽을 따라 이동하는 것을 선호하며, 로봇이 가까워지면 ReactiveFlee처럼 도망친다."""
import numpy as np

from test.evasion_models.base import EvasionModel
from test.evasion_models.reactive_flee import ReactiveFlee


class WallHugger(EvasionModel):
    """위협이 없을 때는 가장 가까운 벽을 따라가고, 로봇이 가까워지면 곧장 도망친다."""

    def __init__(self, max_speed_mps: float, flee_reaction_distance_m: float, grid_map) -> None:
        """속도 상한, 도망 반응 거리, 벽 탐색에 사용할 그리드 맵을 저장한다."""
        self.max_speed_mps = max_speed_mps
        self.flee_reaction_distance_m = flee_reaction_distance_m
        self.grid_map = grid_map
        self._flee = ReactiveFlee(max_speed_mps, flee_reaction_distance_m)

    def step(self, target_state: np.ndarray, robot_positions: list, obstacle_map, dt: float) -> np.ndarray:
        """근처에 로봇이 있으면 도망치고, 없으면 가장 가까운 벽 셀의 접선 방향을 따라간다."""
        flee_velocity = self._flee.step(target_state, robot_positions, obstacle_map, dt)
        if np.linalg.norm(flee_velocity) > 1e-9:
            return flee_velocity

        target_pos = target_state[:2]
        wall_dir = self._nearest_wall_tangent(target_pos)
        if wall_dir is None:
            return np.zeros(2)
        return wall_dir * self.max_speed_mps * 0.5

    def _nearest_wall_tangent(self, target_pos: np.ndarray) -> np.ndarray | None:
        """가장 가까운 장애물 셀을 따라가는 단위 접선 벡터를 반환한다. 근처에 없으면 None을 반환한다."""
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
        return np.array([-normal[1], normal[0]])  # 수직 벡터 = 벽을 따라가는 접선 벡터
