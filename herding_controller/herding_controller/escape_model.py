# herding_controller/herding_controller/escape_model.py
"""타겟의 도주 방향을 예측하는 그리드 기반 마르코프 모델."""
from dataclasses import dataclass

import numpy as np

from herding_controller.grid_map import GridMap

_DIRECTIONS = np.array(
    [[0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]],
    dtype=float,
)
_DIRECTIONS /= np.linalg.norm(_DIRECTIONS, axis=1, keepdims=True)


@dataclass
class EscapeModelConfig:
    """벽 추종(thigmotaxis) 기본 가중치와 로봇 반발/관성 항."""
    wall_follow_p: float
    wall_hug_p: float
    center_p: float
    momentum_weight: float
    robot_repulsion_weight: float
    wall_detect_radius_cells: int
    escape_route_top_k: int


@dataclass
class EscapeEstimate:
    """도주 방향 확률 분포와 상위 K개 후보 경로."""
    directions: np.ndarray
    probabilities: np.ndarray
    top_k_routes: list[np.ndarray]


class EscapeModel:
    """타겟이 8방위 중 어느 방향으로 도주할 가능성이 높은지 예측한다."""

    def __init__(self, config: EscapeModelConfig, grid_map: GridMap) -> None:
        self.config = config
        self.grid_map = grid_map

    def compute(
        self, target_pos: np.ndarray, target_vel: np.ndarray, robot_positions: list[np.ndarray]
    ) -> EscapeEstimate:
        """target_pos로부터의 도주 확률 분포와 상위 K개 경로를 반환한다."""
        wall_dir = self._nearest_wall_direction(target_pos)
        base = self._base_weights(wall_dir)
        base += self._robot_repulsion(target_pos, robot_positions)
        base += self._momentum(target_vel)
        base = np.clip(base, a_min=0.0, a_max=None)
        base = self._mask_obstacles(target_pos, base)

        total = base.sum()
        if total <= 1e-9:
            valid = self._valid_mask(target_pos)
            if valid.any():
                base = valid.astype(float)
            else:
                # 완전히 갇힌 상태: 유효한 방향이 하나도 없다. 무효한 all-zero
                # 퇴화 벡터를 만드는 대신 균등 분포로 대체하여 확률의 합이
                # 여전히 1이 되도록 하고, "좋은 선택지가 없으며 모두 동등하게
                # 나쁘다"는 것을 나타낸다.
                base = np.full(8, 1.0 / 8.0)
            total = base.sum()
        probabilities = base / total

        routes = self._top_k_routes(target_pos, probabilities)
        return EscapeEstimate(directions=_DIRECTIONS.copy(), probabilities=probabilities, top_k_routes=routes)

    def _nearest_wall_direction(self, target_pos: np.ndarray) -> np.ndarray | None:
        row, col = self.grid_map.world_to_cell(*target_pos)
        radius = self.config.wall_detect_radius_cells
        row_lo, row_hi = max(0, row - radius), min(self.grid_map.config.height_cells, row + radius + 1)
        col_lo, col_hi = max(0, col - radius), min(self.grid_map.config.width_cells, col + radius + 1)
        window = self.grid_map.obstacle_mask[row_lo:row_hi, col_lo:col_hi]
        if not window.any():
            return None
        rows, cols = np.nonzero(window)
        offsets = np.stack([cols - (col - col_lo), rows - (row - row_lo)], axis=1).astype(float)
        nearest = offsets[np.argmin(np.linalg.norm(offsets, axis=1))]
        norm = np.linalg.norm(nearest)
        return nearest / norm if norm > 1e-9 else None

    def _base_weights(self, wall_dir: np.ndarray | None) -> np.ndarray:
        if wall_dir is None:
            return np.full(8, 1.0 / 8.0)
        dots = _DIRECTIONS @ wall_dir
        hug = dots > 0.5
        center = dots < -0.5
        follow = ~hug & ~center
        weights = np.zeros(8)
        if follow.any():
            weights[follow] = self.config.wall_follow_p / follow.sum()
        if hug.any():
            weights[hug] = self.config.wall_hug_p / hug.sum()
        if center.any():
            weights[center] = self.config.center_p / center.sum()
        return weights

    def _robot_repulsion(self, target_pos: np.ndarray, robot_positions: list[np.ndarray]) -> np.ndarray:
        contribution = np.zeros(8)
        for robot_pos in robot_positions:
            away = target_pos - robot_pos
            dist = np.linalg.norm(away)
            if dist < 1e-6:
                continue
            away = away / dist
            weight = self.config.robot_repulsion_weight / dist
            contribution += np.clip(_DIRECTIONS @ away, 0.0, None) * weight
        return contribution

    def _momentum(self, target_vel: np.ndarray) -> np.ndarray:
        speed = np.linalg.norm(target_vel)
        if speed < 1e-6:
            return np.zeros(8)
        heading = target_vel / speed
        return np.clip(_DIRECTIONS @ heading, 0.0, None) * self.config.momentum_weight

    def _valid_mask(self, target_pos: np.ndarray) -> np.ndarray:
        row, col = self.grid_map.world_to_cell(*target_pos)
        valid = np.zeros(8, dtype=bool)
        for i, (dx, dy) in enumerate(_DIRECTIONS):
            next_row, next_col = row + int(round(dy)), col + int(round(dx))
            if self.grid_map.in_bounds(next_row, next_col) and not self.grid_map.is_obstacle(next_row, next_col):
                valid[i] = True
        return valid

    def _mask_obstacles(self, target_pos: np.ndarray, weights: np.ndarray) -> np.ndarray:
        valid = self._valid_mask(target_pos)
        return np.where(valid, weights, 0.0)

    def _top_k_routes(self, target_pos: np.ndarray, probabilities: np.ndarray) -> list[np.ndarray]:
        valid = self._valid_mask(target_pos)
        if valid.any():
            # 장애물 셀로 진입하지 않는 방향 중에서만 선택한다.
            # 유효한 방향이 escape_route_top_k보다 적으면, 장애물 셀에
            # 도달하는 경로를 포함시키기보다 더 적은 수의 경로를 반환한다.
            candidate_indices = np.nonzero(valid)[0]
        else:
            # 완전히 갇힌 상태: 유효한 방향이 하나도 없어 정말로 좋은 선택지가
            # 없다. compute()에서 계산된 균등 대체 확률을 기준으로 8방향
            # 전체의 순위를 매긴다 — 이 경우 경로가 장애물 셀을 가리켜도
            # 무방하며, 이는 "좋은 선택지가 없음"을 나타낸다.
            candidate_indices = np.arange(len(probabilities))
        order = candidate_indices[np.argsort(probabilities[candidate_indices])[::-1]]
        k = min(self.config.escape_route_top_k, len(order))
        top_indices = order[:k]
        lookahead_m = self.grid_map.config.resolution_m * 3
        return [target_pos + _DIRECTIONS[i] * lookahead_m for i in top_indices]
