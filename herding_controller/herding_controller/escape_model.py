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
        """target_pos로부터의 도주 확률 분포와 상위 K개 경로를 반환한다.

        세 가지 요인(벽 관계, 로봇 반발, 관성)을 **가산적으로** 결합한 뒤
        마지막에 정규화하는 단순 가산 모델이다: 곱셈이 아니라 덧셈이므로
        한 요인이 0이어도(예: 로봇이 아주 멀어 반발이 거의 없어도) 다른
        요인들이 여전히 분포를 결정할 수 있다. 결합 후 `clip`으로 음수를
        제거하고(각 항이 이미 음수를 clip했지만 합산 자체는 항상 양수이므로
        이 줄은 방어적 안전장치), 장애물 방향을 마스킹한 뒤 마지막에
        합이 1이 되도록 정규화한다.
        """
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
        """향촉성(thigmotaxis) 기본 확률: 벽과의 관계로 8방향에 확률을 나눠 준다.

        `wall_dir`은 타겟에서 가장 가까운 장애물 셀을 향하는 단위벡터다.
        `_DIRECTIONS @ wall_dir`(내적, dot product)은 각 도주 방향이 그
        "벽 쪽 방향"과 얼마나 나란한지를 -1~1로 나타낸다:
          - dot > 0.5  → 그 방향은 대략 벽을 향한다 → "벽에 붙는다"(hug)
          - dot < -0.5 → 그 방향은 대략 벽 반대쪽(방 중앙)을 향한다 → center
          - 그 사이     → 벽과 대략 수직, 즉 "벽을 따라간다"(follow)
        실제 쥐/설치류가 개활지 중앙보다 벽을 따라 도주하는 습성
        (thigmotaxis)을 반영해 `wall_follow_p`(기본 0.70)가 가장 크고,
        `center_p`(0.10)가 가장 작다. 각 카테고리 안에서는 균등하게 나눠
        가지므로(`/ follow.sum()` 등), 해당 카테고리에 방향이 여러 개일수록
        방향 하나당 확률은 낮아진다. 근처에 벽이 없으면(`wall_dir is None`)
        선호할 벽이 없으므로 8방향 균등분포로 대체한다.
        """
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
        """로봇이 가까울수록, 그 로봇과 반대 방향의 도주 확률을 더해준다.

        각 로봇에 대해 `away`(로봇→타겟 단위벡터)와의 내적이 클수록(그
        방향이 "로봇으로부터 멀어지는" 방향에 가까울수록) 가산 가중치를
        더 준다. `np.clip(..., 0.0, None)`으로 음수 내적(로봇 쪽으로
        향하는 방향)은 0으로 잘라내 감점이 아니라 단순 무가산 처리한다 —
        여러 로봇의 위협이 겹칠 때 서로 상쇄되어 "덜 위험해 보이는" 왜곡을
        막기 위함이다. `weight = robot_repulsion_weight / dist`로 거리에
        반비례시키는 것은 "가까운 로봇일수록 훨씬 급하게 도망친다"는
        직관 — 위협이 2배 가까워지면 그 방향에 대한 반발은 2배가 아니라
        더 크게(반비례이므로) 증폭된다.
        """
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
        """현재 진행 방향과 가까운 도주 방향에 관성(momentum) 가중치를 더한다.

        방향을 갑자기 반전하기보다 하던 대로 계속 가려는 관성을 반영한다.
        `_DIRECTIONS @ heading`(현재 속도 방향과의 내적)이 클수록(진행
        방향과 가까울수록) 더 큰 가중치를 받고, 반대 방향(내적 음수)은
        `clip`으로 0 처리해 감점하지 않는다 — 관성은 "계속 갈 이유를
        더해주는 것"이지 "반대로 갈 이유를 깎는 것"이 아니기 때문에
        `_robot_repulsion`과 동일한 clip 패턴을 쓴다. 타겟이 거의 정지해
        있으면(speed < 1e-6) 참고할 진행 방향 자체가 없으므로 기여 없음.
        """
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
