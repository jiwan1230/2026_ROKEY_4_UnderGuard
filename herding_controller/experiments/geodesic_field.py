"""벽을 고려한 "목표까지의 실제 최단거리" 필드 (geodesic distance field).

기존 real_map_sim.py는 Driver/Blocker의 목표 방향을 전부 직선거리(유클리드
거리)로 계산했다: `u = normalize(target_pos - goal_pos)`. 방이 미로처럼
복잡할 때, 이 직선이 벽을 가로지르면 "목표 반대편"이라고 계산한 방향이
실제로는 갈 수 없는 방향이거나, 반대로 진짜 도주로를 "목표 방향"으로
오판하게 만든다 (트러블슈팅 노트 8번 항목: 실패 시행 대부분이 표적을
트랩 근처에 데려가지도 못했던 근본 원인).

이 모듈은 Dijkstra로 트랩(목표)으로부터 모든 자유공간 셀까지의 "벽을
피해서 가는 실제 최단거리"를 1회 계산하고, 그 필드의 기울기(gradient)를
"벽을 고려한 진짜 목표 방향"으로 제공한다. 목표는 시행마다(발견 시점에)
한 번만 정해지므로, 이 계산도 시행당 한 번만 하면 된다 (매 스텝 재계산 아님).
"""
import heapq

import numpy as np

_SQRT2 = 2.0 ** 0.5
# 8방향 이웃과 각 방향의 상대 비용(직선 이동=1, 대각선 이동=sqrt2)
_NEIGHBORS = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, _SQRT2), (-1, 1, _SQRT2), (1, -1, _SQRT2), (1, 1, _SQRT2),
]


class GeodesicField:
    """장애물을 피해서 goal까지 가는 최단거리(미터)를 셀마다 담은 필드."""

    def __init__(self, grid_map, goal_row, goal_col):
        self.grid_map = grid_map
        h, w = grid_map.obstacle_mask.shape
        self.distance = np.full((h, w), np.inf)
        self._dijkstra(grid_map, goal_row, goal_col)

    def _dijkstra(self, grid_map, goal_row, goal_col):
        """goal 셀을 소스로 하는 다익스트라: 장애물 셀은 절대 통과하지 않는다.

        이동 비용은 셀 개수가 아니라 실제 미터 단위(해상도 * 1 또는
        해상도 * sqrt2)로 누적하므로, `self.distance`는 처음부터 미터
        단위의 "벽을 피해서 가는 실제 최단거리"가 된다.
        """
        res = grid_map.config.resolution_m
        h, w = self.distance.shape
        mask = grid_map.obstacle_mask
        if mask[goal_row, goal_col]:
            # 목표 셀 자체가 장애물로 마킹되어 있으면(트랩이 벽에 바짝
            # 붙어 있어 셀 경계상 그렇게 분류된 경우) 소스를 8방향 이웃 중
            # 자유공간인 셀로 대체한다. 그래도 없으면 필드 전체가 inf로
            # 남고, gradient_toward_goal()이 Euclidean으로 안전하게
            # 폴백한다.
            for dr, dc, _ in _NEIGHBORS:
                r, c = goal_row + dr, goal_col + dc
                if 0 <= r < h and 0 <= c < w and not mask[r, c]:
                    goal_row, goal_col = r, c
                    break
            else:
                return

        self.distance[goal_row, goal_col] = 0.0
        heap = [(0.0, goal_row, goal_col)]
        visited = np.zeros((h, w), dtype=bool)
        while heap:
            d, row, col = heapq.heappop(heap)
            if visited[row, col]:
                continue
            visited[row, col] = True
            for dr, dc, cost in _NEIGHBORS:
                nr, nc = row + dr, col + dc
                if not (0 <= nr < h and 0 <= nc < w):
                    continue
                if mask[nr, nc] or visited[nr, nc]:
                    continue
                nd = d + cost * res
                if nd < self.distance[nr, nc]:
                    self.distance[nr, nc] = nd
                    heapq.heappush(heap, (nd, nr, nc))

    def gradient_toward_goal(self, position, radius_cells=6):
        """position에서 "벽을 피해 goal에 실제로 가까워지는" 단위벡터를 반환한다.

        `_wall_repulsion_direction`(real_map_sim.py)과 동일한 중심차분
        기울기 계산이지만, 대상 필드가 "가장 가까운 벽까지 거리"가 아니라
        "목표까지의 geodesic 거리"라는 점이 다르다. 이 필드는 감소하는
        방향이 곧 "목표로 다가가는, 벽을 피해서 실제로 갈 수 있는 방향"이므로
        -gradient를 취한다. 계산 불가능한 경우(inf, 그리드 밖, 평평한
        지점)에는 None을 반환해서 호출부가 유클리드 방향으로 안전하게
        폴백할 수 있게 한다.

        `radius_cells`(기본 1칸=0.05m)로 이웃을 더 멀리 잡을수록 기울기가
        더 부드러워진다. geodesic distance field는 코너/문턱 주변에서
        파면(wavefront)이 꺾이므로, 바로 옆 1칸만 보면 방향이 프레임마다
        들쭉날쭉 바뀔 수 있다(실측: radius=1일 때 오히려 성공률이
        42%->34%로 떨어짐, 트러블슈팅 노트 참고) -- 더 넓게 평균 내면
        이 지역적 꺾임을 눌러줄 것이라는 가설을 이 파라미터로 검증한다.
        """
        try:
            row, col = self.grid_map.world_to_cell(*position)
        except ValueError:
            return None
        h, w = self.distance.shape
        r0, r1 = max(row - radius_cells, 0), min(row + radius_cells, h - 1)
        c0, c1 = max(col - radius_cells, 0), min(col + radius_cells, w - 1)
        d_r0c0 = self.distance[row, c0]
        d_r0c1 = self.distance[row, c1]
        d_r1c0 = self.distance[r0, col]
        d_r1c1 = self.distance[r1, col]
        if not np.isfinite([d_r0c0, d_r0c1, d_r1c0, d_r1c1]).all():
            return None
        grad_x = (d_r0c1 - d_r0c0) / 2.0
        grad_y = (d_r1c1 - d_r1c0) / 2.0
        grad = np.array([grad_x, grad_y])
        norm = np.linalg.norm(grad)
        if norm < 1e-9:
            return None
        # distance는 goal에서 멀어질수록 커지므로, "goal로 다가가는" 방향은
        # -gradient(내리막) 방향이다.
        return -grad / norm

    def virtual_goal_point(self, position, lookahead_m=1.0, radius_cells=6):
        """compute_driving_point/compute_blocking_point에 goal_pos 대신 넘길, 벽을 고려한 가상 목표점.

        이 두 함수는 내부적으로 `normalize(target_pos - goal_pos)`(또는
        그 반대)로 방향만 뽑아 쓰므로, 실제 트랩 좌표 대신 "position에서
        벽을 피해 목표로 가는 방향으로 조금 나아간 가상의 점"을 넣어도
        방향 계산 결과는 동일한 의미를 가지면서 벽을 고려하게 된다.
        `lookahead_m`은 방향 계산 후 normalize로 사라지므로 실제 값은
        중요하지 않다(0이 아니기만 하면 됨).
        """
        direction = self.gradient_toward_goal(position, radius_cells=radius_cells)
        if direction is None:
            return None
        return np.asarray(position, dtype=float) + direction * lookahead_m
