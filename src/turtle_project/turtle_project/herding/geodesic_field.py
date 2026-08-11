# herding_controller/herding_controller/geodesic_field.py
"""벽을 고려한 "목표까지의 실제 최단거리" 필드 (geodesic distance field).

`herding_planner.py`의 `compute_driving_point`/`compute_blocking_point`는
목표 방향을 전부 직선거리(유클리드 거리)로 계산한다:
`u = normalize(target_pos - goal_pos)`. 방이 미로처럼 복잡할 때, 이 직선이
벽을 가로지르면 "목표 반대편"이라고 계산한 방향이 실제로는 갈 수 없는
방향이거나, 반대로 진짜 도주로를 "목표 방향"으로 오판하게 만든다
(트러블슈팅 노트 8/10번 항목: 실제 SLAM 맵에서 실패 시행 대부분이 표적을
포획존 근처에 데려가지도 못했던 원인 중 하나로 확인됨).

이 모듈은 Dijkstra로 포획존(목표)으로부터 모든 자유공간 셀까지의 "벽을
피해서 가는 실제 최단거리"를 계산하고, 그 필드의 기울기(gradient)를 "벽을
고려한 진짜 목표 방향"으로 제공한다. `herding_core.py`가 이 필드를 goal_pos
기준으로 1회만 계산해 캐싱한다 (goal_pos는 설정값으로 미션 내내 고정이므로
매 제어 주기 재계산할 필요가 없다 -- 제어 주기 5Hz에서 매번 다익스트라를
돌리면 지연시간 예산을 넘긴다).
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

        중심차분(central difference)으로 기울기를 근사한다. 이 필드는 감소하는
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
        d_left = self.distance[row, c0]
        d_right = self.distance[row, c1]
        d_down = self.distance[r0, col]
        d_up = self.distance[r1, col]
        if not np.isfinite([d_left, d_right, d_down, d_up]).all():
            return None
        grad_x = (d_right - d_left) / 2.0
        grad_y = (d_up - d_down) / 2.0
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

        한계: 이 함수는 position 한 지점에서의 **국소** 기울기 하나만 보고
        그 방향으로 직선 연장한 것이다. 복도가 코너를 돌아야 하는 상황에서는
        국소 기울기가 "지금 당장 벽에서 벗어나는 방향"만 알려줄 뿐, 그
        다음 코너가 어디서 꺾이는지는 반영하지 못한다 -- 미로형 실제 맵에서
        측정한 실질적 개선폭이 작았던 원인 중 하나로 보인다(+3.8%p,
        트러블슈팅 노트 9번 항목). 여러 구간을 앞서 내다보고 싶다면
        `waypoint_ahead()`를 쓸 것.
        """
        direction = self.gradient_toward_goal(position, radius_cells=radius_cells)
        if direction is None:
            return None
        return np.asarray(position, dtype=float) + direction * lookahead_m

    def trace_path(self, start_pos, step_m=0.15, max_points=400):
        """start_pos에서 goal까지, 매 지점의 국소 기울기를 따라가며 만든 경로(웨이포인트 나열).

        `virtual_goal_point()`가 한 지점의 기울기 하나만 보고 직선으로
        연장하는 것과 달리, 이 함수는 실제로 그 기울기를 따라 `step_m`씩
        전진하기를 반복한다 -- 그러다 보면 경로가 실제 복도를 따라 코너를
        돌아나가는 형태로 이어진다. `max_points * step_m`이 추적 가능한
        최대 경로 길이다.
        """
        path = [np.asarray(start_pos, dtype=float)]
        current = np.asarray(start_pos, dtype=float)
        for _ in range(max_points):
            try:
                row, col = self.grid_map.world_to_cell(*current)
            except ValueError:
                break
            if not np.isfinite(self.distance[row, col]) or self.distance[row, col] <= step_m:
                break  # 목표 근처에 도달
            direction = self.gradient_toward_goal(current, radius_cells=3)
            if direction is None:
                break
            current = current + direction * step_m
            path.append(current.copy())
        return path

    def waypoint_ahead(self, position, lookahead_m=1.5, step_m=0.15):
        """position에서 goal로 가는 실제 경로를 따라 lookahead_m만큼 나아간 지점.

        `trace_path()`로 실제 복도를 따라가는 경로를 만든 뒤, 그 경로의
        누적 길이가 lookahead_m에 도달하는 지점을 반환한다 (직선거리가
        아니라 경로를 따라간 거리 기준). 이렇게 하면 다가올 코너를
        미리 내다보고 그 방향으로 목표 지점을 잡을 수 있다 -- 매 제어
        주기(target 위치가 바뀔 때)마다 다시 계산되므로 별도의 "다음
        웨이포인트 인덱스" 상태를 들고 다닐 필요가 없다(경로 자체가
        매번 표적의 현재 위치에서 새로 그려짐).

        경로가 lookahead_m보다 짧으면(목표 바로 근처) 경로의 끝(목표
        근처)을 그대로 반환한다. 경로를 하나도 못 그리면(그리드 밖 등)
        None을 반환해 호출부가 폴백할 수 있게 한다.
        """
        path = self.trace_path(position, step_m=step_m, max_points=int(lookahead_m / step_m) + 40)
        if len(path) < 2:
            return None
        traveled = 0.0
        for i in range(1, len(path)):
            traveled += float(np.linalg.norm(path[i] - path[i - 1]))
            if traveled >= lookahead_m:
                return path[i]
        return path[-1]
