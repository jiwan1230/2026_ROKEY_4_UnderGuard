# herding_controller_dual/herding_controller_dual/escape_model.py
""""쥐가 다음에 어느 쪽으로 도망갈까?"를 8방향 확률로 짐작하는 모델 (마르코프 모델)."""
from dataclasses import dataclass

import numpy as np

from turtle_project.herding_dual.grid_map import GridMap

# 8방위 화살표들: [북, 북동, 동, 남동, 남, 남서, 서, 북서] 순서로 45도씩 돌아간다
# (world_to_cell에서 쓰는 row=y/col=x 순서와 다르게, 여기 화살표는 그냥
# world 좌표계 그대로 (x, y)다).
# 대각선 화살표(북동 등)는 원래 길이가 sqrt(2)라서 위아래/좌우 화살표보다
# 더 길다 — 그대로 두면 이 파일에서 계속 쓰는 "화살표 내적 비교"(벽 방향과
# 얼마나 나란한지, 로봇 반대 방향과 얼마나 나란한지)가 대각선 방향에서만
# 유독 크게 나오는 왜곡이 생긴다. 그래서 미리 길이를 전부 1로 맞춰둔다.
_DIRECTIONS = np.array(
    [[0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]],
    dtype=float,
)
_DIRECTIONS /= np.linalg.norm(_DIRECTIONS, axis=1, keepdims=True)


@dataclass
class EscapeModelConfig:
    """벽 옆을 좋아하는 정도(향촉성) + 로봇을 무서워하는 정도 + 관성을 정하는 값들."""
    wall_follow_p: float
    wall_hug_p: float
    center_p: float
    momentum_weight: float
    robot_repulsion_weight: float
    wall_detect_radius_cells: int
    escape_route_top_k: int
    # Blocker 역할인 로봇이 이 거리(m)보다 멀리 있으면, "로봇이 무서워서
    # 도망간다" 계산에서 그 로봇은 아예 빼고 계산한다. 기본값 inf는 이
    # 기능을 끈 것과 같다(원래 동작 그대로). 왜 필요했냐면 -- 트러블슈팅
    # 노트 11-9/11-10/12 참고 -- Blocker가 몰이 시작부터 한참 멀리
    # 대기하고 있어도 "무서운 로봇"으로 계속 계산에 들어가서, 쥐의 도망
    # 경로를 초반부터 미묘하게 비틀어놓는다는 의심을 확인해보려던 값이다.
    robot_repulsion_activation_distance_m: float = float("inf")


@dataclass
class EscapeEstimate:
    """"8방향 중 어디로 몇 %씩 도망갈 것 같다"는 결과 + 제일 유력한 경로 몇 개."""
    directions: np.ndarray
    probabilities: np.ndarray
    top_k_routes: list[np.ndarray]


class EscapeModel:
    """쥐가 8방향(동서남북+대각선) 중 어디로 도망갈 확률이 높은지 계산한다."""

    def __init__(self, config: EscapeModelConfig, grid_map: GridMap) -> None:
        self.config = config
        self.grid_map = grid_map

    def compute(
        self, target_pos: np.ndarray, target_vel: np.ndarray, robot_positions: list[np.ndarray],
        blocker_index: int = 1,
    ) -> EscapeEstimate:
        """지금 위치에서 8방향 도망 확률과, 제일 유력한 경로 몇 개를 돌려준다.

        쥐가 도망갈 이유를 세 가지로 나눠서 각각 점수를 매기고, 그냥
        **더한다**: (1) 벽 근처를 좋아하는 습성, (2) 로봇이 무서워서
        반대로 도망가려는 마음, (3) 가던 방향으로 계속 가려는 관성.
        곱하지 않고 더하는 이유는, 한 이유가 0점이어도(예: 로봇이 너무
        멀어서 안 무서워도) 나머지 이유들만으로 여전히 방향을 정할 수
        있게 하기 위해서다 — 곱하면 하나라도 0이면 전체가 0이 돼버린다.

        다 더한 다음 음수는 0으로 자르고(각 이유 안에서 이미 잘랐지만,
        혹시 몰라 한 번 더 확인하는 안전장치), 벽 쪽으로 막힌 방향은
        확률을 0으로 지운 뒤, 마지막에 전체 합이 1이 되도록 나눠서
        "몇 % 확률로 이 방향" 형태로 정리한다.

        `blocker_index`는 지금 이 순간 두 로봇 중 어느 쪽이 "막는
        역할(Blocker)"인지 알려주는 값(0번 또는 1번 로봇)이다. 플랜
        A(herding_controller)는 역할이 안 바뀌니까 그냥 기본값(1번 로봇)을
        쓰면 되지만, 이 패키지(플랜 B)는 두 로봇 역할이 매 순간 바뀔 수
        있어서 부르는 쪽(herding_core.py)이 그 순간의 진짜 Blocker가 누구인지
        매번 알려줘야 한다.
        """
        wall_dir = self._nearest_wall_direction(target_pos)  # 근처에 벽이 있으면 그 방향 (없으면 None)
        base = self._base_weights(wall_dir)                  # 이유 1: 벽 옆을 좋아하는 습성
        base += self._robot_repulsion(target_pos, robot_positions, blocker_index)  # 이유 2: 로봇이 무서워서
        base += self._momentum(target_vel)                   # 이유 3: 가던 방향으로 계속 가려는 관성
        base = np.clip(base, a_min=0.0, a_max=None)           # 혹시 모를 음수 방지 (안전장치)
        base = self._mask_obstacles(target_pos, base)         # 벽으로 막힌 방향은 확률 0으로

        total = base.sum()
        if total <= 1e-9:  # 8방향 전부 점수 0 (사방이 막혔거나 이유들이 전부 0) — 이런 경우를 대비
            valid = self._valid_mask(target_pos)
            if valid.any():
                base = valid.astype(float)  # 갈 수 있는 방향들에 똑같이 나눠준다
            else:
                # 정말로 사방이 다 막혀서 갈 수 있는 방향이 하나도 없다.
                # 그래도 "확률 합은 1"이라는 약속을 지키려고 8방향에
                # 똑같이 나눠준다 — "어디로 가도 다 안 좋다"는 뜻이다.
                base = np.full(8, 1.0 / 8.0)
            total = base.sum()
        probabilities = base / total  # 합이 1이 되도록 정리 (진짜 확률처럼)

        routes = self._top_k_routes(target_pos, probabilities)
        return EscapeEstimate(directions=_DIRECTIONS.copy(), probabilities=probabilities, top_k_routes=routes)

    def _nearest_wall_direction(self, target_pos: np.ndarray) -> np.ndarray | None:
        """쥐 근처 몇 칸 안에서 제일 가까운 벽이 어느 방향에 있는지. 벽이 없으면 None."""
        row, col = self.grid_map.world_to_cell(*target_pos)   # 쥐가 있는 칸
        radius = self.config.wall_detect_radius_cells
        # 쥐를 가운데 두고 사각형 범위를 살펴본다 (그리드 밖으로 안 나가게 자름)
        row_lo, row_hi = max(0, row - radius), min(self.grid_map.config.height_cells, row + radius + 1)
        col_lo, col_hi = max(0, col - radius), min(self.grid_map.config.width_cells, col + radius + 1)
        window = self.grid_map.obstacle_mask[row_lo:row_hi, col_lo:col_hi]  # 그 범위만 잘라서 확인
        if not window.any():
            return None  # 이 범위 안에 벽이 하나도 없음 — 근처에 벽 없음
        rows, cols = np.nonzero(window)  # 범위 안에서 벽인 칸들 (범위 기준 좌표)
        # 범위 기준 좌표를 "쥐를 기준으로 한 상대 위치"로 바꾼다
        offsets = np.stack([cols - (col - col_lo), rows - (row - row_lo)], axis=1).astype(float)
        nearest = offsets[np.argmin(np.linalg.norm(offsets, axis=1))]  # 그중 쥐랑 제일 가까운 벽
        norm = np.linalg.norm(nearest)
        return nearest / norm if norm > 1e-9 else None  # 방향만 남기고 길이는 1로. (쥐가 벽 칸 안에 있으면 방향 미정)

    def _base_weights(self, wall_dir: np.ndarray | None) -> np.ndarray:
        """"벽 옆을 좋아하는 습성"만 반영한 8방향 확률.

        진짜 쥐/설치류는 뻥 뚫린 방 한가운데보다 벽을 따라 도망다니는
        습성이 있다(향촉성, thigmotaxis). 그래서 8방향을 세 그룹으로
        나눈다: 벽을 따라가는 방향(follow, 70%로 제일 큼), 벽에 딱
        붙는 방향(hug, 20%), 방 한가운데로 가는 방향(center, 10%로
        제일 작음). `wall_dir`(가까운 벽 방향)과 각 화살표를 비교해서
        어느 그룹인지 나눈다 — 화살표가 벽 쪽을 많이 향하면 hug, 벽
        반대쪽(방 중앙)을 많이 향하면 center, 그 사이면 follow다.
        같은 그룹 안에서는 방향 개수만큼 똑같이 나눠 갖는다(그룹에
        방향이 여러 개면 하나당 확률은 그만큼 낮아진다). 근처에 벽이
        아예 없으면(`wall_dir is None`) 특별히 좋아할 벽이 없으니
        8방향에 똑같이 나눠준다.
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

    def _robot_repulsion(
        self, target_pos: np.ndarray, robot_positions: list[np.ndarray], blocker_index: int = 1,
    ) -> np.ndarray:
        """"로봇이 무서워서 반대로 도망가고 싶은 마음"만 반영한 점수.

        로봇마다 "그 로봇에서 멀어지는 방향"을 계산해서, 8방향 화살표가
        그 방향과 비슷할수록 점수를 더 준다. 로봇 쪽으로 향하는 방향은
        (내적이 음수가 나오면) 점수를 깎지 않고 그냥 0으로 처리한다 —
        여러 로봇이 동시에 있을 때 "이쪽은 무섭고 저쪽은 안 무섭다"가
        서로 상쇄돼서 "생각보다 안 무섭다"고 착각하는 걸 막기 위해서다.
        가까운 로봇일수록 점수를 훨씬 크게 준다(거리에 반비례) — 로봇이
        2배 가까워지면 무서움은 2배보다 더 커진다는 뜻이다.

        `blocker_index`로 지정된 로봇(그 순간의 Blocker)이 정해둔
        거리보다 멀리 있으면, 그 로봇은 이 계산에서 아예 빼버린다. 다른
        로봇(Driver)은 이 규칙의 영향을 안 받는다 -- Driver는 실제로
        쥐를 쫓는 로봇이라 항상 무서운 게 맞지만, Blocker는 몰이 초반에
        쥐와 멀리 떨어진 대기 지점에 가만히 있는데도 "무서운 로봇"으로
        계속 계산에 들어가서 쥐의 초반 도망 경로를 이상하게 비트는 게
        아닌가 하는 의심(트러블슈팅 노트 11-9)을 확인해보려는 장치다.
        플랜 A는 역할이 안 바뀌니 기본값(1번 로봇)이면 되고, 플랜
        B는 부르는 쪽이 그 순간의 진짜 Blocker 번호를 넘겨준다.
        """
        contribution = np.zeros(8)
        for i, robot_pos in enumerate(robot_positions):
            away = target_pos - robot_pos     # 로봇에서 쥐를 향하는 방향 = 쥐가 멀어져야 할 방향
            dist = np.linalg.norm(away)
            if dist < 1e-6:
                continue  # 로봇과 쥐가 같은 자리 — 방향을 정할 수 없으니 이 로봇은 그냥 무시
            if i == blocker_index and dist > self.config.robot_repulsion_activation_distance_m:
                continue  # 이 로봇이 Blocker인데 너무 멀리 있음 — 계산에서 제외
            away = away / dist               # 방향만 남기고 길이는 1로
            weight = self.config.robot_repulsion_weight / dist  # 가까울수록 큰 점수 (거리에 반비례)
            contribution += np.clip(_DIRECTIONS @ away, 0.0, None) * weight  # 멀어지는 방향들에 점수 쌓기
        return contribution

    def _momentum(self, target_vel: np.ndarray) -> np.ndarray:
        """"가던 방향으로 계속 가려는 관성"만 반영한 점수.

        사람도 그렇듯 쥐도 갑자기 정반대로 확 틀지 않고 하던 방향으로
        계속 가려는 경향이 있다고 가정한다. 지금 움직이는 방향과 비슷한
        화살표일수록 점수를 더 주고, 반대 방향은 깎지 않고 그냥 0점
        처리한다(`_robot_repulsion`과 같은 방식) — 관성은 "계속 갈
        이유를 더해주는 것"이지 "반대로 갈 이유를 깎는 것"은 아니기
        때문이다. 쥐가 거의 멈춰 있으면(속도가 거의 0) 참고할 방향
        자체가 없으니 이 점수는 그냥 0이다.
        """
        speed = np.linalg.norm(target_vel)
        if speed < 1e-6:
            return np.zeros(8)
        heading = target_vel / speed
        return np.clip(_DIRECTIONS @ heading, 0.0, None) * self.config.momentum_weight

    def _valid_mask(self, target_pos: np.ndarray) -> np.ndarray:
        """8방향 중 "그쪽으로 한 칸 가도 벽이 아니고 그리드 안"인 방향만 True로 표시."""
        row, col = self.grid_map.world_to_cell(*target_pos)
        valid = np.zeros(8, dtype=bool)
        for i, (dx, dy) in enumerate(_DIRECTIONS):
            next_row, next_col = row + int(round(dy)), col + int(round(dx))  # 그 방향으로 한 칸 이동한 칸
            if self.grid_map.in_bounds(next_row, next_col) and not self.grid_map.is_obstacle(next_row, next_col):
                valid[i] = True
        return valid

    def _mask_obstacles(self, target_pos: np.ndarray, weights: np.ndarray) -> np.ndarray:
        """벽으로 막힌 방향의 점수를 0으로 지운다."""
        valid = self._valid_mask(target_pos)
        return np.where(valid, weights, 0.0)  # 갈 수 있으면 원래 점수, 막혔으면 0

    def _top_k_routes(self, target_pos: np.ndarray, probabilities: np.ndarray) -> list[np.ndarray]:
        valid = self._valid_mask(target_pos)
        if valid.any():
            # 벽으로 막힌 방향은 애초에 후보에서 뺀다. 갈 수 있는 방향이
            # 원하는 개수(escape_route_top_k)보다 적으면, 억지로 막힌
            # 방향까지 채우지 않고 그냥 더 적게 돌려준다.
            candidate_indices = np.nonzero(valid)[0]
        else:
            # 사방이 다 막혀서 갈 수 있는 방향이 정말 하나도 없다. 이럴 땐
            # compute()에서 만든 "그래도 합은 1" 확률을 기준으로 8방향
            # 전부의 순위를 매긴다 — 이 경우엔 막힌 방향이 섞여도 어쩔 수
            # 없다, "좋은 선택지가 없다"는 뜻이니까.
            candidate_indices = np.arange(len(probabilities))
        order = candidate_indices[np.argsort(probabilities[candidate_indices])[::-1]]  # 확률 높은 순으로 정렬
        k = min(self.config.escape_route_top_k, len(order))  # 원하는 개수와 실제 후보 수 중 작은 쪽
        top_indices = order[:k]
        lookahead_m = self.grid_map.config.resolution_m * 3  # 화면에 짧게 보여줄 미리보기 길이(3칸)
        return [target_pos + _DIRECTIONS[i] * lookahead_m for i in top_indices]  # 각 방향으로 그만큼 나아간 점
