# herding_controller_dual/herding_controller_dual/herding_planner.py
"""Driving Point(Driver 목표점)와 Blocking Point(Blocker 목표점)를 계산한다."""
from dataclasses import dataclass

import numpy as np

from herding_controller_dual.escape_model import EscapeEstimate
from herding_controller_dual.grid_map import GridMap


@dataclass
class PlannerConfig:
    """Driver가 타겟을 얼마나 공격적으로 압박할지를 제어하는 임계값들."""
    drive_distance_m: float
    panic_distance_m: float
    alignment_threshold: float
    drive_distance_ease_factor: float
    block_lookahead_m: float


@dataclass
class DrivingResult:
    """Driver의 목표점과 panic-distance 후퇴 중인지 여부."""
    point: np.ndarray
    is_panic: bool


def compute_driving_point(
    target_pos: np.ndarray,
    target_vel: np.ndarray,
    goal_pos: np.ndarray,
    robot_pos: np.ndarray,
    config: PlannerConfig,
) -> DrivingResult:
    """미는 로봇(Driver)이 서야 할 자리를 알려준다: 쥐를 기준으로 덫의 정반대편.

    쥐는 다가오는 로봇을 보면 무서워서 반대로 도망친다(reactive flee).
    그래서 로봇이 "쥐 기준으로 덫과 정반대편"에 서 있으면, 쥐는 로봇을
    피해 자연스럽게 덫 쪽으로 도망치게 된다. 로봇이 쥐를 직접 붙잡거나
    떠미는 게 아니라, **쥐 자신의 도망 본능을 역이용해서 원하는 방향으로
    유도**하는 것이 이 알고리즘의 핵심 아이디어다.

    계산은 간단하다: "덫 → 쥐" 방향을 구해서, 그 방향으로 쥐에서
    `drive_distance_m`만큼 더 나아간 자리가 로봇의 목표점이다. 즉 쥐를
    사이에 두고 덫과 정확히 반대쪽에 서는 것이다.
    """
    to_target = target_pos - robot_pos   # 로봇 → 쥐 방향
    dist = np.linalg.norm(to_target)
    if dist < config.panic_distance_m:   # 로봇이 쥐한테 너무 가까이 붙어버렸다 — 정상 계산 대신 일단 물러나기
        # 너무 가까움(panic): 예를 들어 쥐가 갑자기 방향을 틀어서 로봇
        # 쪽으로 순간 다가온 경우다. 이럴 때도 평소처럼 "쥐 뒤쪽으로
        # 가라"고 하면 로봇이 쥐한테 더 다가가라는 명령이 되어, 쥐가
        # 너무 놀라서 어디로 튈지 예측할 수 없게 된다. 그래서 이럴 땐
        # 계산을 다 건너뛰고 "일단 쥐 반대쪽으로 한 걸음 물러나기"만
        # 시킨다 — 너무 가까워졌으니 거리부터 벌리자는 것.
        retreat_dir = -to_target / dist if dist > 1e-6 else np.array([1.0, 0.0])  # 쥐 반대쪽 방향
        retreat_point = robot_pos + retreat_dir * (config.panic_distance_m - dist)  # 딱 안전거리만큼 물러난 자리
        return DrivingResult(point=retreat_point, is_panic=True)

    u = target_pos - goal_pos   # 덫 → 쥐 방향
    norm = np.linalg.norm(u)
    u = u / norm if norm > 1e-6 else np.array([1.0, 0.0])  # 방향만 남기고 길이는 1로 (겹쳐 있으면 아무 방향)

    drive_distance = config.drive_distance_m
    to_goal = -u  # 쥐 → 덫 방향 (u의 정반대)
    speed = np.linalg.norm(target_vel)
    if speed > 1e-6:  # 쥐가 멈춰 있으면 "가는 방향"이 없어서 아래 판단을 할 수 없음 — 그냥 평소 거리 그대로
        # 살살 밀기(easing): 쥐가 이미 스스로 덫 쪽으로 가고 있다면
        # (가는 방향과 "쥐→덫" 방향이 많이 비슷하면), 로봇이 굳이 바짝
        # 붙어서 세게 밀 필요가 없다 — 오히려 너무 가까이 붙으면 쥐가
        # 놀라서 엉뚱한 데로 튈 수 있다. 그래서 이럴 땐 목표점을 쥐에서
        # 더 멀리 잡는다(거리를 ease_factor만큼 늘림) — "이미 잘 가고
        # 있으니 살살 압박만 하자"는 뜻이다.
        alignment = float(np.dot(target_vel / speed, to_goal))  # 쥐가 가는 방향과 "쥐→덫" 방향이 얼마나 비슷한지
        if alignment >= config.alignment_threshold:
            drive_distance *= config.drive_distance_ease_factor  # 이미 잘 가고 있음 — 덜 압박 (더 멀리서 지켜봄)

    return DrivingResult(point=target_pos + drive_distance * u, is_panic=False)  # 쥐를 기준으로 덫 반대편 자리


def _geodesic_distance_to_goal(position: np.ndarray, grid_map: GridMap, geodesic_field) -> float | None:
    """이 지점에서 벽을 피해 실제로 돌아가는 거리로 잰 "덫까지 거리". 계산 못 하면 None."""
    if geodesic_field is None:
        return None  # 이 기능을 아직 안 쓰는 경우 — 부르는 쪽이 알아서 다른 방법으로 처리
    try:
        row, col = grid_map.world_to_cell(*position)
    except ValueError:
        return None  # 지도 바깥
    distance = geodesic_field.distance[row, col]
    return float(distance) if np.isfinite(distance) else None  # 아예 못 가는 곳(무한대)이면 None


def _leads_away_from_goal(
    point: np.ndarray, target_pos: np.ndarray, grid_map: GridMap, geodesic_field,
) -> bool:
    """이 지점이 쥐보다 실제로(벽을 피해 돌아가도) 덫에서 더 먼 게 맞는지 확인한다.

    직선 거리만 보면 두 가지를 착각할 수 있다: (a) 벽 뒤 막다른 구석이라
    직선으로는 가까워 보여도 실제로 가려면 덫 쪽으로 크게 돌아가야 하는
    경우, (b) 얇은 벽 건너편이라 직선으로는 뚫려 보이지만 실제 길로는
    오히려 덫에 더 가까운 경우. 이 함수는 벽을 피해 돌아가는 실제 거리로
    다시 확인해서 이런 착각을 걸러낸다. 이 기능 자체를 안 쓰면(geodesic_field
    없음) 항상 통과시킨다.
    """
    if geodesic_field is None:
        return True  # 이 기능을 안 쓰는 경우 — 항상 통과
    point_dist = _geodesic_distance_to_goal(point, grid_map, geodesic_field)
    if point_dist is None:
        return False  # 이 지점엔 아예 갈 수가 없음 — 후보에서 제외
    target_dist = _geodesic_distance_to_goal(target_pos, grid_map, geodesic_field)
    if target_dist is None:
        return True  # 쥐 자신의 거리를 못 재서 비교 기준이 없음 — 그냥 통과시킴
    return point_dist > target_dist  # 쥐보다 실제로 더 멀어야 "진짜 도망로 후보"로 인정


def compute_blocking_point(
    target_pos: np.ndarray,
    goal_pos: np.ndarray,
    escape_estimate: EscapeEstimate,
    grid_map: GridMap,
    config: PlannerConfig,
    geodesic_field=None,
    previous_point: np.ndarray | None = None,
) -> np.ndarray:
    """막는 로봇(Blocker)이 서야 할 자리를 알려준다: 쥐가 도망갈 것 같은 다른 길목.

    마르코프 모델(`escape_model.py`)이 "쥐는 이 방향들로 도망갈 확률이
    높다"고 알려주면, 그중에서 아래 순서로 하나를 고른다:

    1. **덫 쪽 방향은 건너뛴다** — 그쪽은 미는 로봇(Driver)이 이미 맡고
       있으니 Blocker가 또 갈 필요가 없다.
    2. 남은 방향 중 확률이 제일 높은 것부터 순서대로 "여기가 진짜 갈 수
       있는 길인지" 확인한다 — 벽으로 막혀 있으면 건너뛴다(애초에 쥐가
       못 가는 길이니 지킬 필요 없음). geodesic_field가 있으면 한 번 더,
       "벽을 피해 돌아가도 정말 덫에서 더 먼 길인지" 확인한다 — 그냥
       직선거리만 보면 막다른 골목이나 얇은 벽 건너편(사실은 더 가까움)을
       "뚫린 길"로 착각할 수 있어서다.
    3. 이렇게 걸러낸 뒤 살아남은 첫 번째 후보(=덫 반대쪽에서 확률도 제일
       높고 실제로 갈 수 있는 길)를 Blocker의 목표점으로 정한다.
    4. 그런 길이 하나도 없으면(전부 막혔으면), "덫 쪽은 건너뛴다"는
       조건을 빼고 8방향 전체에서 다시 찾는다. 그마저도 없으면(사방이
       다 막힘) 바로 전에 정했던 자리를 그대로 유지하고, 그 자리도
       없으면(맨 처음이면) 그냥 쥐 위치로 대신한다.

    아래 코드의 세 덩어리(반복문 2개 + 마지막 처리)가 정확히 이 2/4-완화/4-완전차단
    단계에 해당한다.
    """
    to_goal = goal_pos - target_pos    # 쥐 → 덫 방향
    norm = np.linalg.norm(to_goal)
    to_goal = to_goal / norm if norm > 1e-6 else np.array([1.0, 0.0])  # 방향만 남기고 길이는 1로

    # dots[i] > 0 이면 그 방향 i는 덫이 있는 쪽이라는 뜻.
    dots = escape_estimate.directions @ to_goal            # 8방향 각각이 "덫 쪽"과 얼마나 비슷한지
    candidate_order = np.argsort(escape_estimate.probabilities)[::-1]  # 확률 높은 순서로 정렬

    for index in candidate_order:  # 확률 높은 방향부터 하나씩 확인
        if dots[index] > 0:
            continue  # 덫 쪽 방향 — Driver가 맡고 있으니 건너뜀 (위 설명 1번)
        direction = escape_estimate.directions[index]
        point = target_pos + direction * config.block_lookahead_m  # 그 방향으로 살짝 나아간 지점
        try:
            row, col = grid_map.world_to_cell(*point)
        except ValueError:
            continue  # 지도 밖 — 후보 탈락
        if grid_map.is_obstacle(row, col):
            continue  # 이미 벽으로 막힌 길 — 지킬 필요 없으니 다음 후보로 (위 설명 2번)
        if not _leads_away_from_goal(point, target_pos, grid_map, geodesic_field):
            continue  # 실제로 돌아가 보면 오히려 덫에 더 가까운 길이었음 — 후보 탈락
        return point  # 살아남은 첫 후보로 확정

    # 덫 반대쪽에서는 갈 수 있는 길을 못 찾았다: "덫 쪽은 건너뛴다"는
    # 조건을 빼고, 8방향 전체에서 확률 제일 높은 뚫린 길을 대신 찾는다.
    for index in candidate_order:  # 위와 똑같은 확인 과정, 다만 덫 쪽 방향도 후보에 포함
        direction = escape_estimate.directions[index]
        point = target_pos + direction * config.block_lookahead_m
        try:
            row, col = grid_map.world_to_cell(*point)
        except ValueError:
            continue
        if grid_map.is_obstacle(row, col):
            continue
        if not _leads_away_from_goal(point, target_pos, grid_map, geodesic_field):
            continue
        return point

    # 8방향 전부 막혔거나 갈 수 없는 곳뿐이다: 쥐가 완전히 갇혀서 지킬
    # 만한 길이 아예 없다는 뜻이다. 바로 전에 정했던 자리가 있으면 그
    # 자리를 계속 지키고(쥐 쪽으로 돌진하지 않고), 없으면(맨 처음 호출
    # 등) 그냥 쥐 위치를 대신 돌려준다.
    if previous_point is not None:
        return np.asarray(previous_point, dtype=float).copy()  # 이전 자리 유지
    return target_pos.copy()  # 이전 자리도 없음(맨 처음) — 일단 쥐 위치로


# --------------------------------------------------------------------------- #
# 봉인 선분 (Sealing Line) -- 2026-08-08                                       #
# --------------------------------------------------------------------------- #
#
# 배경: Driver/Blocker로 역할을 나눠 각자 따로 목표점을 계산하는 기존 방식은,
# 로봇 B가 성공률에 기여한다는 증거를 끝내 못 만들었다 (트러블슈팅 노트 11~14:
# N=450 페어드 비교에서 B가 결과를 바꾼 시행이 1건(0.2%)뿐). 근본 원인은
# "Blocker가 예측된 도주 방향 앞에 혼자 서 있기"라서, 없어도 표적이 대부분
# 같은 결과로 흘러가기 때문이다.
#
# 봉인 선분은 정반대 발상이다: 두 로봇을 **하나의 선분으로 묶어서** 통로를
# 몸으로 막는다. 통로 단면을 [벽|틈1|로봇A|틈2|로봇B|틈3|벽]으로 보고, 세 틈이
# 전부 표적 폭보다 좁으면 표적은 물리적으로 못 지나간다 -- 즉 한 대를 빼면
# 틈2가 벌어져 봉인이 깨지므로, 기여가 정의상 증명된다.
#
# 기하학 (r=로봇 반지름, c=벽 여유, w=표적 폭, S=두 로봇 중심 사이 거리):
#   틈2 = S - 2r  -> 막히려면 S <= 2r + w
#   틈1/틈3       -> 각 로봇을 벽 쪽으로 최대한 붙이면 c 가 되고, c < w 이면 막힘
#   로봇끼리 겹치지 않으려면 S >= 2r
# 실측값(r=0.171, c=0.03, w=0.09): 0.342 <= S <= 0.432 이면 두 대로 봉인.


@dataclass
class SealingPair:
    """봉인 선분의 두 끝점과 그 선분이 실제로 통로를 막고 있는지에 대한 판정."""
    point_a: np.ndarray          # 선분의 한쪽 끝 (진행방향 기준 왼쪽)
    point_b: np.ndarray          # 반대쪽 끝
    is_sealed: bool              # 표적이 이 선을 통과할 수 없는가
    requires_both: bool          # 봉인에 로봇 두 대가 실제로 필요한가
                                 # (False면 한 대로도 막히는 좁은 통로 -- 기여
                                 #  증명에는 쓸 수 없는 구간이다)
    span_m: float                # 두 끝점 사이 거리
    corridor_width_m: float      # 이 선분 위치에서 잰 통로 폭


def _free_extent_along(
    origin: np.ndarray, direction: np.ndarray, grid_map: GridMap,
    clearance_m, body_clearance_m: float, max_extent_m: float, step_m: float = 0.05,
) -> float:
    """origin에서 direction으로, 로봇 몸체가 들어갈 수 있는 마지막 지점까지의 거리."""
    extent = 0.0
    steps = int(max_extent_m / step_m)
    for i in range(1, steps + 1):
        probe = origin + direction * (i * step_m)  # direction으로 step_m씩 전진하며 탐침
        try:
            row, col = grid_map.world_to_cell(*probe)
        except ValueError:
            break  # 그리드 밖으로 나감 — 여기까지가 한계
        if not grid_map.in_bounds(row, col) or clearance_m[row, col] < body_clearance_m:
            break  # 로봇 몸체가 못 들어갈 만큼 벽에 가까워짐
        extent = i * step_m  # 마지막으로 성공한 지점까지의 거리를 갱신
    return extent


def _cell_clearance(position: np.ndarray, grid_map: GridMap, clearance_m) -> float:
    """position이 속한 셀에서 가장 가까운 벽까지의 거리(m). 격자 밖이면 0."""
    try:
        row, col = grid_map.world_to_cell(*position)
    except ValueError:
        return 0.0  # 그리드 밖 — "여유 없음"으로 취급(로봇이 못 서는 곳)
    if not grid_map.in_bounds(row, col):
        return 0.0
    return float(clearance_m[row, col])


def compute_sealing_pair(
    target_pos: np.ndarray,
    goal_direction: np.ndarray,
    grid_map: GridMap,
    clearance_m,
    robot_radius_m: float,
    wall_clearance_m: float,
    target_width_m: float,
    back_distance_m: float,
) -> SealingPair:
    """표적 뒤쪽을 가로지르는 봉인 선분의 두 끝점을 한 번에 계산한다.

    기존 compute_driving_point/compute_blocking_point가 두 로봇의 목표를 서로
    독립적으로 정하는 것과 달리, 이 함수는 **두 목표점을 하나의 기하학에서
    같이** 뽑는다 -- 그래서 한쪽이 빠지면 나머지 하나만으로는 선분이 성립하지
    않는다.

    `goal_direction`: 표적에서 포획존 쪽을 가리키는 단위벡터(벽을 고려한
        geodesic 방향을 넘기면 통로를 따라 밀게 된다).
    `back_distance_m`: 선분을 표적 뒤쪽(포획존 반대편)으로 얼마나 물릴지.

    반환값의 `is_sealed`가 False면 이 자리에서는 두 대로도 통로를 막을 수
    없다는 뜻이므로, 호출부는 기존 유도(Driver/Blocker) 방식으로 넘어가야
    한다. `requires_both`가 False면 한 대로도 막히는 좁은 구간이라, 로봇 B의
    기여를 증명하는 데는 쓸 수 없다.
    """
    direction = np.asarray(goal_direction, dtype=float)
    norm = float(np.linalg.norm(direction))
    direction = np.array([1.0, 0.0]) if norm < 1e-9 else direction / norm  # 단위벡터화(0벡터면 임의 방향)
    perp = np.array([-direction[1], direction[0]])  # direction에 수직인 방향 (90도 회전) — 선분이 뻗는 축

    body_clearance = robot_radius_m + wall_clearance_m  # 로봇이 서려면 필요한 벽까지의 최소 여유
    center = np.asarray(target_pos, dtype=float) - direction * back_distance_m  # 표적 뒤쪽(포획존 반대편)

    min_span = 2.0 * robot_radius_m                    # 두 로봇이 겹치지 않는 최소
    max_span = 2.0 * robot_radius_m + target_width_m   # 사이 틈이 표적보다 좁은 최대

    # 선분 중심에 로봇이 설 수조차 없으면(벽 안이거나 너무 좁음) 봉인 불가.
    if _cell_clearance(center, grid_map, clearance_m) < body_clearance:
        return SealingPair(point_a=center.copy(), point_b=center.copy(), is_sealed=False,
                           requires_both=False, span_m=0.0, corridor_width_m=0.0)

    probe_limit = max(max_span * 3.0, 1.5)  # 탐침 최대 거리 — 필요한 span보다 넉넉히
    reach_a = _free_extent_along(center, perp, grid_map, clearance_m, body_clearance, probe_limit)  # 한쪽으로 얼마나 뻗나
    reach_b = _free_extent_along(center, -perp, grid_map, clearance_m, body_clearance, probe_limit)  # 반대쪽
    # 통로 폭: 로봇 중심이 갈 수 있는 범위 + 양쪽 벽까지의 몸체+여유
    corridor_width = reach_a + reach_b + 2.0 * body_clearance

    # 두 로봇이 나란히 설 자리가 안 나오는 아주 좁은 통로: 한 대로 막히는지 본다.
    if reach_a + reach_b < min_span:
        single_gap = (corridor_width - 2.0 * robot_radius_m) / 2.0  # 로봇 한 대가 벽에 붙었을 때 남는 틈
        sealed_by_one = single_gap < target_width_m  # 그 틈이 표적보다 좁으면 한 대로도 막힘
        return SealingPair(point_a=center.copy(), point_b=center.copy(),
                           is_sealed=sealed_by_one, requires_both=False,
                           span_m=0.0, corridor_width_m=corridor_width)

    # 벽까지 최대한 뻗되, 사이 틈이 표적보다 넓어지지 않게 max_span으로 제한.
    half = max_span / 2.0
    offset_a = min(reach_a, half)  # 뻗을 수 있는 만큼과 절반 중 작은 쪽
    offset_b = min(reach_b, half)
    # 한쪽 벽이 가까워 덜 뻗었다면, 반대쪽을 그만큼 더 뻗어 max_span을 채운다
    # (선분을 통로 한쪽으로 치우쳐 붙이는 경우 -- 벽에 딱 붙는 쪽이 생긴다).
    slack = max_span - (offset_a + offset_b)  # 아직 못 채운 span
    if slack > 0:
        grow_a = min(slack, reach_a - offset_a)  # a쪽으로 더 뻗을 수 있는 만큼만 추가
        offset_a += grow_a
        slack -= grow_a
        offset_b += min(slack, reach_b - offset_b)  # 남은 slack을 b쪽으로

    point_a = center + perp * offset_a  # 선분의 한쪽 끝
    point_b = center - perp * offset_b  # 반대쪽 끝
    span = offset_a + offset_b          # 두 끝점 사이 거리

    gap_between = span - 2.0 * robot_radius_m               # 두 로봇 몸체 사이 실제 틈
    gap_wall_a = max(0.0, reach_a - offset_a) + wall_clearance_m  # a쪽 로봇과 벽 사이 틈
    gap_wall_b = max(0.0, reach_b - offset_b) + wall_clearance_m  # b쪽 로봇과 벽 사이 틈
    is_sealed = (
        span >= min_span - 1e-9        # 로봇 두 대가 겹치지 않을 만큼은 벌어져 있고
        and gap_between < target_width_m   # 로봇 사이 틈이 표적보다 좁고
        and gap_wall_a < target_width_m    # 양쪽 벽 틈도 전부 표적보다 좁아야
        and gap_wall_b < target_width_m    # 진짜로 통과 불가능(봉인 성공)
    )
    return SealingPair(point_a=point_a, point_b=point_b, is_sealed=is_sealed,
                       requires_both=True, span_m=span, corridor_width_m=corridor_width)


# --------------------------------------------------------------------------- #
# 압박 선분 (Pressure Pair) -- 2026-08-08                                      #
# --------------------------------------------------------------------------- #
#
# 봉인 선분(위)은 "두 로봇 몸통으로 통로를 물리적으로 막는다"였는데, 실측 결과
# 이 방에서는 쓸 데가 거의 없었다: 표적이 트랩으로 가는 경로 위에서 진행방향에
# 수직인 통로 폭이 중앙값 2.65m인데(5%ile로도 1.55m), 두 로봇이 몸으로 막을 수
# 있는 폭은 0.83m뿐이라 경로의 0.2%에서만 성립한다. 이 방은 생각보다 훨씬
# 열린 공간이다.
#
# 압박 선분은 조건을 완화한다: 몸으로 막는 대신 **표적이 로봇을 피하는 거리
# (flee_reaction_distance_m)**를 활용한다. 표적이 로봇 반경 f 안으로 안 들어온다면,
# 두 로봇을 2f만큼 벌려 세우면 그 사이 전체가 "가고 싶지 않은 영역"이 되고,
# 바깥쪽으로도 각각 f씩 더 커버되므로 최대 4f 폭을 덮는다.
#   f=0.42(현재) -> 1.68m -> 경로의 11%
#   f=0.70       -> 2.80m -> 경로의 55%
#
# 두 가지를 같이 쓴다:
#   (A) 좌우 협착: 표적 뒤쪽 진행방향 수직선상에 두 로봇을 대칭으로 배치
#   (C) 한쪽 벽 활용: 한쪽에 벽이 가까우면 그쪽 로봇을 벽에 붙이고, 남은
#       커버 폭을 반대쪽(열린 쪽) 로봇에게 몰아준다 -- 벽을 세 번째 로봇처럼
#       쓰는 셈이라, 열린 공간이 많은 이 방에 필요하다.


@dataclass
class PressurePair:
    """압박 선분의 두 끝점과, 그 배치가 표적의 진로를 얼마나 덮는지."""
    point_a: np.ndarray
    point_b: np.ndarray
    coverage_fraction: float   # 진행방향 수직 단면 중 "표적이 피하는 영역" 비율
    span_m: float
    corridor_width_m: float
    wall_anchored: bool        # 한쪽 끝을 벽에 붙였는가 (C 전략 발동 여부)


def compute_pressure_pair(
    target_pos: np.ndarray,
    goal_direction: np.ndarray,
    grid_map: GridMap,
    clearance_m,
    robot_radius_m: float,
    wall_clearance_m: float,
    flee_reaction_distance_m: float,
    back_distance_m: float,
    half_angle_rad: float = np.pi / 3.0,
) -> PressurePair:
    """두 로봇의 목표점을 하나의 기하학에서 같이 계산한다 (좌우 협착 + 벽 활용).

    기존 compute_driving_point/compute_blocking_point는 두 로봇 목표를 서로
    독립적으로 정해서, 한 대를 빼도 나머지 한 대의 행동이 그대로였다 -- 그래서
    로봇 B의 기여가 지표에 안 잡혔다(트러블슈팅 노트 11~14). 이 함수는 두
    점을 **같이** 뽑으므로, 한 대가 빠지면 그쪽 절반이 통째로 열린다.

    배치: 표적을 중심으로 반지름 `back_distance_m`인 원 위에, "목표의 반대편"
    방향을 기준으로 ±`half_angle_rad`만큼 벌린 두 지점.

    처음엔 두 로봇을 표적 뒤쪽 수직선 위에 ±flee만큼 벌려 세웠는데, 그러면
    각 로봇과 표적 사이 거리가 sqrt(back^2 + flee^2)로 flee를 넘어버려
    **표적이 아예 도주 반응을 안 했다**(실측: 5회 중 0회 포획, 전부 120초
    타임아웃). 원 위에 배치하면 두 로봇 모두 정확히 back_distance만큼
    떨어져 있어 둘 다 표적을 민다 -- 벌릴수록 커버는 넓어지지만 목표 방향
    성분(cos)이 줄어드는 트레이드오프를 `half_angle_rad`로 조절한다.

    벽이 가까워 한쪽 지점에 로봇이 설 수 없으면 그 각도를 안쪽으로 줄여
    (C 전략) 실제로 갈 수 있는 자리에 놓는다.
    """
    direction = np.asarray(goal_direction, dtype=float)
    norm = float(np.linalg.norm(direction))
    direction = np.array([1.0, 0.0]) if norm < 1e-9 else direction / norm
    perp = np.array([-direction[1], direction[0]])

    body_clearance = robot_radius_m + wall_clearance_m
    target = np.asarray(target_pos, dtype=float)
    behind = -direction                      # 목표 반대편 = 미는 방향
    radius = max(back_distance_m, 1e-6)

    def _place(sign, angle):
        """behind 방향에서 sign*angle 만큼 돌린 원 위의 점. 벽이면 각도를 줄인다."""
        for a in (angle, angle * 0.66, angle * 0.33, 0.0):  # 큰 각도부터 시도, 안 되면 점점 안쪽으로
            offset = behind * np.cos(a) + perp * (sign * np.sin(a))  # 원 위의 방향(behind에서 a만큼 회전)
            point = target + offset * radius
            if _cell_clearance(point, grid_map, clearance_m) >= body_clearance:
                return point, a, a < angle - 1e-9  # 세 번째 값 = 원래 각도보다 줄였는가(=벽에 걸려 클립됐는가)
        return target + behind * radius, 0.0, True  # 전부 실패 — 각도 0(정확히 behind 방향)으로 폴백

    point_a, angle_a, clipped_a = _place(+1.0, half_angle_rad)   # behind 기준 +쪽으로 벌린 지점
    point_b, angle_b, clipped_b = _place(-1.0, half_angle_rad)   # -쪽으로 벌린 지점
    wall_anchored = clipped_a or clipped_b   # 둘 중 하나라도 벽 때문에 각도가 줄었으면 True
    span = float(np.linalg.norm(point_a - point_b))

    # 커버율: 진행방향 수직축에 투영했을 때, 두 로봇의 반경 f 구간이 통로
    # 단면을 얼마나 덮는지. (원 배치라 두 로봇의 수직 좌표는 ±radius*sin(angle))
    f = flee_reaction_distance_m
    reach_a = _free_extent_along(target, perp, grid_map, clearance_m, body_clearance, max(2.0 * f, 1.5))  # perp쪽 통로 폭
    reach_b = _free_extent_along(target, -perp, grid_map, clearance_m, body_clearance, max(2.0 * f, 1.5))  # 반대쪽
    corridor_width = reach_a + reach_b + 2.0 * body_clearance
    lo, hi = -(reach_b + body_clearance), reach_a + body_clearance  # perp축 위의 통로 범위(표적 기준 상대좌표)
    width = max(hi - lo, 1e-9)   # 전체 통로 폭 (0 나눗셈 방지로 최소값)
    y_a = radius * np.sin(angle_a)   # 로봇 a의 perp축 좌표(원 위 배치이므로 sin으로 투영)
    y_b = -radius * np.sin(angle_b)  # 로봇 b는 반대쪽(-perp)이라 부호 반대
    # 각 로봇이 덮는 구간 [y-f, y+f]를 통로 범위로 자르고, 유효한(폭>0) 것만 남겨 왼쪽 끝 기준 정렬
    segs = sorted(
        s for s in ((max(lo, y_a - f), min(hi, y_a + f)), (max(lo, y_b - f), min(hi, y_b + f)))
        if s[1] > s[0]
    )
    # 구간 합집합(interval union) — 정렬된 구간을 훑으며 겹치면 병합, 안 겹치면 이전 구간 길이를 확정
    covered, cur_lo, cur_hi = 0.0, None, None
    for s_lo, s_hi in segs:
        if cur_hi is None or s_lo > cur_hi:  # 새 구간이 이전 구간과 안 겹침(또는 첫 구간)
            if cur_hi is not None:
                covered += cur_hi - cur_lo   # 직전 구간 길이를 합계에 확정
            cur_lo, cur_hi = s_lo, s_hi      # 새 구간 시작
        else:
            cur_hi = max(cur_hi, s_hi)       # 겹침 — 병합(오른쪽 끝만 늘어날 수 있음)
    if cur_hi is not None:
        covered += cur_hi - cur_lo   # 마지막 구간도 합계에 반영

    return PressurePair(
        point_a=point_a, point_b=point_b,
        coverage_fraction=float(min(1.0, covered / width)),
        span_m=span, corridor_width_m=corridor_width, wall_anchored=wall_anchored,
    )


# --------------------------------------------------------------------------- #
# 도주 분포 직접 최적화 (Escape Shaping) -- 2026-08-08                          #
# --------------------------------------------------------------------------- #
#
# 지금까지의 모든 시도(driving/blocking point, 봉인 선분, 압박 선분)는
# **기하학적 휴리스틱**이었다 -- "목표 반대편에 서라", "도주로 앞을 막아라",
# "±30도로 벌려라". 그리고 전부 로봇 B의 기여를 만들지 못했다(rescue 0~6/450).
#
# 정작 표적이 어디로 갈지 결정하는 건 escape_model의 마르코프 분포인데,
# 그걸 "예측"에만 쓰고 **입력(로봇 위치)을 그 분포에 유리하게 고르는 데는
# 쓴 적이 없었다.** 이 함수가 그걸 한다:
#
#   두 로봇을 어디 세우면 표적이 포획존 쪽으로 갈 확률이 최대가 되는가?
#
# escape_model의 로봇 반발 항은 두 로봇의 기여를 **합산**하므로, 두 대는
# 한 대보다 분포를 더 크게 밀 수 있다 -- 협력이 구조적으로 보장된다.
# Driver/Blocker 구분도 사라진다: 둘 다 "확률을 유리하게 만드는 자리"에 선다.


@dataclass
class ShapingPair:
    """도주 분포를 최적화한 두 로봇 목표점과 그때의 목표방향 확률."""
    point_a: np.ndarray
    point_b: np.ndarray
    goal_prob: float          # 표적이 포획존 쪽(반각 GOAL_CONE 이내)으로 갈 확률
    goal_prob_single: float   # 한 대만 썼을 때의 같은 확률 (기여 측정용)


# 목표 방향으로 간다고 볼 각도 폭(반각). 8방위 모델이라 45도면 목표 방향
# 인접 방향까지 포함된다 -- 너무 좁게 잡으면 정확히 한 방향만 인정해
# 최적화가 과도하게 뾰족해진다.
_GOAL_CONE_COS = np.cos(np.radians(67.5))


def _goal_directed_probability(estimate: EscapeEstimate, goal_direction: np.ndarray) -> float:
    """도주 확률 분포 중 '포획존 쪽' 방향들에 실린 확률의 합."""
    dots = estimate.directions @ goal_direction
    return float(estimate.probabilities[dots >= _GOAL_CONE_COS].sum())


def compute_shaping_pair(
    target_pos: np.ndarray,
    target_vel: np.ndarray,
    goal_direction: np.ndarray,
    escape_model,
    grid_map: GridMap,
    clearance_m,
    robot_radius_m: float,
    wall_clearance_m: float,
    stand_radius_m: float,
    n_angles: int = 16,
    rounds: int = 2,
) -> ShapingPair:
    """표적이 포획존 쪽으로 갈 확률을 최대화하는 두 로봇 위치를 고른다.

    표적 중심 반지름 `stand_radius_m` 원 위의 `n_angles`개 후보 중에서,
    두 로봇 위치 쌍을 좌표상승법(한쪽 고정하고 다른 쪽 최적화, `rounds`회
    반복)으로 고른다 -- 전체 쌍을 다 보면 O(n^2)라 제어 주기 안에 못 끝낸다.

    반환값의 `goal_prob_single`은 같은 조건에서 한 대만 썼을 때의 확률로,
    "두 번째 로봇이 분포를 얼마나 더 밀었는가"를 그대로 보여준다.
    """
    direction = np.asarray(goal_direction, dtype=float)
    norm = float(np.linalg.norm(direction))
    direction = np.array([1.0, 0.0]) if norm < 1e-9 else direction / norm
    target = np.asarray(target_pos, dtype=float)
    body_clearance = robot_radius_m + wall_clearance_m

    # 로봇이 실제로 설 수 있는 후보만 남긴다.
    candidates = []
    for a in np.linspace(0.0, 2.0 * np.pi, n_angles, endpoint=False):  # 원 둘레를 n_angles등분한 각도들
        point = target + stand_radius_m * np.array([np.cos(a), np.sin(a)])  # 그 각도의 원 위 지점
        if _cell_clearance(point, grid_map, clearance_m) >= body_clearance:
            candidates.append(point)
    if not candidates:
        behind = target - direction * stand_radius_m   # 후보가 하나도 없음 — 목표 반대편으로 폴백
        return ShapingPair(behind.copy(), behind.copy(), 0.0, 0.0)

    def score(points):
        """이 로봇 배치일 때 표적이 목표 방향으로 갈 확률."""
        est = escape_model.compute(target, target_vel, points)
        return _goal_directed_probability(est, direction)

    # 초기값: 목표 반대편(가장 그럴듯한 미는 자리)에 가장 가까운 후보 둘
    order = sorted(range(len(candidates)),
                   key=lambda i: -float(np.dot(target - candidates[i], direction)))  # 정렬 기준: behind 방향 정렬도(내림차순)
    idx_a = order[0]
    idx_b = order[1] if len(order) > 1 else order[0]

    for _ in range(rounds):  # 좌표상승법(coordinate ascent): 한쪽 고정하고 다른 쪽 최적화를 번갈아
        idx_a = max(range(len(candidates)),
                    key=lambda i: score([candidates[i], candidates[idx_b]]))  # b 고정, a를 최선으로
        idx_b = max(range(len(candidates)),
                    key=lambda i: score([candidates[idx_a], candidates[i]]))  # a 고정, b를 최선으로

    point_a, point_b = candidates[idx_a], candidates[idx_b]
    both = score([point_a, point_b])  # 두 대 다 썼을 때의 확률
    # 한 대만 쓸 때의 최선: 남은 한 대는 표적에서 아주 멀리 있다고 본다
    far = target + direction * 1e3  # 사실상 무한히 먼 더미 위치(기여를 0으로 만듦)
    single = max(score([candidates[i], far]) for i in range(len(candidates)))  # 최선의 "한 대만" 배치
    return ShapingPair(point_a=point_a, point_b=point_b,
                       goal_prob=both, goal_prob_single=single)


# --------------------------------------------------------------------------- #
# 수비-쓸기 (Guard & Sweep) -- 2026-08-08                                       #
# --------------------------------------------------------------------------- #
#
# lion-and-man 문헌의 2인 추격 전략을 몰이에 적용한 것. 지금까지 만든 모든
# 방식(Driver/Blocker, 봉인, 압박, 분포최적화)은 **두 로봇 다 표적을 쫓았다**.
# 그래서 표적보다 느린 로봇은 영원히 뒤따라가기만 했고, 기여가 안 나왔다.
#
# 이 전략은 구조가 다르다:
#   - **수비수(guard)**: 표적을 쫓지 않는다. 표적이 트랩 반대쪽으로 빠져나갈
#     때 반드시 지나야 하는 **가장 좁은 길목에 미리 가서 자리를 지킨다**.
#     먼저 가서 기다리므로 표적보다 느려도 상관없다.
#   - **쓸기(sweeper)**: 기존 compute_driving_point 그대로, 표적을 트랩 쪽으로 민다.
#
# 즉 수비수가 표적의 자유 영역을 잘라내고, 쓸기가 그 영역을 좁힌다.


@dataclass
class GuardPoint:
    """수비수가 지킬 길목과 그 길목의 폭(좁을수록 잘 막힌다)."""
    point: np.ndarray
    corridor_width_m: float
    distance_from_target_m: float


def compute_guard_point(
    target_pos: np.ndarray,
    goal_direction: np.ndarray,
    grid_map: GridMap,
    clearance_m,
    body_clearance_m: float,
    min_ahead_m: float = 0.6,
    max_ahead_m: float = 2.5,
    step_m: float = 0.1,
) -> GuardPoint | None:
    """표적이 트랩 반대쪽으로 도망칠 때 지나야 하는 가장 좁은 길목을 찾는다.

    표적에서 `goal_direction`의 **반대 방향**(=도주 방향)으로 나아가며, 각
    지점에서 진행방향에 수직인 통로 폭을 재고 가장 좁은 곳을 고른다. 그
    지점의 중앙이 수비 위치다 -- 표적이 그쪽으로 빠져나가려면 반드시 여길
    지나야 하고, 통로가 좁을수록 로봇 한 대로도 실질적으로 막힌다.

    범위 안에 로봇이 설 수 있는 지점이 하나도 없으면 None (그 경우 호출부는
    기존 Blocking Point 방식으로 폴백한다).
    """
    direction = np.asarray(goal_direction, dtype=float)
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        return None  # 방향이 정의 안 됨 — 계산 불가
    direction = direction / norm
    escape = -direction                      # 트랩 반대쪽 = 표적이 달아날 방향
    perp = np.array([-escape[1], escape[0]])  # escape에 수직인 방향 — 통로 폭을 재는 축
    target = np.asarray(target_pos, dtype=float)

    best = None
    steps = int((max_ahead_m - min_ahead_m) / step_m)
    for i in range(steps + 1):  # min_ahead_m부터 max_ahead_m까지 step_m 간격으로 훑음
        ahead = min_ahead_m + i * step_m
        probe = target + escape * ahead  # 표적에서 도주 방향으로 ahead만큼 나아간 지점(길목 후보)
        if _cell_clearance(probe, grid_map, clearance_m) < body_clearance_m:
            continue  # 로봇이 설 수 없는 곳(벽) — 이 후보는 건너뜀
        reach_a = _free_extent_along(probe, perp, grid_map, clearance_m, body_clearance_m, 3.0)   # 한쪽 벽까지
        reach_b = _free_extent_along(probe, -perp, grid_map, clearance_m, body_clearance_m, 3.0)  # 반대쪽 벽까지
        width = reach_a + reach_b + 2.0 * body_clearance_m  # 이 지점에서의 통로 폭
        # 통로 중앙에 서야 양쪽을 고르게 막는다.
        center = probe + perp * ((reach_a - reach_b) / 2.0)  # 좌우 여유가 다르면 중앙으로 보정
        if _cell_clearance(center, grid_map, clearance_m) < body_clearance_m:
            center = probe  # 보정한 중앙이 오히려 벽에 걸리면 원래 probe 지점 사용
        if best is None or width < best.corridor_width_m:  # 지금까지 중 가장 좁은 길목 갱신
            best = GuardPoint(point=center, corridor_width_m=width, distance_from_target_m=ahead)
    return best


# --------------------------------------------------------------------------- #
# 엔드게임 협공 (Endgame Pincer) -- 2026-08-09                                  #
# --------------------------------------------------------------------------- #
#
# 실패 원인 분석에서 나온 것: 구석 회피 표적의 실패 92건 중 **91건이 트랩
# 0.6m 안까지 접근**했고 최근접 거리 중앙값이 **0.37m**였다(포획 반경 0.3m).
# 즉 몰이는 이미 성공했고, **마지막 7cm를 못 넘는다**. 트랩이 벽 구석에
# 있어서 구석 회피 표적이 정확히 그 막다른 곳을 피해 옆으로 빠져나간다.
#
# 지금까지의 시도(압박/봉인/수비-쓸기)가 실패한 이유 중 하나는 **전 구간에
# 같은 배치를 적용**했기 때문이다. 이 모드는 표적이 트랩 근처에 들어온
# 순간에만 발동한다 -- 그 구간에서는 기하가 단순하고(트랩=벽 구석, 표적은
# 그 앞), 두 로봇이 트랩 입구 양옆을 막으면 표적이 갈 곳이 트랩뿐이 된다.


@dataclass  # "정보 여러 개를 한 상자에 묶어라"고 파이썬에게 시키는 것. 아래 3줄이 그 상자 안의 칸들
class PincerPair:
    """엔드게임에서 트랩 입구 양옆을 막는 두 지점."""
    point_a: np.ndarray  # np.ndarray = numpy(숫자 계산 라이브러리)의 배열 타입. 여기선 [x, y] 좌표 두 숫자를 담는 상자. 로봇1이 설 자리
    point_b: np.ndarray  # 위와 같은 타입. 로봇2가 설 자리
    active: bool  # bool = 참(True)/거짓(False) 둘 중 하나만 담는 타입. 협공이 실제로 성립했는지


def compute_endgame_pincer(
    target_pos: np.ndarray,  # 쥐 좌표 [x, y] — np.ndarray는 숫자 여러 개를 한 덩어리로 담는 타입(여기선 2개)
    trap_pos: np.ndarray,    # 덫 좌표 [x, y]. 타입은 target_pos와 같음
    grid_map: GridMap,       # GridMap = 이 프로젝트에서 만든 "방 지도" 전용 타입(클래스). 어디가 벽인지 담고 있음
    clearance_m,             # 타입을 안 적어둔 값. 각 칸이 벽에서 얼마나 떨어졌는지 담은 숫자 격자(2차원 배열)
    body_clearance_m: float, # float = 소수점이 있을 수 있는 숫자 하나(정수 아님). 예: 0.201
    stand_distance_m: float, # float — 로봇이 쥐로부터 떨어져 설 거리(m)
    trigger_radius_m: float, # float — 협공을 시작할 기준 거리(m)
    half_angle_rad: float = np.pi / 3.0,   # float에 "= 값"이 붙으면 기본값이란 뜻: 안 넘겨주면 이 값(60도, 라디안 단위)을 자동으로 씀. 60도 -- 실측 채택값
) -> PincerPair | None:  # "-> 타입"은 이 함수가 끝나고 돌려주는 값의 타입. PincerPair 상자 하나, 또는 None(아무것도 없음) 둘 중 하나
    """쥐가 덫 코앞까지 왔는데 안 잡히면, 두 로봇이 갈라져 설 좌우 두 자리를 알려준다.

    **이게 이 프로젝트의 핵심이다.** 그냥 미는 것만으로는 로봇 한 대로도
    충분해서, 로봇 B(원래 막는 역할)가 진짜로 쓸모 있다는 걸 보여줄 방법이
    없었다. 그런데 쥐가 덫 코앞(`trigger_radius_m` = 0.8m)까지 왔는데도
    3초 넘게 안 잡히면 — 아마 덫 바로 앞에서 좌우로 왔다갔다하며 버티고
    있는 것이다. 이때 두 로봇이 "쥐 기준으로 덫 반대편"에서 좌우로
    쫙 갈라져 서면(±60도), 쥐 입장에서는 왼쪽도 로봇, 오른쪽도 로봇이라
    도망갈 데가 덫 쪽밖에 안 남는다. 이 순간이 바로 로봇 B가 있어야만
    되는 순간이다.

    각도는 왜 60도냐면, 실제로 재봤더니(3곳 트랩 x 50번씩 = 150번) 이랬다:
      - ±60도로 벌리면 → 90.0% 성공
      - ±90도로 벌리면 → 46.0% 성공 (너무 벌어져서 그 사이로 쥐가 빠져나감)
      - ±120도로 벌리면 → 15.3% 성공 (거의 일렬로 서 있는 것과 비슷해짐)
    너무 좁게 벌리면 로봇 한 대나 다름없어지고, 너무 넓게 벌리면 둘 사이
    틈으로 쥐가 도망친다 — 60도가 딱 적당한 지점이다.
    (설정값 위치: `config/herding_params.yaml`의 `endgame_half_angle_deg: 60.0`)

    쥐가 아직 `trigger_radius_m`보다 멀리 있으면 협공을 안 하고 None을
    돌려준다 (그럴 땐 평소처럼 미는 로봇 + 막는 로봇 방식을 쓴다).
    """
    target = np.asarray(target_pos, dtype=float)  # 쥐 위치를 계산하기 좋은 숫자 형태로 (계산 준비운동, 의미 없음)
    trap = np.asarray(trap_pos, dtype=float)       # 덫 위치도 마찬가지
    to_trap = trap - target                          # 쥐 → 덫 방향. 좌표를 빼면 "그 방향을 가리키는 화살표"가 나온다
    dist = float(np.linalg.norm(to_trap))             # 그 화살표의 길이 = 쥐와 덫 사이 실제 거리(m)
    if dist > trigger_radius_m or dist < 1e-6:
        return None  # 아직 덫 근처가 아니거나(0.8m보다 멂) 이미 덫 위에 있음(거리 0) — 협공 안 함
    inward = to_trap / dist                           # 화살표를 길이로 나누면 "방향만" 남는다 (길이가 항상 1인 화살표)
    outward = -inward                       # 방향을 반대로 뒤집은 것. "덫 쪽"의 정반대 = 로봇이 서야 할 쪽
    perp = np.array([-outward[1], outward[0]])  # (x,y)->(-y,x)로 바꾸면 90도 돌아간다는 공식. 좌우로 벌어지는 축이 된다

    def place(sign):
        """"덫 반대편"에서 좌(sign=+1) 또는 우(sign=-1)로 벌어진 자리를 계산한다. 벽에 걸리면 덜 벌려서 다시 시도."""
        for angle in (half_angle_rad, half_angle_rad * 0.7, half_angle_rad * 0.4):  # 60도 안 되면 42도, 그래도 안 되면 24도
            # cos/sin은 "이 각도만큼 돌면 가로/세로로 얼마나 가는지" 알려주는 숫자(시계 바늘 방향 구하기와 같음).
            # outward 방향에서 시작해서 perp 쪽으로 angle만큼 기울인 새 방향을 만든다. sign이 +1/-1이면 좌우 어느 쪽으로 기울지 정해짐.
            offset = outward * np.cos(angle) + perp * (sign * np.sin(angle))
            point = target + offset * stand_distance_m  # 쥐 위치에서 그 방향으로 stand_distance_m(예 0.3m)만큼 간 자리 = 로봇이 실제로 설 좌표
            if _cell_clearance(point, grid_map, clearance_m) >= body_clearance_m:
                return point  # 그 자리가 벽에서 충분히 떨어져 있음(로봇 몸이 들어갈 공간 있음) — 여기로 확정
        return None  # 세 각도 다 벽에 걸림 — 이쪽엔 로봇이 설 자리가 없음

    point_a, point_b = place(+1.0), place(-1.0)   # 미니 계산기를 두 번 돌려서 왼쪽 자리, 오른쪽 자리를 각각 구함
    if point_a is None or point_b is None:
        return None  # 한쪽이라도 설 자리가 없으면 협공 불가 — 평소 방식으로
    # 두 자리 사이 거리를 재서, 로봇 두 대가 겹칠 만큼 가까우면(로봇끼리 겹치면) 협공이라고 할 수 없다.
    # -0.03은 컴퓨터 소수점 계산이 아주 살짝 오차 나는 것을 미리 봐주는 여유값.
    if float(np.linalg.norm(point_a - point_b)) < 2.0 * (body_clearance_m - 0.03):
        return None
    return PincerPair(point_a=point_a, point_b=point_b, active=True)  # 여기까지 왔으면 성공 — 두 좌표를 상자에 담아 반환
