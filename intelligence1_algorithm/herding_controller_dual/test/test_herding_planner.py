# herding_controller_dual/test/test_herding_planner.py
import numpy as np

from herding_controller_dual.escape_model import EscapeEstimate
from herding_controller_dual.geodesic_field import GeodesicField
from herding_controller_dual.grid_map import GridConfig, GridMap
from herding_controller_dual.herding_planner import PlannerConfig, compute_blocking_point, compute_driving_point


def make_config():
    return PlannerConfig(
        drive_distance_m=0.8, panic_distance_m=0.35, alignment_threshold=0.7,
        drive_distance_ease_factor=1.3, block_lookahead_m=1.2,
    )


def test_driving_point_is_opposite_the_goal():
    config = make_config()
    target_pos = np.array([2.0, 2.0])
    goal_pos = np.array([5.0, 2.0])
    result = compute_driving_point(target_pos, np.zeros(2), goal_pos, np.array([1.0, 2.0]), config)
    # goal이 target의 +x 쪽에 있으므로, driving point는 -x 쪽에 있어야 한다
    assert result.point[0] < target_pos[0]
    assert result.is_panic is False


def test_panic_distance_triggers_retreat():
    config = make_config()
    target_pos = np.array([2.0, 2.0])
    robot_pos = np.array([2.1, 2.0])  # 0.1m 거리, panic_distance_m 이내
    result = compute_driving_point(target_pos, np.zeros(2), np.array([5.0, 2.0]), robot_pos, config)
    assert result.is_panic is True
    # 후퇴 지점은 로봇의 현재 위치보다 타겟으로부터 더 멀어야 한다
    assert np.linalg.norm(result.point - target_pos) > np.linalg.norm(robot_pos - target_pos)


def test_blocking_point_excludes_goal_hemisphere():
    config = make_config()
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    target_pos = np.array([5.0, 5.0])
    goal_pos = np.array([8.0, 5.0])  # goal은 target의 정확히 "E" 방향에 있음
    directions = np.array(
        [[0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]], dtype=float
    )
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    probabilities = np.zeros(8)
    probabilities[2] = 1.0  # "E"(goal 쪽)는 확률이 최대지만 제외되어야 함
    probabilities[6] = 0.5  # "W"(goal 반대쪽)가 허용되는 후보 중 최선
    estimate = EscapeEstimate(directions=directions, probabilities=probabilities, top_k_routes=[])
    point = compute_blocking_point(target_pos, goal_pos, estimate, grid, config)
    assert point[0] < target_pos[0]  # 선택된 경로가 goal 반대 방향(서쪽)을 향함


def test_blocking_point_falls_back_when_all_non_hemisphere_routes_blocked():
    config = make_config()
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    target_pos = np.array([5.0, 5.0])
    goal_pos = np.array([8.0, 5.0])  # goal은 target의 정확히 "E" 방향에 있음
    directions = np.array(
        [[0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]], dtype=float
    )
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    to_goal = goal_pos - target_pos
    to_goal = to_goal / np.linalg.norm(to_goal)
    dots = directions @ to_goal
    # hemisphere가 아닌 모든 방향(dots <= 0)을 장애물로 막아서, 메인 루프의
    # hemisphere 제한 탐색이 유효한 경로를 전혀 찾을 수 없도록 한다.
    for index, dot in enumerate(dots):
        if dot <= 0:
            point = target_pos + directions[index] * config.block_lookahead_m
            row, col = grid.world_to_cell(*point)
            grid.obstacle_mask[row, col] = True
    # "W"(인덱스 6, 막힘)가 확률 순위가 가장 높다; hemisphere에 속한 두 방향
    # ("NE", "E")은 막혀 있지 않으므로 fallback에서 고려되어야 한다.
    probabilities = np.array([0.1, 0.1, 0.5, 0.1, 0.05, 0.05, 0.15, 0.05])
    estimate = EscapeEstimate(directions=directions, probabilities=probabilities, top_k_routes=[])
    point = compute_blocking_point(target_pos, goal_pos, estimate, grid, config)
    # fallback은 (메인 루프에서 이미 장애물 셀 안에 있다고 알려진) 지점을 반환하는 대신
    # goal-hemisphere 제한을 완화해야 한다.
    row, col = grid.world_to_cell(*point)
    assert not grid.is_obstacle(row, col)


def test_blocking_point_stays_put_when_fully_boxed_in():
    config = make_config()
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    target_pos = np.array([5.0, 5.0])
    goal_pos = np.array([8.0, 5.0])
    directions = np.array(
        [[0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]], dtype=float
    )
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    # 8방향 전부를 막는다: 타겟에게 유효한 escape 경로가 전혀 없는 상황.
    for direction in directions:
        point = target_pos + direction * config.block_lookahead_m
        row, col = grid.world_to_cell(*point)
        grid.obstacle_mask[row, col] = True
    probabilities = np.full(8, 1.0 / 8.0)
    estimate = EscapeEstimate(directions=directions, probabilities=probabilities, top_k_routes=[])
    point = compute_blocking_point(target_pos, goal_pos, estimate, grid, config)
    # 유효한 방향이 없음: Blocker를 벽으로 보내는 대신 제자리에 머무른다.
    np.testing.assert_array_equal(point, target_pos)


def test_blocking_point_returns_previous_point_when_fully_boxed_in():
    config = make_config()
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    target_pos = np.array([5.0, 5.0])
    goal_pos = np.array([8.0, 5.0])
    directions = np.array(
        [[0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]], dtype=float
    )
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    for direction in directions:
        point = target_pos + direction * config.block_lookahead_m
        row, col = grid.world_to_cell(*point)
        grid.obstacle_mask[row, col] = True
    probabilities = np.full(8, 1.0 / 8.0)
    estimate = EscapeEstimate(directions=directions, probabilities=probabilities, top_k_routes=[])
    previous_point = np.array([4.0, 6.0])
    point = compute_blocking_point(
        target_pos, goal_pos, estimate, grid, config, previous_point=previous_point,
    )
    # 직전 커밋된 위치가 있으면, 표적 위치로 돌진하는 대신 그 자리를 지킨다.
    np.testing.assert_array_equal(point, previous_point)


def test_blocking_point_geodesic_filter_rejects_sealed_pocket():
    config = make_config()
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    target_pos = np.array([5.0, 5.0])
    goal_pos = np.array([8.0, 5.0])  # goal은 target의 정확히 "E" 방향에 있음
    directions = np.array(
        [[0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]], dtype=float
    )
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    probabilities = np.zeros(8)
    probabilities[6] = 0.9  # "W" -- 확률상 가장 유력하지만, 아래에서 밀폐된
    # 고립 셀로 만들 것이다 (그 지점 자체는 장애물이 아니므로 기존
    # Euclidean 검사만으로는 "뚫려 있다"고 오판한다).
    probabilities[7] = 0.1  # "NW" -- 차선책. 실제로 뚫려 있고 goal에서도 더 멂.

    w_point = target_pos + directions[6] * config.block_lookahead_m
    row, col = grid.world_to_cell(*w_point)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            grid.obstacle_mask[row + dr, col + dc] = True

    estimate = EscapeEstimate(directions=directions, probabilities=probabilities, top_k_routes=[])
    goal_row, goal_col = grid.world_to_cell(*goal_pos)
    geodesic_field = GeodesicField(grid, goal_row, goal_col)

    point = compute_blocking_point(
        target_pos, goal_pos, estimate, grid, config, geodesic_field=geodesic_field,
    )
    # "W"는 사방이 막힌 고립 셀이라 geodesic으로는 도달 불가능(거부)하고,
    # 대신 실제로 도달 가능하고 goal에서 더 먼 "NW"가 선택되어야 한다.
    expected = target_pos + directions[7] * config.block_lookahead_m
    np.testing.assert_allclose(point, expected)


def test_blocking_point_ignores_geodesic_filter_when_field_is_none():
    """geodesic_field를 안 넘기면(기본값 None) 기존 순수 Euclidean 동작과
    동일해야 한다 -- 하위호환 회귀 가드."""
    config = make_config()
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    target_pos = np.array([5.0, 5.0])
    goal_pos = np.array([8.0, 5.0])
    directions = np.array(
        [[0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]], dtype=float
    )
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    probabilities = np.zeros(8)
    probabilities[6] = 1.0  # "W"
    estimate = EscapeEstimate(directions=directions, probabilities=probabilities, top_k_routes=[])
    point = compute_blocking_point(target_pos, goal_pos, estimate, grid, config)
    expected = target_pos + directions[6] * config.block_lookahead_m
    np.testing.assert_allclose(point, expected)


# --------------------------------------------------------------------------- #
# 봉인 선분 (compute_sealing_pair) -- 2026-08-08                               #
# --------------------------------------------------------------------------- #
#
# 실측 상수 (test/real_map_arena.py와 같은 값): 로봇 반지름 0.171m,
# 벽 여유 0.03m, 표적(RC카) 폭 0.09m.
#   -> 두 로봇 중심 사이가 0.342~0.432m면 두 대로 봉인
#   -> 통로가 그보다 넓으면 봉인 불가(유도 모드로 넘어가야 함)

_R, _C, _W = 0.171, 0.03, 0.09


def _corridor_grid(width_m, resolution_m=0.05, length_m=4.0):
    """가로로 뻗은 폭 `width_m`짜리 직선 통로 하나짜리 격자."""
    from scipy import ndimage

    from herding_controller_dual.grid_map import GridConfig, GridMap

    w = int(round(length_m / resolution_m))
    h = int(round((width_m + 2.0) / resolution_m))
    mask = np.ones((h, w), dtype=bool)
    y0 = int(round(1.0 / resolution_m))
    y1 = y0 + int(round(width_m / resolution_m))
    mask[y0:y1, :] = False

    grid_map = GridMap(GridConfig(resolution_m=resolution_m, width_cells=w, height_cells=h,
                                  origin_x_m=0.0, origin_y_m=0.0))
    grid_map.obstacle_mask = mask
    clearance = ndimage.distance_transform_edt(~mask) * resolution_m
    return grid_map, clearance, (y0 + y1) / 2.0 * resolution_m


def _seal(width_m, back=0.3):
    from herding_controller_dual.herding_planner import compute_sealing_pair

    grid_map, clearance, cy = _corridor_grid(width_m)
    return compute_sealing_pair(
        np.array([2.0, cy]), goal_direction=np.array([1.0, 0.0]), grid_map=grid_map,
        clearance_m=clearance, robot_radius_m=_R, wall_clearance_m=_C,
        target_width_m=_W, back_distance_m=back,
    ), grid_map, clearance


def test_sealing_pair_seals_a_corridor_that_needs_both_robots():
    """두 로봇이 나란히 서야만 막히는 폭(0.85m)에서 봉인이 성립해야 한다."""
    pair, _, _ = _seal(0.85)
    assert pair.is_sealed, f"봉인 실패 (span={pair.span_m:.3f}, 폭={pair.corridor_width_m:.3f})"
    assert pair.requires_both, "두 대가 필요한 폭인데 한 대로 충분하다고 판정됨"
    assert 2 * _R - 1e-9 <= pair.span_m <= 2 * _R + _W + 1e-9


def test_sealing_pair_refuses_to_claim_a_seal_in_a_wide_room():
    """물리 봉인 한계보다 훨씬 넓은 공간은 봉인됐다고 주장하면 안 된다.

    이 검사가 없으면 두 로봇을 0.43m 벌려 세워놓고 "봉인 성공"이라 보고하면서
    양옆이 뻥 뚫린 걸 놓친다 -- 넓은 공간에서는 봉인이 불가능하다는 걸
    알고리즘이 스스로 알아야 유도 모드로 넘어갈 수 있다.
    """
    pair, _, _ = _seal(2.5)
    assert not pair.is_sealed, "폭 2.5m 공간을 봉인했다고 잘못 보고함"
    assert pair.corridor_width_m > 2.0


def test_sealing_pair_reports_single_robot_seal_without_claiming_cooperation():
    """로봇 두 대가 나란히 못 서는 아주 좁은 통로는 requires_both=False여야 한다.

    이 구간은 한 대로도 막히므로 "로봇 B가 기여했다"는 근거로 쓸 수 없다 --
    두 대를 같은 자리에 겹쳐 세워놓고 봉인 성공이라 보고하는 퇴화 케이스를
    막는 검사이기도 하다.
    """
    pair, _, _ = _seal(0.45)   # 로봇(지름 0.342) 하나는 들어가지만 둘은 못 나란히 섬
    assert not pair.requires_both
    np.testing.assert_allclose(pair.point_a, pair.point_b, atol=1e-9)


def test_sealing_pair_places_the_line_behind_the_target():
    """선분은 표적 뒤쪽(포획존 반대편)에 놓여야 밀어내는 방향이 맞다."""
    pair, _, _ = _seal(0.85)
    midpoint = (pair.point_a + pair.point_b) / 2.0
    assert midpoint[0] < 2.0, f"선분이 표적 앞에 놓임: {midpoint}"
    np.testing.assert_allclose(midpoint[0], 2.0 - 0.3, atol=1e-6)


def test_sealing_pair_endpoints_are_reachable_by_a_robot_body():
    """두 끝점 모두 로봇 몸체가 실제로 설 수 있는 자리여야 한다(벽 속이면 안 됨)."""
    pair, grid_map, clearance = _seal(0.85)
    for label, point in (("a", pair.point_a), ("b", pair.point_b)):
        row, col = grid_map.world_to_cell(*point)
        assert clearance[row, col] >= _R + _C - 1e-9, (
            f"끝점 {label} {point}: 벽까지 {clearance[row, col]:.3f}m < 필요 {_R + _C:.3f}m")


def test_sealing_pair_gap_between_robots_is_narrower_than_the_target():
    """봉인이라고 판정했다면, 두 로봇 몸통 사이 틈이 표적 폭보다 실제로 좁아야 한다."""
    for width in (0.75, 0.85, 0.95):
        pair, _, _ = _seal(width)
        if not pair.is_sealed:
            continue
        gap = pair.span_m - 2 * _R
        assert gap < _W, f"폭 {width}m: 봉인이라면서 틈이 {gap:.3f}m >= 표적 폭 {_W}m"


def test_sealing_pair_returns_unsealed_when_the_line_center_is_inside_a_wall():
    """선분 중심이 벽 안이면(로봇이 설 수 없으면) 봉인 불가로 나와야 한다."""
    from herding_controller_dual.herding_planner import compute_sealing_pair

    grid_map, clearance, cy = _corridor_grid(0.85)
    # 표적을 통로 끝 벽 쪽에 두고, 선분을 벽 바깥으로 물리게 한다.
    pair = compute_sealing_pair(
        np.array([0.1, cy]), goal_direction=np.array([1.0, 0.0]), grid_map=grid_map,
        clearance_m=clearance, robot_radius_m=_R, wall_clearance_m=_C,
        target_width_m=_W, back_distance_m=1.0,
    )
    assert not pair.is_sealed


# --------------------------------------------------------------------------- #
# 압박 선분 (compute_pressure_pair) -- 2026-08-08                              #
# --------------------------------------------------------------------------- #

_F = 0.42   # flee_reaction_distance_m


def _pressure(width_m, flee=_F, back=0.3):
    from herding_controller_dual.herding_planner import compute_pressure_pair

    grid_map, clearance, cy = _corridor_grid(width_m)
    return compute_pressure_pair(
        np.array([2.0, cy]), goal_direction=np.array([1.0, 0.0]), grid_map=grid_map,
        clearance_m=clearance, robot_radius_m=_R, wall_clearance_m=_C,
        flee_reaction_distance_m=flee, back_distance_m=back,
    ), grid_map, clearance


def test_pressure_pair_fully_covers_a_narrow_corridor():
    """두 로봇의 커버 폭 안쪽인 통로는 단면을 100% 덮어야 한다.

    로봇은 표적 중심 반지름 back(0.3m) 원 위 ±half_angle(기본 60도)에 놓이므로,
    진행방향 수직축 투영 좌표는 ±0.3*sin(60도)=±0.26. 각자 f=0.42를 덮으므로
    전체 커버 폭은 2*(0.26+0.42)=1.36m다.
    """
    pair, _, _ = _pressure(1.2)
    assert pair.coverage_fraction >= 0.999, f"커버율 {pair.coverage_fraction:.3f}"


def test_pressure_pair_cannot_cover_a_room_much_wider_than_four_flee_radii():
    """4f보다 훨씬 넓으면 100% 커버는 불가능해야 한다 -- 과대보고 방지."""
    pair, _, _ = _pressure(3.5)
    assert pair.coverage_fraction < 0.7, f"넓은 공간인데 커버율 {pair.coverage_fraction:.3f}"


def test_pressure_pair_spread_follows_the_half_angle_on_the_push_circle():
    """벽이 멀면 두 로봇 간격은 2*back*sin(half_angle)이어야 한다.

    처음엔 수직선 위에 ±f로 벌렸는데, 그러면 각 로봇이 표적에서
    sqrt(back^2+f^2)=0.52m 떨어져 반응거리(0.42m) 밖으로 나가 표적이 아예
    도주하지 않았다(5회 중 0회 포획). 원 위 배치는 두 로봇 모두 정확히
    back만큼 떨어뜨려 이 문제를 없앤다 -- 이 테스트가 그 회귀를 막는다.
    """
    back, half_angle = 0.3, np.pi / 3.0
    pair, _, _ = _pressure(3.5, back=back)
    np.testing.assert_allclose(pair.span_m, 2 * back * np.sin(half_angle), atol=0.06)
    assert not pair.wall_anchored
    # 두 로봇 모두 표적에서 back만큼 떨어져 있어야 도주 반응이 걸린다
    target = np.array([2.0, _corridor_grid(3.5)[2]])
    for point in (pair.point_a, pair.point_b):
        np.testing.assert_allclose(np.linalg.norm(point - target), back, atol=1e-6)


def test_pressure_pair_anchors_to_a_near_wall_and_shifts_coverage_outward():
    """한쪽 벽이 가까우면 그쪽은 벽에 붙이고 남은 폭을 반대쪽에 몰아줘야 한다 (C 전략).

    벽에 붙였는데도 반대쪽을 안 늘리면, 열린 쪽이 그만큼 뚫린 채로 남는다 --
    이 검사가 그 회귀를 막는다.
    """
    from herding_controller_dual.herding_planner import compute_pressure_pair

    grid_map, clearance, cy = _corridor_grid(1.0)
    # 표적을 통로 중앙이 아니라 한쪽 벽 가까이에 둔다 -> 한쪽 reach가 짧아진다
    target = np.array([2.0, cy + 0.30])
    pair = compute_pressure_pair(
        target, goal_direction=np.array([1.0, 0.0]), grid_map=grid_map,
        clearance_m=clearance, robot_radius_m=_R, wall_clearance_m=_C,
        flee_reaction_distance_m=_F, back_distance_m=0.3,
    )
    assert pair.wall_anchored, "벽이 가까운데 벽 활용이 발동하지 않음"
    # 합이 2f를 넘지 않아야 가운데가 안 뚫린다
    assert pair.span_m <= 2 * _F + 1e-6


def test_pressure_pair_never_places_a_robot_inside_a_wall():
    for width in (0.8, 1.0, 1.5, 2.5):
        pair, grid_map, clearance = _pressure(width)
        for label, point in (("a", pair.point_a), ("b", pair.point_b)):
            row, col = grid_map.world_to_cell(*point)
            assert clearance[row, col] >= _R + _C - 1e-9, (
                f"폭 {width}m 끝점 {label}: 벽까지 {clearance[row, col]:.3f}m")


def test_pressure_pair_coverage_grows_with_the_flee_radius():
    """쥐가 더 멀리서부터 피할수록 같은 공간에서 커버율이 높아져야 한다."""
    low, _, _ = _pressure(2.8, flee=0.42)
    high, _, _ = _pressure(2.8, flee=0.70)
    assert high.coverage_fraction > low.coverage_fraction + 0.1, (
        f"f=0.42 -> {low.coverage_fraction:.2f}, f=0.70 -> {high.coverage_fraction:.2f}")


def test_pressure_pair_two_robots_cover_more_than_one_would():
    """두 로봇의 커버 폭이 한 대(2f)보다 실제로 넓어야 한다 -- 협력의 기하학적 근거."""
    pair, _, _ = _pressure(2.8)
    single_cover = 2 * _F
    pair_cover = pair.coverage_fraction * pair.corridor_width_m
    assert pair_cover > single_cover * 1.5, (
        f"두 대 커버 {pair_cover:.2f}m vs 한 대 {single_cover:.2f}m -- 협력 이득이 없다")


# --------------------------------------------------------------------------- #
# 수비 지점 (compute_guard_point) -- 2026-08-08                                 #
# --------------------------------------------------------------------------- #

def test_guard_point_sits_on_the_escape_side_not_the_goal_side():
    """수비 지점은 표적 기준 **트랩 반대쪽**(도주 방향)에 있어야 한다.

    트랩 쪽에 서면 표적이 가야 할 길을 막는 셈이라 몰이가 안 된다 -- 실제로
    게이트를 트랩 가는 길에 놓았다가 성공률이 92%->15.8%로 무너진 적이 있다
    (트러블슈팅 노트 16-4).
    """
    from herding_controller_dual.herding_planner import compute_guard_point

    grid_map, clearance, cy = _corridor_grid(1.2, length_m=8.0)
    target = np.array([4.0, cy])
    goal_dir = np.array([1.0, 0.0])           # 트랩은 +x 쪽
    guard = compute_guard_point(target, goal_dir, grid_map, clearance, _R + _C)
    assert guard is not None
    assert guard.point[0] < target[0], f"수비 지점이 트랩 쪽에 있음: {guard.point}"


def test_guard_point_prefers_the_narrowest_chokepoint():
    """여러 후보 중 통로가 가장 좁은 길목을 골라야 막는 효과가 크다."""
    from scipy import ndimage

    from herding_controller_dual.grid_map import GridConfig, GridMap
    from herding_controller_dual.herding_planner import compute_guard_point

    res = 0.05
    w, h = int(8.0 / res), int(4.0 / res)
    mask = np.ones((h, w), dtype=bool)
    wide_lo, wide_hi = int(1.0 / res), int(3.0 / res)      # 폭 2.0m 통로
    mask[wide_lo:wide_hi, :] = False
    # x=2.0m 근처만 폭 0.8m로 좁힌다 (여기가 병목)
    narrow_lo, narrow_hi = int(1.6 / res), int(2.4 / res)
    pinch = slice(int(1.9 / res), int(2.1 / res))
    mask[wide_lo:narrow_lo, pinch] = True
    mask[narrow_hi:wide_hi, pinch] = True

    grid_map = GridMap(GridConfig(resolution_m=res, width_cells=w, height_cells=h,
                                  origin_x_m=0.0, origin_y_m=0.0))
    grid_map.obstacle_mask = mask
    clearance = ndimage.distance_transform_edt(~mask) * res

    target = np.array([3.5, 2.0])
    guard = compute_guard_point(target, np.array([1.0, 0.0]), grid_map, clearance, _R + _C,
                                min_ahead_m=0.6, max_ahead_m=2.5)
    assert guard is not None
    # 병목(x~2.0)을 골랐어야 한다 -- 넓은 구간(폭 2.0m)이 아니라
    assert guard.corridor_width_m < 1.2, f"넓은 곳을 골랐다: 폭 {guard.corridor_width_m:.2f}m"
    assert abs(guard.point[0] - 2.0) < 0.45, f"병목이 아닌 곳: {guard.point}"


def test_guard_point_returns_none_when_no_standable_spot_exists():
    """도주 방향에 로봇이 설 자리가 없으면 None을 돌려 폴백하게 해야 한다."""
    from herding_controller_dual.herding_planner import compute_guard_point

    grid_map, clearance, cy = _corridor_grid(1.2, length_m=8.0)
    # 표적을 통로 왼쪽 끝에 두고 도주 방향(-x)을 벽 밖으로 향하게 한다
    guard = compute_guard_point(np.array([0.2, cy]), np.array([1.0, 0.0]),
                                grid_map, clearance, _R + _C)
    assert guard is None
