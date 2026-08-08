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
