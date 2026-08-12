# herding_controller_dual/test/test_escape_model.py
import numpy as np

from herding_controller_dual.escape_model import EscapeModel, EscapeModelConfig
from herding_controller_dual.grid_map import GridConfig, GridMap


def make_model(grid=None, robot_repulsion_activation_distance_m=float("inf")):
    grid = grid or GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    config = EscapeModelConfig(
        wall_follow_p=0.70, wall_hug_p=0.20, center_p=0.10,
        momentum_weight=0.4, robot_repulsion_weight=1.5,
        wall_detect_radius_cells=1, escape_route_top_k=3,
        robot_repulsion_activation_distance_m=robot_repulsion_activation_distance_m,
    )
    return EscapeModel(config, grid), grid


def test_probabilities_sum_to_one():
    model, _ = make_model()
    estimate = model.compute(np.array([5.0, 5.0]), np.array([0.0, 0.0]), [np.array([4.0, 5.0])])
    assert np.isclose(estimate.probabilities.sum(), 1.0, atol=1e-6)


def test_obstacle_direction_is_masked_to_zero():
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    model, grid = make_model(grid)
    target_pos = np.array([5.0, 5.0])
    row, col = grid.world_to_cell(5.0, 5.25)  # "N" 인접 셀
    grid.obstacle_mask[row, col] = True
    estimate = model.compute(target_pos, np.array([0.0, 0.0]), [np.array([2.0, 2.0])])
    north_index = 0  # directions[0] == N == (0, 1)
    assert estimate.probabilities[north_index] == 0.0


def test_top_k_routes_length_matches_config():
    model, _ = make_model()
    estimate = model.compute(np.array([5.0, 5.0]), np.array([0.0, 0.0]), [np.array([4.0, 5.0])])
    assert len(estimate.top_k_routes) == 3


def test_fully_boxed_in_falls_back_to_uniform_distribution():
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    model, grid = make_model(grid)
    target_pos = np.array([5.0, 5.0])
    row, col = grid.world_to_cell(*target_pos)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            grid.obstacle_mask[row + dr, col + dc] = True
    estimate = model.compute(target_pos, np.array([0.0, 0.0]), [np.array([2.0, 2.0])])
    assert np.isclose(estimate.probabilities.sum(), 1.0, atol=1e-6)


def test_top_k_routes_excludes_obstacle_cells_when_some_directions_invalid():
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    model, grid = make_model(grid)
    target_pos = np.array([5.0, 5.0])
    row, col = grid.world_to_cell(*target_pos)
    # N, NE, E, SE, SW를 차단 (8방향 중 5개); S, W, NW만 유효하게 남음.
    blocked_dirs = [(0, 1), (1, 1), (1, 0), (1, -1), (-1, -1)]
    for dx, dy in blocked_dirs:
        grid.obstacle_mask[row + dy, col + dx] = True
    estimate = model.compute(target_pos, np.array([0.0, 0.0]), [np.array([2.0, 2.0])])
    assert len(estimate.top_k_routes) == 3
    for route in estimate.top_k_routes:
        r, c = grid.world_to_cell(*route)
        assert not grid.is_obstacle(r, c)


def test_robot_repulsion_gates_second_robot_beyond_activation_distance():
    """Blocker(robot_positions의 두 번째 원소)가 활성화 거리보다 멀면 기여가 0이어야 한다."""
    model, _ = make_model(robot_repulsion_activation_distance_m=1.0)
    target_pos = np.array([5.0, 5.0])
    driver_pos = np.array([4.5, 5.0])   # 0.5m -- 인덱스 0(Driver)은 게이팅 안 받음
    blocker_far = np.array([5.0, 8.0])  # 3.0m > 1.0m 활성화 거리

    with_far_blocker = model._robot_repulsion(target_pos, [driver_pos, blocker_far])
    driver_only = model._robot_repulsion(target_pos, [driver_pos])
    assert np.allclose(with_far_blocker, driver_only)


def test_robot_repulsion_includes_second_robot_within_activation_distance():
    """Blocker가 활성화 거리 안이면 게이팅 없을 때와 동일하게 기여해야 한다."""
    gated_model, _ = make_model(robot_repulsion_activation_distance_m=1.0)
    ungated_model, _ = make_model(robot_repulsion_activation_distance_m=float("inf"))
    target_pos = np.array([5.0, 5.0])
    driver_pos = np.array([4.5, 5.0])
    blocker_near = np.array([5.0, 5.5])  # 0.5m < 1.0m 활성화 거리

    gated = gated_model._robot_repulsion(target_pos, [driver_pos, blocker_near])
    ungated = ungated_model._robot_repulsion(target_pos, [driver_pos, blocker_near])
    assert np.allclose(gated, ungated)


def test_robot_repulsion_default_activation_distance_does_not_gate():
    """기본값(inf)에서는 아주 먼 두 번째 로봇도 여전히(약하게) 기여해야 한다 -- 하위호환 확인."""
    model, _ = make_model()  # 기본값 = inf
    target_pos = np.array([5.0, 5.0])
    driver_pos = np.array([4.5, 5.0])
    blocker_far = np.array([5.0, 20.0])  # 15m

    with_blocker = model._robot_repulsion(target_pos, [driver_pos, blocker_far])
    driver_only = model._robot_repulsion(target_pos, [driver_pos])
    assert not np.allclose(with_blocker, driver_only)


def test_robot_repulsion_gates_whichever_index_is_passed_as_blocker():
    """플랜 B는 역할이 매 주기 바뀔 수 있어, 게이팅 대상이 하드코딩된 인덱스 1이
    아니라 호출부가 넘긴 blocker_index를 따라야 한다 -- 로봇 0이 Blocker인
    주기에는 로봇 0이 게이팅되고 로봇 1(지금은 Driver)은 게이팅되지 않아야
    한다(플랜 A의 고정 배정과 정반대)."""
    model, _ = make_model(robot_repulsion_activation_distance_m=1.0)
    target_pos = np.array([5.0, 5.0])
    robot0_far = np.array([5.0, 8.0])   # 3.0m > 1.0m
    robot1_near = np.array([4.5, 5.0])  # 0.5m < 1.0m

    # blocker_index=0(로봇 0이 지금 Blocker)이면, 로봇 0이 멀어도 게이팅되어
    # 로봇 1(지금 Driver)만 기여해야 한다.
    gated_as_blocker0 = model._robot_repulsion(target_pos, [robot0_far, robot1_near], blocker_index=0)
    driver1_only = model._robot_repulsion(target_pos, [robot1_near])
    assert np.allclose(gated_as_blocker0, driver1_only)

    # blocker_index=1(로봇 1이 지금 Blocker, 플랜 A와 동일 규약)이면, 로봇
    # 0(지금 Driver, 3.0m 밖이지만 게이팅 대상이 아님)이 여전히 기여해야
    # 하므로 위 결과와 달라야 한다.
    gated_as_blocker1 = model._robot_repulsion(target_pos, [robot0_far, robot1_near], blocker_index=1)
    assert not np.allclose(gated_as_blocker1, gated_as_blocker0)
