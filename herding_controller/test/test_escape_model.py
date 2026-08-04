# herding_controller/test/test_escape_model.py
import numpy as np

from herding_controller.escape_model import EscapeModel, EscapeModelConfig
from herding_controller.grid_map import GridConfig, GridMap


def make_model(grid=None):
    grid = grid or GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    config = EscapeModelConfig(
        wall_follow_p=0.70, wall_hug_p=0.20, center_p=0.10,
        momentum_weight=0.4, robot_repulsion_weight=1.5,
        wall_detect_radius_cells=1, escape_route_top_k=3,
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
    row, col = grid.world_to_cell(5.0, 5.25)  # the "N" neighbor cell
    grid.obstacle_mask[row, col] = True
    estimate = model.compute(target_pos, np.array([0.0, 0.0]), [np.array([2.0, 2.0])])
    north_index = 0  # directions[0] == N == (0, 1)
    assert estimate.probabilities[north_index] == 0.0


def test_top_k_routes_length_matches_config():
    model, _ = make_model()
    estimate = model.compute(np.array([5.0, 5.0]), np.array([0.0, 0.0]), [np.array([4.0, 5.0])])
    assert len(estimate.top_k_routes) == 3
