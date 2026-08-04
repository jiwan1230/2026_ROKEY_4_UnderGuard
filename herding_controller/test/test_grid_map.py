import numpy as np
import pytest

from herding_controller.grid_map import GridConfig, GridMap


def make_grid():
    return GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))


def test_world_to_cell_round_trip():
    grid = make_grid()
    x, y = 3.1, 2.6
    row, col = grid.world_to_cell(x, y)
    back_x, back_y = grid.cell_to_world(row, col)
    assert abs(back_x - x) <= grid.config.resolution_m
    assert abs(back_y - y) <= grid.config.resolution_m


def test_world_to_cell_out_of_bounds_raises():
    grid = make_grid()
    with pytest.raises(ValueError):
        grid.world_to_cell(-5.0, -5.0)


def test_obstacle_mask_from_occupancy():
    grid = make_grid()
    occ = np.zeros((40, 40), dtype=int)
    occ[5, 5] = 100
    grid.set_obstacle_mask_from_occupancy(occ, threshold=50)
    assert grid.is_obstacle(5, 5) is True
    assert grid.is_obstacle(0, 0) is False


def test_world_to_cell_negative_near_origin_raises():
    """Regression test: small negative offsets from origin should raise ValueError."""
    grid = make_grid()
    # With resolution_m=0.25, origin=(0,0), coordinates like -0.1 have
    # offset -0.1 from origin, which floors to cell -1 (out-of-bounds).
    # Previously, int(-0.1/0.25) = int(-0.4) = 0 (wrong truncation direction).
    with pytest.raises(ValueError):
        grid.world_to_cell(-0.1, -0.1)
