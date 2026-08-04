# herding_controller/test/test_occlusion_grid.py
import numpy as np

from herding_controller.grid_map import GridConfig, GridMap
from herding_controller.occlusion_grid import OcclusionGrid, OcclusionGridConfig


def make_grid_and_belief():
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    belief = OcclusionGrid(OcclusionGridConfig(diffusion_rate=0.2, decay_factor=0.9), grid)
    return grid, belief


def test_seed_sets_best_guess_to_seeded_cell():
    _, belief = make_grid_and_belief()
    belief.seed(10, 10)
    assert belief.best_guess_cell() == (10, 10)


def test_step_diffuses_probability_to_neighbors():
    _, belief = make_grid_and_belief()
    belief.seed(10, 10)
    belief.step(dt=0.1)
    assert belief.belief[10, 11] > 0
    assert belief.belief[10, 9] > 0


def test_step_decays_total_probability_mass():
    _, belief = make_grid_and_belief()
    belief.seed(10, 10)
    total_before = belief.belief.sum()
    belief.step(dt=0.1)
    assert belief.belief.sum() < total_before


def test_obstacle_cells_never_hold_belief():
    grid, belief = make_grid_and_belief()
    grid.obstacle_mask[10, 11] = True
    belief.seed(10, 10)
    belief.step(dt=0.5)
    assert belief.belief[10, 11] == 0.0


def test_repeated_steps_decay_mass_geometrically_not_plateau():
    # Regression test: step() used to renormalize the diffused belief back to
    # a total of 1.0 before applying decay_factor, which erased every prior
    # step's decay and pinned the total mass at exactly decay_factor forever
    # instead of letting it fall off geometrically (decay_factor ** n) as
    # more steps elapse without observing the target.
    _, belief = make_grid_and_belief()
    belief.seed(10, 10)
    for _ in range(50):
        belief.step(dt=0.1)
    # decay_factor=0.9 for 50 steps -> 0.9**50 ~= 0.00515; a plateaued
    # implementation would instead sit at exactly 0.9 forever.
    assert belief.belief.sum() < 0.9 ** 10
    assert belief.belief.sum() >= 0.0
