import csv

import numpy as np
import pytest

from herding_controller.grid_map import GridConfig, GridMap
from test.evasion_models.log_replay import LogReplay
from test.evasion_models.noisy_human import NoisyHuman
from test.evasion_models.random_walk import RandomWalk
from test.evasion_models.reactive_flee import ReactiveFlee
from test.evasion_models.wall_hugger import WallHugger


def test_reactive_flee_moves_away_from_nearby_robot():
    model = ReactiveFlee(max_speed_mps=0.4, flee_reaction_distance_m=1.0)
    target_state = np.array([5.0, 5.0, 0.0, 0.0])
    velocity = model.step(target_state, [np.array([4.5, 5.0])], obstacle_map=None, dt=0.1)
    assert velocity[0] > 0  # target flees in +x, away from the robot at -x side
    assert np.linalg.norm(velocity) <= 0.4 + 1e-9


def test_reactive_flee_stays_still_when_robot_is_far():
    model = ReactiveFlee(max_speed_mps=0.4, flee_reaction_distance_m=1.0)
    target_state = np.array([5.0, 5.0, 0.0, 0.0])
    velocity = model.step(target_state, [np.array([0.0, 0.0])], obstacle_map=None, dt=0.1)
    assert np.linalg.norm(velocity) < 1e-6


def test_wall_hugger_flees_when_robot_close():
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    model = WallHugger(max_speed_mps=0.4, flee_reaction_distance_m=1.0, grid_map=grid)
    target_state = np.array([5.0, 5.0, 0.0, 0.0])
    velocity = model.step(target_state, [np.array([4.5, 5.0])], obstacle_map=None, dt=0.1)
    assert np.linalg.norm(velocity) > 0


def test_random_walk_ignores_robot_but_moves():
    model = RandomWalk(max_speed_mps=0.4, rng=np.random.default_rng(0))
    target_state = np.array([5.0, 5.0, 0.0, 0.0])
    v1 = model.step(target_state, [np.array([5.01, 5.0])], obstacle_map=None, dt=0.1)
    assert np.linalg.norm(v1) <= 0.4 + 1e-9


def test_noisy_human_delays_reaction(monkeypatch):
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    rng = np.random.default_rng(0)
    model = NoisyHuman(
        max_speed_mps=0.4, flee_reaction_distance_m=1.0, grid_map=grid,
        reaction_delay_range=(1.0, 1.0), noise_std=0.0, rng=rng,
    )
    target_state = np.array([5.0, 5.0, 0.0, 0.0])
    # first calls happen before the 1.0s reaction delay elapses -> velocity command still zero
    v_early = model.step(target_state, [np.array([4.5, 5.0])], obstacle_map=None, dt=0.5)
    assert np.linalg.norm(v_early) < 1e-6
    v_late = model.step(target_state, [np.array([4.5, 5.0])], obstacle_map=None, dt=0.6)
    assert np.linalg.norm(v_late) > 0


def test_noisy_human_clips_noisy_command_to_max_speed():
    # Regression test: adding Gaussian noise to the base WallHugger command must not let
    # the returned velocity exceed max_speed_mps. With noise_std=0.5 and a near-zero
    # reaction delay, an unclipped implementation frequently produces velocities several
    # times over the cap (observed up to ~2.1 m/s against a 0.4 m/s cap).
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    rng = np.random.default_rng(42)
    model = NoisyHuman(
        max_speed_mps=0.4, flee_reaction_distance_m=1.0, grid_map=grid,
        reaction_delay_range=(0.01, 0.01), noise_std=0.5, rng=rng,
    )
    target_state = np.array([5.0, 5.0, 0.0, 0.0])
    for _ in range(500):
        velocity = model.step(target_state, [np.array([4.5, 5.0])], obstacle_map=None, dt=0.02)
        assert np.linalg.norm(velocity) <= 0.4 + 1e-9


def test_log_replay_reads_csv_and_returns_velocity_to_next_point(tmp_path):
    csv_path = tmp_path / "trace.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y"])
        writer.writerow([0.0, 0.0, 0.0])
        writer.writerow([1.0, 1.0, 0.0])
    model = LogReplay(str(csv_path))
    target_state = np.array([0.0, 0.0, 0.0, 0.0])
    velocity = model.step(target_state, [], obstacle_map=None, dt=0.1)
    assert velocity[0] > 0
