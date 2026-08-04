# herding_controller/test/test_simulator.py
import numpy as np
import pytest

from herding_controller.herding_core import HerdingConfig, HerdingCore
from test.evasion_models.base import EvasionModel
from test.evasion_models.noisy_human import NoisyHuman
from test.evasion_models.random_walk import RandomWalk
from test.evasion_models.wall_hugger import WallHugger
from test.simulator import SimulatorConfig, _update_heading, run_trial


def make_herding_config(**overrides):
    """Build a HerdingConfig mirroring herding_params.yaml, with per-test overrides."""
    defaults = dict(
        frame_id="map", control_rate_hz=5.0,
        capture_zone_x_m=3.0, capture_zone_y_m=3.0, capture_radius_m=0.5, capture_hold_sec=1.0,
        grid_resolution_m=0.25, grid_width_cells=40, grid_height_cells=40,
        kf_process_noise=0.1, kf_measurement_noise=0.05, occlusion_timeout_sec=3.0,
        markov_wall_follow_p=0.70, markov_wall_hug_p=0.20, markov_center_p=0.10,
        momentum_weight=0.4, robot_repulsion_weight=1.5, wall_detect_radius_cells=1, escape_route_top_k=3,
        escape_concentration_threshold=0.5,
        drive_distance_m=0.8, flee_reaction_distance_m=1.0, panic_distance_m=0.35,
        alignment_threshold=0.7, drive_distance_ease_factor=1.3, block_lookahead_m=1.2,
        role_swap_margin=0.5, role_swap_cooldown_sec=2.0, min_robot_separation_m=0.6,
        role_cost_turn_weight=0.3, diffusion_rate=0.2, decay_factor=0.9,
    )
    defaults.update(overrides)
    return HerdingConfig(**defaults)


class ScriptedTarget(EvasionModel):
    """Teleports the target to a scripted position each cycle (needs a lifted speed cap)."""

    def __init__(self, positions_by_cycle, dt: float) -> None:
        self.positions_by_cycle = positions_by_cycle
        self.dt = dt
        self.calls = 0
        self.received_states = []

    def step(self, target_state, robot_positions, obstacle_map, dt):
        """Return the velocity that lands the target on this cycle's scripted position."""
        self.received_states.append(np.asarray(target_state, dtype=float).copy())
        desired = np.asarray(self.positions_by_cycle(self.calls), dtype=float)
        self.calls += 1
        return (desired - np.asarray(target_state[:2], dtype=float)) / self.dt


class ConstantVelocity(EvasionModel):
    """Commands a fixed velocity forever and records every state it was handed."""

    def __init__(self, velocity) -> None:
        self.velocity = np.asarray(velocity, dtype=float)
        self.received_states = []

    def step(self, target_state, robot_positions, obstacle_map, dt):
        """Record the incoming state and return the constant command velocity."""
        self.received_states.append(np.asarray(target_state, dtype=float).copy())
        return self.velocity.copy()


def test_run_trial_produces_bounded_trajectories():
    sim_config = SimulatorConfig(max_sim_time_sec=20.0)
    model = RandomWalk(max_speed_mps=0.4, rng=np.random.default_rng(1))
    result = run_trial(make_herding_config(), model, seed=1, sim_config=sim_config)
    assert result.duration_sec <= sim_config.max_sim_time_sec + 1e-6
    assert result.target_trajectory.shape[1] == 2
    assert result.robot1_trajectory.shape == result.target_trajectory.shape


def test_run_trial_is_deterministic_given_same_seed():
    sim_config = SimulatorConfig(max_sim_time_sec=10.0)
    model_a = RandomWalk(max_speed_mps=0.4, rng=np.random.default_rng(42))
    model_b = RandomWalk(max_speed_mps=0.4, rng=np.random.default_rng(42))
    result_a = run_trial(make_herding_config(), model_a, seed=42, sim_config=sim_config)
    result_b = run_trial(make_herding_config(), model_b, seed=42, sim_config=sim_config)
    assert np.allclose(result_a.target_trajectory, result_b.target_trajectory)


# --------------------------------------------------------------------------- #
# Heading update                                                              #
# --------------------------------------------------------------------------- #

def test_heading_update_depends_on_movement_not_on_resulting_x_sign():
    # A robot that moved is a robot whose heading changed, wherever it ended up.
    # Gating the update on the new x-coordinate froze the heading of every robot
    # operating at x <= 0, which is half the map whenever grid_origin_x_m < 0.
    heading = _update_heading(np.array([-3.0, 1.0]), np.array([-4.0, 1.0]), np.array([1.0, 0.0]))
    assert np.allclose(heading, [-1.0, 0.0])


def test_heading_is_held_when_the_robot_does_not_move():
    previous = np.array([0.0, 1.0])
    heading = _update_heading(np.array([2.0, 2.0]), np.array([2.0, 2.0]), previous)
    assert np.allclose(heading, previous)


def test_trial_spawns_inside_a_negative_origin_arena():
    config = make_herding_config(
        grid_origin_x_m=-5.0, grid_origin_y_m=-5.0, capture_zone_x_m=-2.0, capture_zone_y_m=-2.0,
    )
    result = run_trial(config, RandomWalk(0.4, np.random.default_rng(3)), seed=3,
                       sim_config=SimulatorConfig(max_sim_time_sec=10.0))
    for trajectory in (result.target_trajectory, result.robot1_trajectory, result.robot2_trajectory):
        assert (trajectory >= -5.0).all() and (trajectory <= 5.0).all()


# --------------------------------------------------------------------------- #
# Arena confinement                                                           #
# --------------------------------------------------------------------------- #

def test_target_is_confined_by_the_arena_walls():
    # The escape model treats every off-grid neighbour cell as a wall, so a target
    # that physically leaves the grid puts the simulation somewhere the algorithm
    # believes is impossible and where no escape distribution exists at all.
    config = make_herding_config()
    model = ConstantVelocity([10.0, 10.0])
    result = run_trial(config, model, seed=1, sim_config=SimulatorConfig(max_sim_time_sec=60.0),
                       control_mode="idle")
    assert (result.target_trajectory >= 0.0).all()
    assert (result.target_trajectory <= 10.0).all()
    # Once pinned to the corner the achieved velocity must read zero, not the
    # command that the wall refused: the next cycle's models all consume it.
    assert np.allclose(model.received_states[-1][2:], [0.0, 0.0])


def test_random_mode_keeps_robots_inside_the_arena():
    config = make_herding_config()
    result = run_trial(config, RandomWalk(0.4, np.random.default_rng(7)), seed=7,
                       sim_config=SimulatorConfig(max_sim_time_sec=60.0), control_mode="random")
    for trajectory in (result.robot1_trajectory, result.robot2_trajectory):
        assert (trajectory >= 0.0).all() and (trajectory <= 10.0).all()
    assert not np.allclose(result.robot1_trajectory[0], result.robot1_trajectory[-1])


def test_target_is_stopped_by_an_obstacle_cell():
    config = make_herding_config()
    mask = np.zeros((config.grid_height_cells, config.grid_width_cells), dtype=bool)
    mask[:, 24:] = True  # solid wall from x = 6.0 m rightwards
    result = run_trial(config, ConstantVelocity([0.4, 0.0]), seed=1,
                       sim_config=SimulatorConfig(max_sim_time_sec=30.0),
                       obstacle_mask=mask, control_mode="idle")
    assert result.target_trajectory[:, 0].max() < 6.0


def make_probe_grid_map(config):
    """The grid map a caller hands to WallHugger/NoisyHuman, exactly as Task 12 builds it."""
    return HerdingCore(config).grid_map


def target_path_length(result) -> float:
    """Total distance the target actually travelled over a trial."""
    return float(np.linalg.norm(np.diff(result.target_trajectory, axis=0), axis=1).sum())


# Seeds whose target spends time inside WallHugger's 3-cell wall-detection window.
# Measured total target path over 30 s: 0.16 / 0.56 / 1.72 m with an all-False mask,
# 6.76 / 6.84 / 5.98 m once the arena walls exist as obstacle cells.
@pytest.mark.parametrize("seed", [1, 8, 10])
def test_default_arena_walls_are_visible_to_a_wall_following_target(seed):
    # With an all-False obstacle mask WallHugger._nearest_wall_tangent() finds nothing
    # to hug and returns None, so the target stands still whenever no robot is inside
    # flee_reaction_distance_m -- a near-frozen target that would inflate every capture
    # rate, noisy_human's above all (it wraps WallHugger and is Task 12's real-world
    # predictor). The walls must exist as obstacle cells, not only as a clamp in the
    # physics, and the model must be reading THIS trial's arena.
    config = make_herding_config()
    model = WallHugger(0.4, config.flee_reaction_distance_m, make_probe_grid_map(config))
    result = run_trial(config, model, seed=seed, sim_config=SimulatorConfig(max_sim_time_sec=30.0))
    assert target_path_length(result) > 4.0


def test_noisy_human_inherits_the_walled_arena():
    # 4.38 m of target path before the fix, 8.31 m after.
    config = make_herding_config()
    model = NoisyHuman(0.4, config.flee_reaction_distance_m, make_probe_grid_map(config),
                       rng=np.random.default_rng(8))
    result = run_trial(config, model, seed=8, sim_config=SimulatorConfig(max_sim_time_sec=30.0))
    assert model._wall_hugger.grid_map.obstacle_mask[0, :].all()
    assert target_path_length(result) > 6.0


def test_default_obstacle_mask_is_a_boundary_ring():
    config = make_herding_config()
    model = WallHugger(0.4, config.flee_reaction_distance_m, make_probe_grid_map(config))
    run_trial(config, model, seed=2, sim_config=SimulatorConfig(max_sim_time_sec=1.0))
    mask = model.grid_map.obstacle_mask  # rebound to the trial's own arena
    assert mask.shape == (config.grid_height_cells, config.grid_width_cells)
    assert mask[0, :].all() and mask[-1, :].all() and mask[:, 0].all() and mask[:, -1].all()
    assert not mask[1:-1, 1:-1].any()


def test_explicit_obstacle_mask_is_used_verbatim():
    config = make_herding_config()
    supplied = np.zeros((config.grid_height_cells, config.grid_width_cells), dtype=bool)
    supplied[10, 10] = True
    model = WallHugger(0.4, config.flee_reaction_distance_m, make_probe_grid_map(config))
    run_trial(config, model, seed=2, sim_config=SimulatorConfig(max_sim_time_sec=1.0),
              obstacle_mask=supplied)
    assert np.array_equal(model.grid_map.obstacle_mask, supplied)


def test_robots_do_not_walk_into_obstacle_cells():
    # Robots and the target obey the same collision rule; a robot chasing a goal that
    # sits inside the wall ring must stop at the wall rather than tunnel into it.
    config = make_herding_config()
    result = run_trial(config, RandomWalk(0.4, np.random.default_rng(5)), seed=5,
                       sim_config=SimulatorConfig(max_sim_time_sec=60.0))
    ring = config.grid_resolution_m  # outermost cell ring is solid wall
    for trajectory in (result.robot1_trajectory, result.robot2_trajectory,
                       result.target_trajectory):
        assert (trajectory >= ring).all() and (trajectory <= 10.0 - ring).all()


def test_obstacle_mask_shape_is_validated():
    with pytest.raises(ValueError):
        run_trial(make_herding_config(), RandomWalk(0.4, np.random.default_rng(0)), seed=0,
                  sim_config=SimulatorConfig(max_sim_time_sec=1.0),
                  obstacle_mask=np.zeros((10, 10), dtype=bool))


# --------------------------------------------------------------------------- #
# Target state handed to the evasion model                                    #
# --------------------------------------------------------------------------- #

def test_evasion_model_receives_position_and_achieved_velocity():
    dt = 0.1
    velocity = np.array([0.2, 0.1])
    model = ConstantVelocity(velocity)
    run_trial(make_herding_config(), model, seed=1,
              sim_config=SimulatorConfig(max_sim_time_sec=2.0, dt=dt), control_mode="idle")
    assert all(state.shape == (4,) for state in model.received_states)
    assert np.allclose(model.received_states[0][2:], [0.0, 0.0])  # at rest before the first command
    for previous, current in zip(model.received_states, model.received_states[1:]):
        assert np.allclose(current[2:], velocity)
        assert np.allclose(current[:2], previous[:2] + velocity * dt)


# --------------------------------------------------------------------------- #
# Panic accounting                                                            #
# --------------------------------------------------------------------------- #

def test_panic_count_is_per_cycle_and_not_latched_once_triggered():
    # A running minimum compared against panic_distance_m keeps firing for every
    # remaining cycle of the trial once the target has been close even once,
    # turning panic_count into "cycles since the first violation".
    config = make_herding_config()
    dt = 0.1
    far, close = np.array([5.0, 5.0]), np.array([0.7, 0.5])  # 0.2 m from robot 1 at (0.5, 0.5)
    model = ScriptedTarget(lambda cycle: close if 3 <= cycle <= 5 else far, dt)
    result = run_trial(config, model, seed=1,
                       sim_config=SimulatorConfig(max_sim_time_sec=3.0, dt=dt,
                                                  target_max_speed_mps=1e3),
                       control_mode="idle")
    assert result.min_robot_target_dist == pytest.approx(0.2, abs=1e-6)
    assert 1 <= result.panic_count <= 6  # 30 cycles ran; only a handful were close


def test_panic_is_detected_when_it_happens_at_the_very_end_of_the_last_cycle():
    # Sampling only at the top of each cycle never measures the state the target
    # and robots actually finished in, so a violation created by the final move
    # is silently dropped from ALGO-003's panic accounting.
    config = make_herding_config()
    dt = 0.1
    far, close = np.array([5.0, 5.0]), np.array([0.7, 0.5])
    model = ScriptedTarget(lambda cycle: close if cycle == 9 else far, dt)
    result = run_trial(config, model, seed=1,
                       sim_config=SimulatorConfig(max_sim_time_sec=1.0, dt=dt,
                                                  target_max_speed_mps=1e3),
                       control_mode="idle")
    assert result.min_robot_target_dist == pytest.approx(0.2, abs=1e-6)
    assert result.panic_count == 1


# --------------------------------------------------------------------------- #
# Control modes and trial outcome reporting                                   #
# --------------------------------------------------------------------------- #

def test_idle_mode_holds_the_robots_but_still_advances_the_core():
    config = make_herding_config()
    result = run_trial(config, ConstantVelocity([0.0, 0.0]), seed=1,
                       sim_config=SimulatorConfig(max_sim_time_sec=5.0), control_mode="idle")
    assert np.allclose(result.robot1_trajectory, result.robot1_trajectory[0])
    assert np.allclose(result.robot2_trajectory, result.robot2_trajectory[0])
    assert result.mean_latency_ms > 0.0
    # A populated escape snapshot proves the FSM reached HERD and the escape model
    # ran, i.e. the core was stepped exactly as it is in 'algorithm' mode.
    assert result.escape_snapshot is not None
    assert result.escape_snapshot.shape[1] == 2


def test_unknown_control_mode_is_rejected():
    with pytest.raises(ValueError):
        run_trial(make_herding_config(), RandomWalk(0.4, np.random.default_rng(0)), seed=0,
                  sim_config=SimulatorConfig(max_sim_time_sec=1.0), control_mode="typo")


def test_trial_that_never_captures_reports_failure_at_the_full_duration():
    sim_config = SimulatorConfig(max_sim_time_sec=5.0)
    result = run_trial(make_herding_config(), ConstantVelocity([0.0, 0.0]), seed=1,
                       sim_config=sim_config, control_mode="idle")
    assert result.success is False
    assert result.duration_sec == pytest.approx(sim_config.max_sim_time_sec)
    assert len(result.target_trajectory) == 50


def test_capture_reports_the_elapsed_time_not_the_trial_limit():
    config = make_herding_config()  # capture_hold_sec = 1.0, capture zone (3, 3) r = 0.5
    sim_config = SimulatorConfig(max_sim_time_sec=30.0, target_max_speed_mps=1e3)
    capture_zone = np.array([config.capture_zone_x_m, config.capture_zone_y_m])
    result = run_trial(config, ScriptedTarget(lambda cycle: capture_zone, sim_config.dt), seed=1,
                       sim_config=sim_config, control_mode="idle")
    assert result.success is True
    assert result.duration_sec < 5.0
    assert result.duration_sec == pytest.approx(len(result.target_trajectory) * sim_config.dt)
