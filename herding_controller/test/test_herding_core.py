# herding_controller/test/test_herding_core.py
import numpy as np

from herding_controller.herding_core import HerdingConfig, HerdingCore, Observation
from herding_controller.state_machine import FSMState


def make_config():
    return HerdingConfig(
        frame_id="map", control_rate_hz=5.0,
        capture_zone_x_m=3.0, capture_zone_y_m=3.0, capture_radius_m=0.5, capture_hold_sec=0.4,
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


def test_no_rclpy_import_anywhere_in_core_chain():
    import herding_controller.herding_core as core_module
    with open(core_module.__file__) as f:
        assert "import rclpy" not in f.read()


def test_step_returns_search_state_with_no_observation():
    core = HerdingCore(make_config())
    obs = Observation(
        target_measurement=None, robot1_pos=np.array([0.0, 0.0]), robot2_pos=np.array([1.0, 0.0]),
        robot1_heading=np.array([1.0, 0.0]), robot2_heading=np.array([1.0, 0.0]),
        occupancy=None, sim_time_sec=0.0, dt=0.2,
    )
    output = core.step(obs)
    assert output.fsm_state == FSMState.SEARCH


def test_step_tracks_and_drives_toward_target_after_observations():
    core = HerdingCore(make_config())
    t = 0.0
    output = None
    for _ in range(20):
        obs = Observation(
            target_measurement=np.array([2.0, 2.0]), robot1_pos=np.array([0.0, 0.0]),
            robot2_pos=np.array([4.0, 4.0]), robot1_heading=np.array([1.0, 0.0]),
            robot2_heading=np.array([1.0, 0.0]), occupancy=None, sim_time_sec=t, dt=0.2,
        )
        output = core.step(obs)
        t += 0.2
    assert output.fsm_state in (FSMState.HERD, FSMState.CORNER, FSMState.CAPTURED)
    assert output.driver_id in (1, 2)
    assert output.blocker_id in (1, 2)
    assert output.driver_id != output.blocker_id


# --------------------------------------------------------------------------- #
# Integration edge cases                                                       #
# --------------------------------------------------------------------------- #


class Runner:
    """Drives a HerdingCore through cycles with a monotonically advancing clock."""

    def __init__(self, config=None, dt=0.2):
        self.core = HerdingCore(config or make_config())
        self.dt = dt
        self.t = 0.0

    def run(self, n, measurement, r1=(0.0, 0.0), r2=(4.0, 4.0), occupancy=None):
        output = None
        for _ in range(n):
            output = self.core.step(Observation(
                target_measurement=None if measurement is None else np.array(measurement, dtype=float),
                robot1_pos=np.array(r1, dtype=float), robot2_pos=np.array(r2, dtype=float),
                robot1_heading=np.array([1.0, 0.0]), robot2_heading=np.array([1.0, 0.0]),
                occupancy=occupancy, sim_time_sec=self.t, dt=self.dt,
            ))
            self.t += self.dt
        return output

    def run_until(self, state, measurement, limit=40, **kwargs):
        for _ in range(limit):
            output = self.run(1, measurement, **kwargs)
            if output.fsm_state == state:
                return output
        raise AssertionError(f"never reached {state}")


def test_config_wires_every_subconfig_without_type_error():
    core = HerdingCore(make_config())
    assert core.grid_map.config.resolution_m == 0.25
    assert core.grid_map.config.width_cells == 40
    assert core.estimator.config.process_noise == 0.1
    assert core.estimator.config.occlusion_timeout_sec == 3.0
    assert core.escape_model.config.wall_follow_p == 0.70
    assert core.escape_model.config.escape_route_top_k == 3
    assert core.planner_config.drive_distance_m == 0.8
    assert core.planner_config.block_lookahead_m == 1.2
    assert core.role_assigner_config.role_cost_turn_weight == 0.3
    assert core.role_assigner_config.min_robot_separation_m == 0.6
    assert core.occlusion_grid.config.diffusion_rate == 0.2
    assert core.occlusion_grid.config.decay_factor == 0.9
    # The escape model and the occlusion grid must share the core's single GridMap.
    assert core.escape_model.grid_map is core.grid_map
    assert core.occlusion_grid.grid_map is core.grid_map


def test_escape_concentration_threshold_gates_the_corner_transition():
    """The threshold must come from config, not a hardcoded literal, and must actually gate."""
    import dataclasses
    # Target parked inside the capture zone, with a hold long enough that CORNER is observable.
    base = dataclasses.replace(make_config(), capture_hold_sec=1e6)

    never = Runner(dataclasses.replace(base, escape_concentration_threshold=1.1))
    assert never.run(10, (3.1, 3.0)).fsm_state == FSMState.HERD

    always = Runner(dataclasses.replace(base, escape_concentration_threshold=0.0))
    assert always.run(10, (3.1, 3.0)).fsm_state == FSMState.CORNER


def test_search_state_skips_escape_planner_and_role_logic():
    """Before any observation the KF state is all-zero; nothing downstream may run on it."""
    runner = Runner()
    output = runner.run(5, None)
    assert output.fsm_state == FSMState.SEARCH
    assert output.escape_top3 == []
    assert output.panic is False
    assert output.role_swapped is False
    # Robots hold position, and the estimator was never advanced.
    np.testing.assert_allclose(output.robot1_goal, [0.0, 0.0])
    np.testing.assert_allclose(output.robot2_goal, [4.0, 4.0])
    assert runner.core.estimator.get_state().time_since_observation == 0.0


def test_lost_state_seeds_occlusion_grid_and_skips_escape_planner_role():
    runner = Runner()
    assert runner.run(5, (2.0, 2.0)).fsm_state == FSMState.HERD
    output = runner.run_until(FSMState.LOST, None)

    assert runner.core._occlusion_seeded is True
    assert runner.core._last_known_cell == runner.core.grid_map.world_to_cell(2.0, 2.0)
    assert runner.core.occlusion_grid.belief.sum() > 0.0
    # Occlusion grid is used ONLY in LOST: no escape distribution, no panic, no swap.
    assert output.escape_top3 == []
    assert output.panic is False
    assert output.role_swapped is False
    # Both robots search near the last known target position, but not the same point.
    np.testing.assert_allclose(output.robot1_goal, [2.125, 2.125], atol=0.3)
    separation = float(np.linalg.norm(output.robot1_goal - output.robot2_goal))
    assert separation >= runner.core.config.min_robot_separation_m - 1e-6


def test_second_lost_episode_reseeds_from_new_position():
    runner = Runner()
    runner.run(5, (2.0, 2.0))
    runner.run_until(FSMState.LOST, None)
    first_cell = runner.core._last_known_cell

    # Recovering must clear the seed state so the next episode starts fresh.
    recovered = runner.run(1, (2.0, 2.0))
    assert recovered.fsm_state == FSMState.TRACK
    assert runner.core._occlusion_seeded is False
    assert runner.core._last_known_cell is None

    runner.run(12, (5.0, 5.0))
    runner.run_until(FSMState.LOST, None)
    second_cell = runner.core._last_known_cell
    assert second_cell is not None
    assert second_cell != first_cell
    # Seeded near the new location, not the stale one.
    assert abs(second_cell[0] - 20) <= 6 and abs(second_cell[1] - 20) <= 6


def test_role_swapped_is_false_on_the_bootstrap_assignment():
    """The first assign() picks the cost-optimal driver outright; that is not a swap."""
    runner = Runner()
    output = runner.run_until(FSMState.HERD, (2.0, 2.0), r1=(8.0, 8.0), r2=(1.5, 1.5))
    assert output.driver_id == 2  # robot 2 is far cheaper, so the bootstrap picks it
    assert output.role_swapped is False


def test_role_swapped_is_true_only_on_the_cycle_of_a_real_swap():
    runner = Runner()
    runner.run_until(FSMState.HERD, (2.0, 2.0), r1=(8.0, 8.0), r2=(1.5, 1.5))
    swaps = []
    drivers = []
    for _ in range(20):
        output = runner.run(1, (2.0, 2.0), r1=(1.5, 1.5), r2=(9.0, 9.0))
        drivers.append(output.driver_id)
        swaps.append(output.role_swapped)
    assert drivers[0] == 2 and drivers[-1] == 1, "cost-optimal driver must eventually flip"
    assert sum(swaps) == 1, "role_swapped must be True on exactly the flip cycle"
    flip_index = swaps.index(True)
    assert drivers[flip_index] == 1 and drivers[flip_index - 1] == 2


def test_role_assignment_candidate_is_not_biased_by_a_panicking_robot():
    """A robot inside panic distance must not win the Driver role just by being close.

    compute_driving_point() collapses to a retreat point next to the robot it is
    evaluated from, so using robot1's result as the role-assignment candidate would
    make robot1 unbeatable whenever it is inside panic_distance_m.
    """
    runner = Runner()
    # robot1 sits between the target and the goal (a bad Driver) and is in panic range;
    # robot2 sits behind the target, right at the ideal driving point (the good Driver).
    output = runner.run_until(FSMState.HERD, (2.0, 2.0), r1=(2.2, 2.2), r2=(1.4, 1.4))
    assert output.driver_id == 2
    assert output.panic is False  # robot2 (the Driver) is not in panic range


def test_panic_flag_propagates_from_the_driving_point():
    runner = Runner()
    output = runner.run_until(FSMState.HERD, (2.0, 2.0), r1=(1.9, 2.0), r2=(5.0, 5.0))
    assert output.driver_id == 1
    assert output.panic is True
    # Panic retreat drives robot 1 directly away from the target.
    np.testing.assert_allclose(output.robot1_goal, [1.65, 2.0], atol=1e-6)


def test_captured_state_is_reached_and_holds_position():
    runner = Runner()
    output = runner.run_until(FSMState.CAPTURED, (3.1, 3.0), limit=60)
    assert output.fsm_state == FSMState.CAPTURED
    np.testing.assert_allclose(output.robot1_goal, [0.0, 0.0])
    np.testing.assert_allclose(output.robot2_goal, [4.0, 4.0])


def test_off_grid_target_does_not_crash_the_cycle():
    """world_to_cell() raises out of bounds; the core must degrade, not explode."""
    runner = Runner()
    output = runner.run(6, (50.0, 50.0))
    assert output.fsm_state == FSMState.HERD
    assert output.escape_top3 == []
    # Driver still drives; blocker holds because no blocking point can be computed.
    assert output.driver_id == 2
    np.testing.assert_allclose(output.robot1_goal, [0.0, 0.0])


def test_mismatched_occupancy_shape_is_ignored():
    runner = Runner()
    output = runner.run(3, (2.0, 2.0), occupancy=np.zeros((5, 5), dtype=np.int8))
    assert runner.core.grid_map.obstacle_mask.shape == (40, 40)
    assert output.fsm_state == FSMState.HERD


def test_occupancy_of_the_right_shape_is_applied():
    runner = Runner()
    occupancy = np.zeros((40, 40), dtype=np.int8)
    occupancy[0:4, 0:4] = 100
    runner.run(3, (2.0, 2.0), occupancy=occupancy)
    assert runner.core.grid_map.obstacle_mask[0, 0]
    assert not runner.core.grid_map.obstacle_mask[20, 20]


def test_output_goals_do_not_alias_the_observation_arrays():
    runner = Runner()
    robot1 = np.array([0.0, 0.0])
    output = runner.core.step(Observation(
        target_measurement=None, robot1_pos=robot1, robot2_pos=np.array([1.0, 0.0]),
        robot1_heading=np.array([1.0, 0.0]), robot2_heading=np.array([1.0, 0.0]),
        occupancy=None, sim_time_sec=0.0, dt=0.2,
    ))
    output.robot1_goal[0] = 99.0
    assert robot1[0] == 0.0


def test_no_module_in_the_core_import_chain_imports_rclpy():
    import herding_controller.herding_core  # noqa: F401
    import sys
    for name, module in list(sys.modules.items()):
        if not name.startswith("herding_controller."):
            continue
        if name.endswith(".herding_node"):
            # herding_node.py is the single ROS2 boundary of the package: it is
            # required to import rclpy. Every other module must stay ROS-free so
            # the algorithm can be exercised offline.
            continue
        path = getattr(module, "__file__", None)
        if not path or not path.endswith(".py"):
            continue
        with open(path) as handle:
            source = handle.read()
        assert "import rclpy" not in source, f"{name} imports rclpy"
        assert "from rclpy" not in source, f"{name} imports rclpy"
