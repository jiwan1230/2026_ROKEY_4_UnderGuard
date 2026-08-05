# herding_controller/test/test_herding_node_imports.py
"""Import-boundary + adapter-logic tests for herding_node.py.

The module-level import below is deliberate: pytest fully collects (imports)
every test module before executing any test bodies, so by the time
test_herding_core.py's `test_no_module_in_the_core_import_chain_imports_rclpy`
runs, `herding_controller.herding_node` is already present in sys.modules.
That is what makes that test's `.herding_node` exemption branch actually get
exercised instead of being dead code that never triggers.
"""
import dataclasses
import json
import math
import pathlib

import numpy as np
import pytest
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.parameter import Parameter

import herding_controller.herding_node as herding_node
from herding_controller.grid_map import GridConfig, GridMap
from herding_controller.herding_core import HerdingConfig, HerdingOutput
from herding_controller.state_machine import FSMState


def test_herding_node_is_the_only_module_importing_rclpy():
    package_dir = pathlib.Path(__file__).resolve().parent.parent / "herding_controller"
    offenders = []
    for path in package_dir.glob("*.py"):
        if path.name == "herding_node.py":
            continue
        text = path.read_text()
        if "import rclpy" in text or "from rclpy" in text:
            offenders.append(path.name)
    assert offenders == []


def test_herding_node_module_has_expected_public_surface():
    assert hasattr(herding_node, "HerdingNode")
    assert hasattr(herding_node, "main")
    assert callable(herding_node.main)


def test_load_config_declares_every_herdingconfig_field_without_crashing():
    """_load_config(node) must produce a valid HerdingConfig with no missing args.

    This constructs a real (non-spinning) rclpy Node, so it directly exercises
    the declare_parameter/get_parameter path a launch file would hit, rather
    than just eyeballing the dict against the dataclass definition.
    """
    rclpy.init(args=[])
    try:
        node = rclpy.create_node("test_load_config_node")
        try:
            config = herding_node._load_config(node)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()

    assert isinstance(config, HerdingConfig)
    field_names = {f.name for f in dataclasses.fields(HerdingConfig)}
    declared_names = set(herding_node._PARAM_DEFAULTS)
    # Every dataclass field must either be explicitly declared as a ROS
    # parameter, or have a dataclass default -- otherwise HerdingConfig(**values)
    # would have already raised TypeError above.
    fields_without_default = {
        f.name for f in dataclasses.fields(HerdingConfig)
        if f.default is dataclasses.MISSING and f.default_factory is dataclasses.MISSING
    }
    assert fields_without_default <= declared_names, (
        f"HerdingConfig fields with no default are missing from _PARAM_DEFAULTS: "
        f"{fields_without_default - declared_names}"
    )
    # grid_origin_x_m/grid_origin_y_m have dataclass defaults but are still
    # explicitly declared as ROS parameters (see _PARAM_DEFAULTS), so this is
    # an exact match rather than a subset check.
    assert field_names == declared_names


def _make_output(**overrides):
    defaults = dict(
        robot1_goal=np.array([1.0, 2.0]),
        robot2_goal=np.array([3.0, 4.0]),
        fsm_state=FSMState.HERD,
        driver_id=1,
        blocker_id=2,
        target_position=np.array([0.5, 0.5]),
        target_velocity=np.array([0.1, -0.1]),
        escape_top3=[np.array([1.0, 1.0]), np.array([2.0, 2.0])],
        escape_directions=None,
        escape_probabilities=None,
        latency_ms=1.23,
        panic=False,
        role_swapped=True,
    )
    defaults.update(overrides)
    return HerdingOutput(**defaults)


def test_serialize_state_is_json_safe_with_escape_routes_and_enum():
    payload = herding_node._serialize_state(_make_output())
    encoded = json.dumps(payload)  # must not raise TypeError
    decoded = json.loads(encoded)
    assert decoded["fsm_state"] == "HERD"
    assert decoded["roles"] == {"driver": 1, "blocker": 2}
    assert decoded["target_pos"] == [0.5, 0.5]
    assert decoded["target_vel"] == [0.1, -0.1]
    assert decoded["escape_prob_top3"] == [[1.0, 1.0], [2.0, 2.0]]
    assert decoded["role_swapped"] is True
    assert decoded["panic"] is False


def test_serialize_state_handles_empty_escape_top3():
    payload = herding_node._serialize_state(_make_output(escape_top3=[]))
    encoded = json.dumps(payload)
    decoded = json.loads(encoded)
    assert decoded["escape_prob_top3"] == []


def test_serialize_state_handles_every_fsm_state():
    for state in FSMState:
        payload = herding_node._serialize_state(_make_output(fsm_state=state))
        json.dumps(payload)  # must not raise for any enum member
        assert payload["fsm_state"] == state.name


def test_prob_grid_to_flat_int8_range_shape_and_row_major_order():
    grid = np.array([[0.0, 0.5], [1.0, 2.0]])  # 2.0 exercises the clip-to-1.0 path
    flat = herding_node._prob_grid_to_flat_int8(grid)
    assert flat == [0, 50, 100, 100]
    assert all(-1 <= v <= 100 for v in flat)
    assert all(isinstance(v, int) for v in flat)


def test_prob_grid_to_flat_int8_round_trips_with_on_map_reshape():
    """The publisher's flatten order must be the inverse of `_on_map`'s reshape."""
    height, width = 3, 5
    grid = np.arange(height * width, dtype=float).reshape(height, width)
    grid = grid / grid.max()
    flat = herding_node._prob_grid_to_flat_int8(grid)
    reshaped = np.array(flat, dtype=int).reshape(height, width)
    expected = np.clip(grid, 0.0, 1.0) * 100.0
    assert np.array_equal(reshaped, expected.astype(np.int8))


# --- Fix 1: ~/escape_probability rasterizes the real escape distribution --- #

def _make_grid_map():
    return GridMap(GridConfig(resolution_m=1.0, width_cells=10, height_cells=10))


def test_rasterize_escape_probabilities_paints_rays_from_target_cell():
    grid_map = _make_grid_map()
    # Two directions: due "east" (dx=1, dy=0) at high probability, due "north"
    # (dx=0, dy=1) at lower probability. Target sits at cell (row=5, col=5).
    directions = np.array([[1.0, 0.0], [0.0, 1.0]])
    probabilities = np.array([0.8, 0.2])
    target_position = np.array(grid_map.cell_to_world(5, 5))
    grid = herding_node._rasterize_escape_probabilities(
        target_position, directions, probabilities, grid_map, cells_per_ray=2,
    )
    assert grid.shape == (10, 10)
    # East ray: col increases, row fixed.
    assert grid[5, 6] == 0.8
    assert grid[5, 7] == 0.8
    # North ray: row increases, col fixed.
    assert grid[6, 5] == 0.2
    assert grid[7, 5] == 0.2
    # A cell nobody painted stays zero.
    assert grid[0, 0] == 0.0


def test_rasterize_escape_probabilities_stops_at_grid_edge_without_crashing():
    grid_map = _make_grid_map()
    directions = np.array([[1.0, 0.0]])
    probabilities = np.array([0.9])
    target_position = np.array(grid_map.cell_to_world(0, 9))  # already at the east edge
    grid = herding_node._rasterize_escape_probabilities(
        target_position, directions, probabilities, grid_map, cells_per_ray=3,
    )
    assert grid.shape == (10, 10)  # no IndexError, ray simply truncates


def test_rasterize_escape_probabilities_off_grid_target_returns_all_zero():
    grid_map = _make_grid_map()
    directions = np.array([[1.0, 0.0]])
    probabilities = np.array([0.9])
    grid = herding_node._rasterize_escape_probabilities(
        np.array([1000.0, 1000.0]), directions, probabilities, grid_map,
    )
    assert np.array_equal(grid, np.zeros((10, 10)))


def test_to_escape_grid_is_all_zero_when_no_escape_estimate_this_cycle():
    """SEARCH/TRACK/LOST states carry no escape_directions/probabilities."""
    rclpy.init(args=[])
    try:
        node = herding_node.HerdingNode()
        try:
            output = _make_output(escape_directions=None, escape_probabilities=None)
            msg = node._to_escape_grid(output)
            # msg.data comes back as array.array('b', ...) from the generated
            # rosidl message class, not a plain list -- compare via list().
            assert list(msg.data) == [0] * (node.config.grid_width_cells * node.config.grid_height_cells)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_to_escape_grid_reflects_the_current_cycles_distribution_not_stale_state():
    """Publishing must not silently reuse leftover data from a previous cycle."""
    rclpy.init(args=[])
    try:
        node = herding_node.HerdingNode()
        try:
            directions = np.array([[1.0, 0.0]])
            probabilities = np.array([1.0])
            target_position = np.array(node.core.grid_map.cell_to_world(5, 5))
            populated = _make_output(
                target_position=target_position,
                escape_directions=directions, escape_probabilities=probabilities,
            )
            msg1 = node._to_escape_grid(populated)
            assert max(msg1.data) > 0

            empty = _make_output(escape_directions=None, escape_probabilities=None)
            msg2 = node._to_escape_grid(empty)
            assert max(msg2.data) == 0  # must not still show the previous cycle's ray
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


# --- Fix 2: goal publishing withheld until both robot poses are known ------ #

def test_goal_publish_is_withheld_until_both_robot_poses_received():
    rclpy.init(args=[])
    try:
        node = herding_node.HerdingNode()
        try:
            published = []
            node.robot1_goal_pub.publish = lambda msg: published.append(("r1", msg))
            node.robot2_goal_pub.publish = lambda msg: published.append(("r2", msg))

            node._on_timer()  # neither pose ever received
            assert published == []

            r1 = PoseStamped()
            r1.pose.position.x, r1.pose.position.y = 1.0, 1.0
            node._on_robot1_pose(r1)
            node._on_timer()  # only robot1 known
            assert published == []

            r2 = PoseStamped()
            r2.pose.position.x, r2.pose.position.y = 9.0, 1.0
            node._on_robot2_pose(r2)
            node._on_timer()  # both known now
            assert {name for name, _ in published} == {"r1", "r2"}
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_other_topics_still_publish_while_goals_are_withheld():
    """Only the goal topics are gated -- state/escape/capture must still flow."""
    rclpy.init(args=[])
    try:
        node = herding_node.HerdingNode()
        try:
            state_published = []
            node.state_pub.publish = lambda msg: state_published.append(msg)
            node._on_timer()
            assert len(state_published) == 1
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


# --- Fix 3: robot heading is extracted from the pose's orientation --------- #

def test_quaternion_to_heading_identity_points_along_positive_x():
    heading = herding_node._quaternion_to_heading(0.0, 0.0, 0.0, 1.0)
    assert np.allclose(heading, [1.0, 0.0])


def test_quaternion_to_heading_ninety_degree_yaw_points_along_positive_y():
    # Rotation of +90 deg about Z: qz = sin(45 deg), qw = cos(45 deg).
    heading = herding_node._quaternion_to_heading(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert np.allclose(heading, [0.0, 1.0], atol=1e-9)


def test_on_robot_pose_updates_heading_from_orientation_not_hardcoded():
    rclpy.init(args=[])
    try:
        node = herding_node.HerdingNode()
        try:
            assert np.allclose(node._robot1_heading, [1.0, 0.0])  # default before any pose
            msg = PoseStamped()
            msg.pose.orientation.z = math.sin(math.pi / 4)
            msg.pose.orientation.w = math.cos(math.pi / 4)
            node._on_robot1_pose(msg)
            assert np.allclose(node._robot1_heading, [0.0, 1.0], atol=1e-9)

            # robot2 gets a different orientation than robot1: role_cost_turn_weight
            # can only ever differentiate the two robots if their headings can differ.
            msg2 = PoseStamped()
            msg2.pose.orientation.w = 1.0  # identity: facing +X
            node._on_robot2_pose(msg2)
            assert not np.allclose(node._robot1_heading, node._robot2_heading)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


# --- Final review C2: node defaults must equal the shipping yaml exactly ---- #

def _load_shipping_yaml_params() -> dict:
    path = pathlib.Path(__file__).resolve().parent.parent / "config" / "herding_params.yaml"
    with open(path) as handle:
        return yaml.safe_load(handle)["herding_controller"]["ros__parameters"]


def test_param_defaults_match_shipping_yaml_values():
    """Running with no launch file (no yaml) must behave exactly like running with it.

    The pre-existing test above only checks that the *names* line up. Before this
    check existed the node's built-in defaults still held the pre-Task-15 tuning
    (drive_distance_m=0.8, ease=1.3, block_lookahead_m=1.2), which violates the
    drive/flee constraint documented in the yaml -- so `ros2 run herding_controller
    herding_node` silently ran a configuration measured at ~2.5% success.
    """
    params = _load_shipping_yaml_params()
    assert set(params) == set(herding_node._PARAM_DEFAULTS), (
        "yaml keys and _PARAM_DEFAULTS keys differ: "
        f"yaml-only={set(params) - set(herding_node._PARAM_DEFAULTS)}, "
        f"defaults-only={set(herding_node._PARAM_DEFAULTS) - set(params)}"
    )
    mismatched = {
        name: (herding_node._PARAM_DEFAULTS[name], value)
        for name, value in params.items()
        if herding_node._PARAM_DEFAULTS[name] != value
    }
    assert mismatched == {}, f"node default vs yaml value drift (default, yaml): {mismatched}"


def test_node_builtin_defaults_build_a_valid_herding_config():
    """The defaults alone must satisfy HerdingConfig's invariants (no yaml, no launch file)."""
    config = HerdingConfig(**herding_node._PARAM_DEFAULTS)  # raises if the invariant is violated
    assert config.drive_distance_m * config.drive_distance_ease_factor < config.flee_reaction_distance_m


def test_load_config_rejects_zero_control_rate_with_a_clear_error():
    rclpy.init(args=[])
    try:
        node = rclpy.create_node(
            "test_zero_rate_node",
            parameter_overrides=[Parameter("control_rate_hz", Parameter.Type.DOUBLE, 0.0)],
        )
        try:
            with pytest.raises(ValueError) as excinfo:
                herding_node._load_config(node)
            assert "control_rate_hz" in str(excinfo.value)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


# --- Final review C1/I3: freshness gating + real-clock dt ------------------- #

class _FakeClock:
    """Controllable stand-in for HerdingNode._now_sec()."""

    def __init__(self, start: float = 1000.0) -> None:
        self.t = start

    def advance(self, dt: float) -> None:
        self.t += dt

    def __call__(self) -> float:
        return self.t


def _target_pose(x: float, y: float) -> PoseStamped:
    msg = PoseStamped()
    msg.pose.position.x, msg.pose.position.y = x, y
    return msg


class _RecordingLogger:
    """Captures rclpy logger calls so tests can assert on operator-visible output."""

    def __init__(self) -> None:
        self.warnings = []
        self.errors = []

    def warn(self, message, *args, **kwargs):
        self.warnings.append(str(message))

    def warning(self, message, *args, **kwargs):
        self.warnings.append(str(message))

    def error(self, message, *args, **kwargs):
        self.errors.append(str(message))

    def info(self, message, *args, **kwargs):
        pass

    def debug(self, message, *args, **kwargs):
        pass


def _make_node_with_fake_clock():
    """A HerdingNode whose time source is a controllable counter, with both robot poses known."""
    node = herding_node.HerdingNode()
    clock = _FakeClock()
    node._now_sec = clock
    r1 = PoseStamped()
    r1.pose.position.x, r1.pose.position.y = 0.0, 0.0
    r1.pose.orientation.w = 1.0
    node._on_robot1_pose(r1)
    r2 = PoseStamped()
    r2.pose.position.x, r2.pose.position.y = 6.0, 6.0
    r2.pose.orientation.w = 1.0
    node._on_robot2_pose(r2)
    return node, clock


def test_timer_reports_no_observation_when_no_new_target_pose_arrived():
    """A cycle with no new ~/target_pose must hand HerdingCore None, not the stale position."""
    rclpy.init(args=[])
    try:
        node, clock = _make_node_with_fake_clock()
        try:
            seen = []
            real_step = node.core.step
            node.core.step = lambda obs: (seen.append(obs), real_step(obs))[1]

            node._on_target_pose(_target_pose(2.0, 2.0))
            clock.advance(0.2)
            node._on_timer()
            assert seen[-1].target_measurement is not None

            # No new pose published between these two cycles.
            clock.advance(0.2)
            node._on_timer()
            assert seen[-1].target_measurement is None

            # A new pose arriving makes the next cycle an observation again.
            node._on_target_pose(_target_pose(2.1, 2.0))
            clock.advance(0.2)
            node._on_timer()
            assert seen[-1].target_measurement is not None
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_fsm_enters_lost_after_occlusion_timeout_of_silent_target_topic():
    """The whole LOST/occlusion-recovery subsystem must be reachable from the node.

    Before the freshness gate, `_on_timer` re-fed the last received position every
    cycle forever, `TargetEstimator.update()` zeroed `_time_since_obs` each time,
    and `is_lost` could never become True in a real deployment.
    """
    rclpy.init(args=[])
    try:
        node, clock = _make_node_with_fake_clock()
        try:
            dt = 0.2
            for _ in range(6):  # observed every cycle: reach a tracking state
                node._on_target_pose(_target_pose(2.0, 2.0))
                clock.advance(dt)
                node._on_timer()
            assert node.core.fsm.state in (FSMState.TRACK, FSMState.HERD, FSMState.CORNER)
            assert node.core.estimator.get_state().is_lost is False

            # Perception goes silent: nothing publishes ~/target_pose any more.
            silent_cycles = int(node.config.occlusion_timeout_sec / dt) + 2
            for _ in range(silent_cycles):
                clock.advance(dt)
                node._on_timer()

            state = node.core.estimator.get_state()
            assert state.time_since_observation > node.config.occlusion_timeout_sec
            assert state.is_lost is True
            assert node.core.fsm.state == FSMState.LOST
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_fsm_stays_out_of_lost_while_target_poses_keep_arriving():
    """The freshness gate must not manufacture false LOST episodes on a healthy topic."""
    rclpy.init(args=[])
    try:
        node, clock = _make_node_with_fake_clock()
        try:
            for _ in range(int(node.config.occlusion_timeout_sec / 0.2) + 10):
                node._on_target_pose(_target_pose(2.0, 2.0))
                clock.advance(0.2)
                node._on_timer()
            assert node.core.estimator.get_state().is_lost is False
            assert node.core.fsm.state != FSMState.LOST
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_observation_dt_and_sim_time_track_the_real_clock_not_the_nominal_period():
    """capture_hold/role_swap_cooldown/occlusion timeouts must run on wall-clock time."""
    rclpy.init(args=[])
    try:
        node, clock = _make_node_with_fake_clock()
        try:
            seen = []
            real_step = node.core.step
            node.core.step = lambda obs: (seen.append(obs), real_step(obs))[1]

            nominal = 1.0 / node.config.control_rate_hz
            node._on_timer()  # first cycle has no previous timestamp: nominal dt
            assert seen[-1].dt == nominal

            clock.advance(0.9)  # a badly jittered / delayed cycle
            node._on_timer()
            assert seen[-1].dt == pytest.approx(0.9)
            assert seen[-1].sim_time_sec == pytest.approx(nominal + 0.9)

            clock.advance(0.05)  # an early cycle
            node._on_timer()
            assert seen[-1].dt == pytest.approx(0.05)
            assert seen[-1].sim_time_sec == pytest.approx(nominal + 0.95)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_non_positive_elapsed_time_falls_back_to_the_nominal_period():
    """A stalled or backwards clock must not stall the KF / timeouts with dt <= 0."""
    rclpy.init(args=[])
    try:
        node, clock = _make_node_with_fake_clock()
        try:
            seen = []
            real_step = node.core.step
            node.core.step = lambda obs: (seen.append(obs), real_step(obs))[1]

            node._on_timer()
            node._on_timer()  # clock did not advance at all
            assert seen[-1].dt == 1.0 / node.config.control_rate_hz
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


# --- Final review I4: a failing cycle must not tear down the node ----------- #

def test_timer_survives_an_exception_from_core_step_and_logs_it():
    rclpy.init(args=[])
    try:
        node, clock = _make_node_with_fake_clock()
        try:
            logger = _RecordingLogger()
            node.get_logger = lambda: logger
            published = []
            node.robot1_goal_pub.publish = lambda msg: published.append(msg)
            node.robot2_goal_pub.publish = lambda msg: published.append(msg)
            node.state_pub.publish = lambda msg: published.append(msg)

            def boom(_obs):
                raise RuntimeError("synthetic core failure")

            node.core.step = boom
            clock.advance(0.2)
            node._on_timer()  # must not propagate

            assert published == []  # no goals published for the failed cycle
            assert len(logger.errors) == 1
            assert "synthetic core failure" in logger.errors[0]

            # The node keeps running: a healthy next cycle publishes again.
            node.core.step = herding_node.HerdingCore(node.config).step
            clock.advance(0.2)
            node._on_timer()
            assert published != []
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_timer_survives_an_exception_raised_while_publishing():
    rclpy.init(args=[])
    try:
        node, clock = _make_node_with_fake_clock()
        try:
            logger = _RecordingLogger()
            node.get_logger = lambda: logger

            def boom(_msg):
                raise RuntimeError("synthetic publish failure")

            node.state_pub.publish = boom
            clock.advance(0.2)
            node._on_timer()  # must not propagate
            assert len(logger.errors) == 1
            assert "synthetic publish failure" in logger.errors[0]
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


# --- Final review I2: map mismatches are warned about through the ROS logger - #

def _make_map_msg(node, height=None, width=None, resolution=None, origin_x=0.0, origin_y=0.0):
    msg = OccupancyGrid()
    msg.info.height = int(height if height is not None else node.config.grid_height_cells)
    msg.info.width = int(width if width is not None else node.config.grid_width_cells)
    msg.info.resolution = float(
        resolution if resolution is not None else node.config.grid_resolution_m
    )
    msg.info.origin.position.x = float(origin_x)
    msg.info.origin.position.y = float(origin_y)
    msg.data = [0] * (msg.info.height * msg.info.width)
    return msg


def test_on_map_warns_through_the_ros_logger_on_shape_mismatch():
    """HerdingCore's stdlib-logging warning never reaches /rosout; the node's must."""
    rclpy.init(args=[])
    try:
        node, _ = _make_node_with_fake_clock()
        try:
            logger = _RecordingLogger()
            node.get_logger = lambda: logger
            node._on_map(_make_map_msg(node, height=60, width=80))
            assert len(logger.warnings) == 1
            assert "60x80" in logger.warnings[0]
            assert "grid_height_cells" in logger.warnings[0]
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_on_map_warns_on_resolution_and_origin_mismatch():
    rclpy.init(args=[])
    try:
        node, _ = _make_node_with_fake_clock()
        try:
            logger = _RecordingLogger()
            node.get_logger = lambda: logger
            node._on_map(_make_map_msg(node, resolution=0.05, origin_x=-5.0, origin_y=-5.0))
            joined = " ".join(logger.warnings)
            assert "resolution" in joined
            assert "origin" in joined
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_on_map_is_silent_and_stores_the_grid_when_the_map_matches_the_config():
    rclpy.init(args=[])
    try:
        node, _ = _make_node_with_fake_clock()
        try:
            logger = _RecordingLogger()
            node.get_logger = lambda: logger
            node._on_map(_make_map_msg(node))
            assert logger.warnings == []
            assert node._occupancy.shape == (
                node.config.grid_height_cells, node.config.grid_width_cells
            )
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
