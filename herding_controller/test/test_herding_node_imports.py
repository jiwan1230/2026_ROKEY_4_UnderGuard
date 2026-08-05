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
import rclpy
from geometry_msgs.msg import PoseStamped

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
