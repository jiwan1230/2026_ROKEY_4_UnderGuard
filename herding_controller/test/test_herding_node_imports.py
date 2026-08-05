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
import pathlib

import numpy as np
import rclpy

import herding_controller.herding_node as herding_node
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
    assert field_names <= (declared_names | {"grid_origin_x_m", "grid_origin_y_m"})


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


def test_belief_to_flat_int8_range_shape_and_row_major_order():
    belief = np.array([[0.0, 0.5], [1.0, 2.0]])  # 2.0 exercises the clip-to-1.0 path
    flat = herding_node._belief_to_flat_int8(belief)
    assert flat == [0, 50, 100, 100]
    assert all(-1 <= v <= 100 for v in flat)
    assert all(isinstance(v, int) for v in flat)


def test_belief_to_flat_int8_round_trips_with_on_map_reshape():
    """The publisher's flatten order must be the inverse of `_on_map`'s reshape."""
    height, width = 3, 5
    belief = np.arange(height * width, dtype=float).reshape(height, width)
    belief = belief / belief.max()
    flat = herding_node._belief_to_flat_int8(belief)
    reshaped = np.array(flat, dtype=int).reshape(height, width)
    expected = np.clip(belief, 0.0, 1.0) * 100.0
    assert np.array_equal(reshaped, expected.astype(np.int8))
