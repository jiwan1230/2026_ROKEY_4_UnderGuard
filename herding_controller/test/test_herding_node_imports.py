# herding_controller/test/test_herding_node_imports.py
"""herding_node.py에 대한 import 경계 + 어댑터 로직 테스트.

아래의 모듈 수준 import는 의도적이다: pytest는 어떤 테스트 본문을 실행하기 전에
모든 테스트 모듈을 완전히 수집(import)하므로, test_herding_core.py의
`test_no_module_in_the_core_import_chain_imports_rclpy`가 실행될 시점에는
`herding_controller.herding_node`가 이미 sys.modules에 존재한다.
이 덕분에 그 테스트의 `.herding_node` 예외 분기가 결코 실행되지 않는
죽은 코드가 아니라 실제로 실행되게 된다.
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
    """_load_config(node)는 인자 누락 없이 유효한 HerdingConfig를 생성해야 한다.

    실제(spin하지 않는) rclpy Node를 생성하므로, dict를 dataclass 정의와
    눈으로 비교하는 대신 launch 파일이 거치게 될 declare_parameter/get_parameter
    경로를 직접 실행한다.
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
    # 모든 dataclass 필드는 ROS 파라미터로 명시적으로 선언되거나
    # dataclass 기본값을 가져야 한다 -- 그렇지 않으면 위에서 이미
    # HerdingConfig(**values)가 TypeError를 발생시켰을 것이다.
    fields_without_default = {
        f.name for f in dataclasses.fields(HerdingConfig)
        if f.default is dataclasses.MISSING and f.default_factory is dataclasses.MISSING
    }
    assert fields_without_default <= declared_names, (
        f"HerdingConfig fields with no default are missing from _PARAM_DEFAULTS: "
        f"{fields_without_default - declared_names}"
    )
    # grid_origin_x_m/grid_origin_y_m은 dataclass 기본값을 갖고 있지만
    # 여전히 ROS 파라미터로 명시적으로 선언되어 있으므로(_PARAM_DEFAULTS 참조),
    # 부분집합 검사가 아니라 정확히 일치하는지 검사한다.
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
    encoded = json.dumps(payload)  # TypeError가 발생하면 안 됨
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
        json.dumps(payload)  # 어떤 enum 멤버에 대해서도 예외가 발생하면 안 됨
        assert payload["fsm_state"] == state.name


def test_prob_grid_to_flat_int8_range_shape_and_row_major_order():
    grid = np.array([[0.0, 0.5], [1.0, 2.0]])  # 2.0은 1.0으로 클리핑되는 경로를 검증함
    flat = herding_node._prob_grid_to_flat_int8(grid)
    assert flat == [0, 50, 100, 100]
    assert all(-1 <= v <= 100 for v in flat)
    assert all(isinstance(v, int) for v in flat)


def test_prob_grid_to_flat_int8_round_trips_with_on_map_reshape():
    """publisher의 flatten 순서는 `_on_map`의 reshape과 정확히 역순이어야 한다."""
    height, width = 3, 5
    grid = np.arange(height * width, dtype=float).reshape(height, width)
    grid = grid / grid.max()
    flat = herding_node._prob_grid_to_flat_int8(grid)
    reshaped = np.array(flat, dtype=int).reshape(height, width)
    expected = np.clip(grid, 0.0, 1.0) * 100.0
    assert np.array_equal(reshaped, expected.astype(np.int8))


# --- Fix 1: ~/escape_probability는 실제 escape 분포를 래스터화한다 --- #

def _make_grid_map():
    return GridMap(GridConfig(resolution_m=1.0, width_cells=10, height_cells=10))


def test_rasterize_escape_probabilities_paints_rays_from_target_cell():
    grid_map = _make_grid_map()
    # 두 방향: 정확히 "east"(dx=1, dy=0)는 높은 확률, "north"
    # (dx=0, dy=1)는 낮은 확률. 타겟은 셀 (row=5, col=5)에 위치.
    directions = np.array([[1.0, 0.0], [0.0, 1.0]])
    probabilities = np.array([0.8, 0.2])
    target_position = np.array(grid_map.cell_to_world(5, 5))
    grid = herding_node._rasterize_escape_probabilities(
        target_position, directions, probabilities, grid_map, cells_per_ray=2,
    )
    assert grid.shape == (10, 10)
    # East ray: col은 증가, row는 고정.
    assert grid[5, 6] == 0.8
    assert grid[5, 7] == 0.8
    # North ray: row는 증가, col은 고정.
    assert grid[6, 5] == 0.2
    assert grid[7, 5] == 0.2
    # 아무도 칠하지 않은 셀은 0으로 남는다.
    assert grid[0, 0] == 0.0


def test_rasterize_escape_probabilities_stops_at_grid_edge_without_crashing():
    grid_map = _make_grid_map()
    directions = np.array([[1.0, 0.0]])
    probabilities = np.array([0.9])
    target_position = np.array(grid_map.cell_to_world(0, 9))  # 이미 east 가장자리에 있음
    grid = herding_node._rasterize_escape_probabilities(
        target_position, directions, probabilities, grid_map, cells_per_ray=3,
    )
    assert grid.shape == (10, 10)  # IndexError 없이 ray가 단순히 잘림


def test_rasterize_escape_probabilities_off_grid_target_returns_all_zero():
    grid_map = _make_grid_map()
    directions = np.array([[1.0, 0.0]])
    probabilities = np.array([0.9])
    grid = herding_node._rasterize_escape_probabilities(
        np.array([1000.0, 1000.0]), directions, probabilities, grid_map,
    )
    assert np.array_equal(grid, np.zeros((10, 10)))


def test_to_escape_grid_is_all_zero_when_no_escape_estimate_this_cycle():
    """SEARCH/TRACK/LOST 상태는 escape_directions/probabilities를 갖지 않는다."""
    rclpy.init(args=[])
    try:
        node = herding_node.HerdingNode()
        try:
            output = _make_output(escape_directions=None, escape_probabilities=None)
            msg = node._to_escape_grid(output)
            # msg.data는 생성된 rosidl 메시지 클래스로부터 일반 리스트가 아니라
            # array.array('b', ...)로 반환된다 -- list()로 변환하여 비교한다.
            assert list(msg.data) == [0] * (node.config.grid_width_cells * node.config.grid_height_cells)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_to_escape_grid_reflects_the_current_cycles_distribution_not_stale_state():
    """publish는 이전 사이클에 남은 데이터를 조용히 재사용해서는 안 된다."""
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
            assert max(msg2.data) == 0  # 이전 사이클의 ray가 여전히 보이면 안 됨
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


# --- Fix 2: 두 로봇의 pose가 모두 확인될 때까지 goal publishing을 보류함 ------ #

def test_goal_publish_is_withheld_until_both_robot_poses_received():
    rclpy.init(args=[])
    try:
        node = herding_node.HerdingNode()
        try:
            published = []
            node.robot2_goal_pub.publish = lambda msg: published.append(("r2", msg))

            node._on_timer()  # 어느 pose도 아직 수신되지 않음
            assert published == []

            r1 = PoseStamped()
            r1.pose.position.x, r1.pose.position.y = 1.0, 1.0
            node._on_robot1_pose(r1)
            node._on_timer()  # robot1만 알려짐
            assert published == []

            r2 = PoseStamped()
            r2.pose.position.x, r2.pose.position.y = 9.0, 1.0
            node._on_robot2_pose(r2)
            node._on_timer()  # 이제 둘 다 알려짐 (robot1_goal은 애초에 발행되지 않음)
            assert {name for name, _ in published} == {"r2"}
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_other_topics_still_publish_while_goals_are_withheld():
    """goal 토픽만 게이팅된다 -- state/escape/capture는 계속 흘러야 한다."""
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


# --- Fix 3: robot heading은 pose의 orientation으로부터 추출된다 --------- #

def test_quaternion_to_heading_identity_points_along_positive_x():
    heading = herding_node._quaternion_to_heading(0.0, 0.0, 0.0, 1.0)
    assert np.allclose(heading, [1.0, 0.0])


def test_quaternion_to_heading_ninety_degree_yaw_points_along_positive_y():
    # Z축 기준 +90도 회전: qz = sin(45도), qw = cos(45도).
    heading = herding_node._quaternion_to_heading(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert np.allclose(heading, [0.0, 1.0], atol=1e-9)


def test_on_robot_pose_updates_heading_from_orientation_not_hardcoded():
    rclpy.init(args=[])
    try:
        node = herding_node.HerdingNode()
        try:
            assert np.allclose(node._robot1_heading, [1.0, 0.0])  # 어떤 pose도 오기 전의 기본값
            msg = PoseStamped()
            msg.pose.orientation.z = math.sin(math.pi / 4)
            msg.pose.orientation.w = math.cos(math.pi / 4)
            node._on_robot1_pose(msg)
            assert np.allclose(node._robot1_heading, [0.0, 1.0], atol=1e-9)

            # robot2는 robot1과 다른 orientation을 받는다: 두 heading이
            # 서로 독립적으로 추출되는지(한쪽이 다른 쪽 값을 덮어쓰지
            # 않는지) 확인한다.
            msg2 = PoseStamped()
            msg2.pose.orientation.w = 1.0  # identity: +X를 향함
            node._on_robot2_pose(msg2)
            assert not np.allclose(node._robot1_heading, node._robot2_heading)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


# --- 최종 검토 C2: 노드 기본값은 배포용 yaml과 정확히 일치해야 함 ---- #

def _load_shipping_yaml_params() -> dict:
    path = pathlib.Path(__file__).resolve().parent.parent / "config" / "herding_params.yaml"
    with open(path) as handle:
        return yaml.safe_load(handle)["herding_controller"]["ros__parameters"]


def test_param_defaults_match_shipping_yaml_values():
    """launch 파일(yaml) 없이 실행해도 있을 때와 정확히 동일하게 동작해야 한다.

    위쪽의 기존 테스트는 *이름*이 일치하는지만 확인한다. 이 검사가 존재하기
    전에는 노드의 내장 기본값이 여전히 Task-15 이전 튜닝값
    (drive_distance_m=0.8, ease=1.3, block_lookahead_m=1.2)을 갖고 있었는데,
    이는 yaml에 문서화된 drive/flee 제약을 위반한다 -- 즉 `ros2 run
    herding_controller herding_node`가 약 2.5% 성공률로 측정된 구성을
    조용히 실행하고 있었다.
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
    """기본값만으로도 HerdingConfig의 불변식을 만족해야 한다(yaml도, launch 파일도 없이)."""
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


# --- 최종 검토 C1/I3: freshness gating + 실시간 클록 dt ------------------- #

class _FakeClock:
    """HerdingNode._now_sec()를 대체할 수 있는 제어 가능한 스텁."""

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
    """rclpy logger 호출을 캡처하여 테스트가 운영자에게 보이는 출력을 검증할 수 있게 한다."""

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
    """시간 소스가 제어 가능한 카운터이며, 두 로봇의 pose가 모두 알려진 HerdingNode."""
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
    """새로운 ~/target_pose가 없는 사이클은 오래된 위치가 아니라 None을 HerdingCore에 전달해야 한다."""
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

            # 이 두 사이클 사이에는 새 pose가 publish되지 않았다.
            clock.advance(0.2)
            node._on_timer()
            assert seen[-1].target_measurement is None

            # 새 pose가 도착하면 다음 사이클은 다시 관측이 된다.
            node._on_target_pose(_target_pose(2.1, 2.0))
            clock.advance(0.2)
            node._on_timer()
            assert seen[-1].target_measurement is not None
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_fsm_enters_lost_after_occlusion_timeout_of_silent_target_topic():
    """LOST/occlusion-recovery 서브시스템 전체가 노드에서 도달 가능해야 한다.

    freshness gate가 도입되기 전에는 `_on_timer`가 마지막으로 받은 위치를
    매 사이클마다 영원히 다시 공급했고, `TargetEstimator.update()`는 매번
    `_time_since_obs`를 0으로 만들었으며, 실제 배포 환경에서 `is_lost`가
    결코 True가 될 수 없었다.
    """
    rclpy.init(args=[])
    try:
        node, clock = _make_node_with_fake_clock()
        try:
            dt = 0.2
            for _ in range(6):  # 매 사이클마다 관측됨: tracking 상태에 도달
                node._on_target_pose(_target_pose(2.0, 2.0))
                clock.advance(dt)
                node._on_timer()
            assert node.core.fsm.state in (FSMState.TRACK, FSMState.HERD, FSMState.CORNER)
            assert node.core.estimator.get_state().is_lost is False

            # 인식이 멈춤: 더 이상 아무도 ~/target_pose를 publish하지 않는다.
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
    """freshness gate는 정상적인 토픽에서 가짜 LOST 에피소드를 만들어내서는 안 된다."""
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
    """capture_hold/role_swap_cooldown/occlusion 타임아웃은 실제 벽시계 시간으로 동작해야 한다."""
    rclpy.init(args=[])
    try:
        node, clock = _make_node_with_fake_clock()
        try:
            seen = []
            real_step = node.core.step
            node.core.step = lambda obs: (seen.append(obs), real_step(obs))[1]

            nominal = 1.0 / node.config.control_rate_hz
            node._on_timer()  # 첫 사이클은 이전 타임스탬프가 없음: 명목 dt 사용
            assert seen[-1].dt == nominal

            clock.advance(0.9)  # 지터가 심하거나 지연된 사이클
            node._on_timer()
            assert seen[-1].dt == pytest.approx(0.9)
            assert seen[-1].sim_time_sec == pytest.approx(nominal + 0.9)

            clock.advance(0.05)  # 이른 사이클
            node._on_timer()
            assert seen[-1].dt == pytest.approx(0.05)
            assert seen[-1].sim_time_sec == pytest.approx(nominal + 0.95)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_non_positive_elapsed_time_falls_back_to_the_nominal_period():
    """정지되거나 거꾸로 흐르는 클록이 dt <= 0으로 인해 KF/타임아웃을 멈추게 해서는 안 된다."""
    rclpy.init(args=[])
    try:
        node, clock = _make_node_with_fake_clock()
        try:
            seen = []
            real_step = node.core.step
            node.core.step = lambda obs: (seen.append(obs), real_step(obs))[1]

            node._on_timer()
            node._on_timer()  # 클록이 전혀 진행되지 않음
            assert seen[-1].dt == 1.0 / node.config.control_rate_hz
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


# --- 최종 검토 I4: 실패한 사이클이 노드를 무너뜨려서는 안 됨 ----------- #

def test_timer_survives_an_exception_from_core_step_and_logs_it():
    rclpy.init(args=[])
    try:
        node, clock = _make_node_with_fake_clock()
        try:
            logger = _RecordingLogger()
            node.get_logger = lambda: logger
            published = []
            node.robot2_goal_pub.publish = lambda msg: published.append(msg)
            node.state_pub.publish = lambda msg: published.append(msg)

            def boom(_obs):
                raise RuntimeError("synthetic core failure")

            node.core.step = boom
            clock.advance(0.2)
            node._on_timer()  # 예외가 전파되면 안 됨

            assert published == []  # 실패한 사이클에서는 goal이 publish되지 않음
            assert len(logger.errors) == 1
            assert "synthetic core failure" in logger.errors[0]

            # 노드는 계속 실행된다: 다음 정상 사이클에서는 다시 publish된다.
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
            node._on_timer()  # 예외가 전파되면 안 됨
            assert len(logger.errors) == 1
            assert "synthetic publish failure" in logger.errors[0]
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


# --- 최종 검토 I2: map 불일치는 ROS logger를 통해 경고됨 - #

def _make_map_msg(node, height=None, width=None, resolution=None, origin_x=None, origin_y=None):
    msg = OccupancyGrid()
    msg.info.height = int(height if height is not None else node.config.grid_height_cells)
    msg.info.width = int(width if width is not None else node.config.grid_width_cells)
    msg.info.resolution = float(
        resolution if resolution is not None else node.config.grid_resolution_m
    )
    msg.info.origin.position.x = float(origin_x if origin_x is not None else node.config.grid_origin_x_m)
    msg.info.origin.position.y = float(origin_y if origin_y is not None else node.config.grid_origin_y_m)
    msg.data = [0] * (msg.info.height * msg.info.width)
    return msg


def test_on_map_warns_through_the_ros_logger_on_shape_mismatch():
    """HerdingCore의 stdlib-logging 경고는 /rosout에 도달하지 않는다; 노드의 경고는 도달해야 한다."""
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
            node._on_map(_make_map_msg(node, resolution=0.25, origin_x=-5.0, origin_y=-5.0))
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
