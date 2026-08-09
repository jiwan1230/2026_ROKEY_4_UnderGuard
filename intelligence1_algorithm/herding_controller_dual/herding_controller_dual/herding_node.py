# herding_controller_dual/herding_controller_dual/herding_node.py
"""rclpy 어댑터: 이 패키지에서 ROS를 가져오는 유일한 파일. HerdingCore를 감싼다.

살아있는 ROS 노드를 필요로 하지 않는 모든 함수(config→dict 매핑, JSON 페이로드
구성, escape-probability 래스터화, 쿼터니언-헤딩 변환)는 rclpy 장치를 띄우지
않고도 단위 테스트할 수 있도록 순수 함수로 유지한다.
"""
import json
import traceback

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from herding_controller_dual.grid_map import GridMap
from herding_controller_dual.herding_core import HerdingConfig, HerdingCore, HerdingOutput, Observation
from herding_controller_dual.state_machine import FSMState

# 기본값은 config/herding_params.yaml의 ros__parameters 블록을 그대로 반영한다.
# HerdingConfig(herding_core.py 참고)의 모든 필드는 여기에 나타나야 하며(또는
# dataclass 기본값을 가져야 하며), 그렇지 않으면 HerdingConfig(**values)가
# 시작 시 TypeError를 발생시킨다.
_PARAM_DEFAULTS = {
    "frame_id": "map",
    "control_rate_hz": 5.0,
    # --- 캡처 존 --- (2026-08-06: 실제 방의 "top" 트랩 좌표 기본값, config/herding_params.yaml 참고)
    "capture_zone_x_m": -2.81,
    "capture_zone_y_m": -5.36,
    "capture_radius_m": 0.3,
    "capture_hold_sec": 1.5,  # 2026-08-06: 3.0 -> 1.5, 실제 맵 재검증 결과 (config/herding_params.yaml 주석 참고)
    # --- 그리드 --- (2026-08-06: maps/room_map.yaml 실측값으로 교체)
    "grid_resolution_m": 0.05,
    "grid_width_cells": 106,
    "grid_height_cells": 147,
    "grid_origin_x_m": -3.19,
    "grid_origin_y_m": -9.03,
    # --- 타겟 추정기 (KF) ---
    "kf_process_noise": 0.1,
    "kf_measurement_noise": 0.05,
    "occlusion_timeout_sec": 3.0,
    # --- 도주 모델 (마르코프) ---
    "markov_wall_follow_p": 0.70,
    "markov_wall_hug_p": 0.20,
    "markov_center_p": 0.10,
    "momentum_weight": 0.4,
    "robot_repulsion_weight": 1.5,
    "wall_detect_radius_cells": 1,
    "escape_route_top_k": 3,
    "escape_concentration_threshold": 0.5,
    # --- Herding 제어 ---
    # 2026-08-06 실제 맵(5.3x7.35m) 기준 재튜닝. drive_distance_m *
    # drive_distance_ease_factor는 flee_reaction_distance_m 미만이어야 한다
    # (0.3 * 1.15 = 0.345 < 0.42) -- 자세한 근거는 config/herding_params.yaml과
    # HerdingConfig.__post_init__을 참고. test_param_defaults_match_shipping_yaml_values가
    # 이 값들을 yaml에 고정시킨다.
    "drive_distance_m": 0.3,
    "flee_reaction_distance_m": 0.42,
    "panic_distance_m": 0.12,
    "alignment_threshold": 0.7,
    "drive_distance_ease_factor": 1.15,
    "block_lookahead_m": 1.8,  # 2026-08-06: 5.3x7.35m 실제 방 기준 재조정 (기존 3.0m는 10x10m 아레나 기준)
    "blocking_point_commit_sec": 1.0,  # 2026-08-07: 하드코딩 상수에서 이전 (트러블슈팅 노트 10-7)
    "deadlock_stall_sec": 3.0,  # 2026-08-07: 교착 감지 (트러블슈팅 노트 11-8)
    "deadlock_drift_radius_m": 0.1,
    "deadlock_release_distance_m": 1.0,
    # --- 최소 이격 거리 (Driver 실제 위치와 Blocker 목표점) ---
    "min_robot_separation_m": 0.6,
    # --- 동적 Driver/Blocker 역할 배정 (플랜 B 전용, role_assigner.py::RoleAssigner) ---
    # 옛 10x10m 아레나 검증값 그대로 이식(algorithm/dual-robot-herding
    # 브랜치, 4abf472) -- 이 방(5.3x7.35m) 기준 재튜닝은 안 됨.
    "role_swap_margin": 0.5,
    "role_swap_cooldown_sec": 2.0,
    "role_cost_turn_weight": 0.3,
    # --- 엔드게임 협공 (2026-08-09 채택, 트러블슈팅 노트 16-15) ---
    "endgame_pincer_enabled": True,
    "endgame_trigger_radius_m": 0.8,
    "endgame_half_angle_deg": 60.0,
    "endgame_stall_sec": 3.0,
    # --- 로봇 물리 치수 (협공 목표점의 벽 여유 검사에 사용) ---
    # 2026-08-09: 코드 상수에서 이전. 목표점이 설 수 있는 자리인지 볼 때
    # robot_radius_m + robot_wall_clearance_m를 요구 여유로 쓴다. 기체나
    # 벽 이격 기준이 바뀌면 재빌드 없이 여기서 바꾼다. Nav2의 inflation
    # radius와 어긋나면 우리가 유효하다고 판단한 목표점을 Nav2가 거부한다.
    "robot_radius_m": 0.171,
    "robot_wall_clearance_m": 0.03,
    # --- Occlusion 그리드 ---
    "diffusion_rate": 0.2,
    "decay_factor": 0.9,
}


def _load_config(node: Node) -> HerdingConfig:
    """HerdingConfig의 모든 필드를 ROS 파라미터로 선언하고 config를 만든다."""
    for name, default in _PARAM_DEFAULTS.items():
        node.declare_parameter(name, default)
    values = {name: node.get_parameter(name).value for name in _PARAM_DEFAULTS}
    # 제어 주기(1 / control_rate_hz)는 아래 두 곳에서 계산된다; rate가 0이면
    # 생성자 내부 깊은 곳에서 맨 ZeroDivisionError로만 드러날 것이다. 운영자가
    # 조치할 수 있도록 여기서 실패시킨다. (다른 파라미터 간 불변 조건들은
    # HerdingConfig.__post_init__에서 강제된다.)
    if not values["control_rate_hz"] > 0.0:
        raise ValueError(
            f"control_rate_hz must be > 0, got {values['control_rate_hz']}"
        )
    return HerdingConfig(**values)


def _serialize_state(output: HerdingOutput) -> dict:
    """HerdingOutput으로부터 JSON에 안전한 dict를 만든다 (numpy 배열도, enum도 없이)."""
    return {
        "fsm_state": output.fsm_state.name,
        "roles": {"driver": output.driver_id, "blocker": output.blocker_id},
        "target_pos": output.target_position.tolist(),
        "target_vel": output.target_velocity.tolist(),
        "escape_prob_top3": [route.tolist() for route in output.escape_top3],
        "latency_ms": output.latency_ms,
        "panic": output.panic,
        "role_swapped": output.role_swapped,
    }


def _prob_grid_to_flat_int8(grid: np.ndarray) -> list:
    """[0, 1] 확률 그리드를 [0, 100] 범위의 평탄한 row-major int8 리스트로 양자화한다.

    row-major(C order)는 `_on_map`이 들어오는 OccupancyGrid.data를 다시
    (height, width)로 reshape하는 방식과 일치한다: `np.array(data).reshape(h, w)`는
    여기서의 `.flatten(order="C")`의 정확한 역연산이다.
    """
    scaled = np.clip(grid, 0.0, 1.0) * 100.0
    return scaled.astype(np.int8).flatten(order="C").tolist()


def _rasterize_escape_probabilities(
    target_position: np.ndarray,
    directions: np.ndarray,
    probabilities: np.ndarray,
    grid_map: GridMap,
    cells_per_ray: int = 3,
) -> np.ndarray:
    """8방향 도주 분포를 (height, width) 그리드 위에 그린다.

    8방위 각각에 대해, 타겟이 위치한 그리드 셀에서부터 뻗어 나가는
    `cells_per_ray` 셀 길이의 짧은 광선이 해당 방향의 확률로 칠해진다
    (광선은 그리드 경계에서 조기에 멈춘다). 여러 광선이 도달할 수 있는
    셀은 합이 아니라 최댓값을 유지하므로, 광선이 겹쳐도 범위를 벗어나는
    값이 생기지 않는다. 타겟 위치가 그리드 밖에 있으면 (예외를 발생시키는
    대신 HerdingCore 자체의 off-grid 처리를 그대로 따라) all-zero 그리드를
    반환한다.
    """
    height, width = grid_map.config.height_cells, grid_map.config.width_cells
    grid = np.zeros((height, width))
    try:
        row0, col0 = grid_map.world_to_cell(float(target_position[0]), float(target_position[1]))
    except (ValueError, TypeError):
        return grid
    for (dx, dy), probability in zip(directions, probabilities):
        for step in range(1, cells_per_ray + 1):
            row = row0 + int(round(dy * step))
            col = col0 + int(round(dx * step))
            if not grid_map.in_bounds(row, col):
                break
            grid[row, col] = max(grid[row, col], float(probability))
    return grid


def _quaternion_to_heading(x: float, y: float, z: float, w: float) -> np.ndarray:
    """쿼터니언에서 평면(2D) yaw를 추출하여 단위 헤딩 벡터로 반환한다.

    Z축 회전에 대한 표준 쿼터니언-to-yaw 변환:
    yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)).
    """
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([np.cos(yaw), np.sin(yaw)])


class HerdingNode(Node):
    """poses/map을 구독하고 HerdingCore를 실행하며 목표점과 텔레메트리를 발행한다."""

    def __init__(self) -> None:
        super().__init__("herding_controller_dual")
        self.config = _load_config(self)
        self.core = HerdingCore(self.config)

        self._target_pos = None
        self._robot1_pos = np.zeros(2)
        self._robot2_pos = np.zeros(2)
        self._robot1_heading = np.array([1.0, 0.0])
        self._robot2_heading = np.array([1.0, 0.0])
        # 각 로봇에 대한 실제 pose가 도착하기 전까지 robotN_pos는 의미 없는
        # (0, 0) 자리 표시자다. 목표점 발행은 둘 다 True로 바뀔 때까지
        # 보류되므로(_publish 참고), 로봇이 실제 위치를 한 번도 보고하지
        # 않은 상태에서 그리드 원점 쪽으로 명령받는 일은 없다.
        self._robot1_pose_received = False
        self._robot2_pose_received = False
        self._occupancy = None
        # ~/target_pose가 도착한 시점과 다음 제어 주기가 그것을 소비하는
        # 시점 사이에서만 True이다. False인 상태로 주기를 맞이하면, 마지막
        # (오래된) 위치를 다시 넣는 대신 HerdingCore에게 "이번 주기에는
        # 관측이 없다"고 보고한다. 바로 이 덕분에 인지가 끊겼을 때 estimator의
        # occlusion timeout -- 그리고 결국 FSM의 LOST 상태 -- 가 발동할 수
        # 있다.
        self._target_pose_is_fresh = False
        # 벽시계 기준 기록: HerdingCore에 넘겨지는 sim_time_sec/dt는 명목상의
        # 1/control_rate_hz 카운터가 아니라 실제로 경과한 노드 클록 시간이므로,
        # 타이머 지터나 콜백 누락이 있어도 capture_hold_sec /
        # occlusion_timeout_sec가 실제 시간에 충실하게 유지된다.
        self._nominal_dt = 1.0 / self.config.control_rate_hz
        self._last_cycle_sec: float | None = None
        self._elapsed_sec = 0.0

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, "~/target_pose", self._on_target_pose, 10)
        self.create_subscription(PoseStamped, "~/robot1_pose", self._on_robot1_pose, 10)
        self.create_subscription(PoseStamped, "~/robot2_pose", self._on_robot2_pose, 10)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)

        # (플랜 B) 어느 로봇이 이번 주기 Driver/Blocker인지는 매 주기
        # 바뀔 수 있으므로(role_assigner.py::RoleAssigner), robot1_goal도
        # 발행한다 -- 플랜 A(herding_controller)는 로봇 A를 이 노드가
        # 조종하지 않는다는 이유로 발행하지 않았지만, 플랜 B는 두 로봇
        # 다 이 노드가 조종한다.
        self.robot1_goal_pub = self.create_publisher(PoseStamped, "~/robot1_goal", 10)
        self.robot2_goal_pub = self.create_publisher(PoseStamped, "~/robot2_goal", 10)
        self.state_pub = self.create_publisher(String, "~/herding_state", 10)
        self.escape_prob_pub = self.create_publisher(OccupancyGrid, "~/escape_probability", 10)
        self.capture_result_pub = self.create_publisher(Bool, "~/capture_result", 10)

        self.create_timer(self._nominal_dt, self._on_timer)

    def _now_sec(self) -> float:
        """노드 클록 시간(초). 테스트가 시간을 직접 제어할 수 있는 단일 접합점."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_target_pose(self, msg: PoseStamped) -> None:
        self._target_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        self._target_pose_is_fresh = True

    def _on_robot1_pose(self, msg: PoseStamped) -> None:
        self._robot1_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        q = msg.pose.orientation
        self._robot1_heading = _quaternion_to_heading(q.x, q.y, q.z, q.w)
        self._robot1_pose_received = True

    def _on_robot2_pose(self, msg: PoseStamped) -> None:
        self._robot2_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        q = msg.pose.orientation
        self._robot2_heading = _quaternion_to_heading(q.x, q.y, q.z, q.w)
        self._robot2_pose_received = True

    def _on_map(self, msg: OccupancyGrid) -> None:
        # OccupancyGrid 스펙상 msg.data는 항상 정확히 height*width개의 항목을
        # 가지므로, 들어오는 맵의 크기가 우리가 설정한
        # grid_width_cells/grid_height_cells와 다르더라도 이 reshape는 실패할
        # 수 없다. HerdingCore.step()도 자신의 grid config와 크기를 비교하여
        # 불일치하는 프레임을 무시하지만, 그것은 stdlib `logging`을 통해
        # 로그를 남기며 이는 ROS 로그로 연결되지 않으므로 운영자가 절대 볼 수
        # 없다. 여기서도 확인하여, 실제로 `ros2 run`의 콘솔과 /rosout에
        # 도달하는 rclpy 로거를 통해 경고한다.
        self._occupancy = np.array(msg.data, dtype=int).reshape(msg.info.height, msg.info.width)
        self._warn_on_map_mismatch(msg)

    def _warn_on_map_mismatch(self, msg: OccupancyGrid) -> None:
        """/map이 우리 grid config와 맞지 않을 때 (ROS 로거를 통해) 경고한다."""
        expected_shape = (self.config.grid_height_cells, self.config.grid_width_cells)
        if (msg.info.height, msg.info.width) != expected_shape:
            self.get_logger().warn(
                f"/map is {msg.info.height}x{msg.info.width} cells but "
                f"grid_height_cells x grid_width_cells is "
                f"{expected_shape[0]}x{expected_shape[1]}; the map will be IGNORED. "
                f"Set grid_width_cells/grid_height_cells to match the published map."
            )
        if not np.isclose(msg.info.resolution, self.config.grid_resolution_m):
            self.get_logger().warn(
                f"/map resolution is {msg.info.resolution} m/cell but "
                f"grid_resolution_m is {self.config.grid_resolution_m}; "
                f"world<->cell conversions will be wrong."
            )
        origin = msg.info.origin.position
        if not (np.isclose(origin.x, self.config.grid_origin_x_m)
                and np.isclose(origin.y, self.config.grid_origin_y_m)):
            self.get_logger().warn(
                f"/map origin is ({origin.x}, {origin.y}) but grid_origin_x_m/"
                f"grid_origin_y_m is ({self.config.grid_origin_x_m}, "
                f"{self.config.grid_origin_y_m}); obstacles will be offset."
            )

    def _on_timer(self) -> None:
        now_sec = self._now_sec()
        if self._last_cycle_sec is None:
            dt = self._nominal_dt  # 첫 주기: 비교할 이전 타임스탬프가 없음
        else:
            dt = now_sec - self._last_cycle_sec
            if dt <= 0.0:
                # 클록이 아직 돌지 않고 있거나(/clock 퍼블리셔 없이
                # use_sim_time을 쓰는 경우) 시간이 뒤로 튄 경우. dt가 0 이하면
                # KF와 모든 timeout이 정체되므로, 이번 주기는 명목 주기로
                # 대체한다.
                dt = self._nominal_dt
        self._last_cycle_sec = now_sec
        self._elapsed_sec += dt

        # 이전 주기 이후에 도착한 pose만 관측으로 취급한다; 그렇지 않으면
        # estimator가 오래된 위치를 계속 재융합하여 결코 LOST로 타임아웃되지
        # 않을 것이다.
        target_measurement = self._target_pos if self._target_pose_is_fresh else None
        self._target_pose_is_fresh = False

        observation = Observation(
            target_measurement=target_measurement,
            robot1_pos=self._robot1_pos, robot2_pos=self._robot2_pos,
            robot1_heading=self._robot1_heading, robot2_heading=self._robot2_heading,
            occupancy=self._occupancy, sim_time_sec=self._elapsed_sec, dt=dt,
        )
        try:
            output = self.core.step(observation)
            self._publish(output)
        except Exception:
            # 타이머 콜백을 벗어난 예외는 rclpy.spin()을 무너뜨려 미션 도중
            # 노드를 죽인다. 그 대신 로그를 남기고 이번 주기를 건너뛴다:
            # 로봇들은 그저 이전에 발행된 목표점을 그대로 유지한다.
            self.get_logger().error(
                f"control cycle failed, skipping this cycle's goals:\n{traceback.format_exc()}"
            )

    def _publish(self, output: HerdingOutput) -> None:
        # robot1_pos/robot2_pos가 실제 pose를 보고하기 전까지는 __init__에서
        # 설정한 (0, 0) 자리 표시자이며, HerdingCore는 (IDLE/SEARCH/TRACK/
        # CAPTURED 상태에서) 이를 그대로 목표점으로 되돌려준다 -- 여기서
        # 그것을 발행하면 로봇을 그리드 원점 쪽으로 실제로 몰아가게 된다.
        # (플랜 B) 두 로봇 다 이 노드가 조종하므로, 두 pose 모두 확정된
        # 뒤에야 robot1_goal/robot2_goal을 둘 다 발행한다.
        if self._robot1_pose_received and self._robot2_pose_received:
            self.robot1_goal_pub.publish(self._to_pose(output.robot1_goal))
            self.robot2_goal_pub.publish(self._to_pose(output.robot2_goal))

        state_msg = String()
        state_msg.data = json.dumps(_serialize_state(output))
        self.state_pub.publish(state_msg)

        self.escape_prob_pub.publish(self._to_escape_grid(output))

        capture_msg = Bool()
        capture_msg.data = output.fsm_state == FSMState.CAPTURED
        self.capture_result_pub.publish(capture_msg)

    def _to_pose(self, point: np.ndarray) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.config.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(point[0])
        msg.pose.position.y = float(point[1])
        msg.pose.orientation.w = 1.0
        return msg

    def _to_escape_grid(self, output: HerdingOutput) -> OccupancyGrid:
        """현재 도주 방향 분포를 맵 그리드 위에 래스터화한다.

        `output.escape_directions`/`escape_probabilities`는 이번 주기에
        escape model이 실제로 실행되었을 때만(즉 KF가 수렴했고 타겟의 셀이
        그리드 위에 있을 때만 -- HerdingCore.step() 참고) HerdingCore에 의해
        채워진다. 그 밖의 모든 상태(수렴 전 SEARCH/TRACK, LOST)에서는 정말로
        escape distribution이 존재하지 않으므로, 관련 없는 배열(예를 들어
        "숨어 있는 동안 타겟이 있을 만한 곳"을 나타낼 뿐 "어느 방향으로 도주할
        가능성이 높은지"는 나타내지 않는 occlusion belief)을 용도 변경하는
        대신 정직하게 all-zero 그리드를 발행한다.
        """
        msg = OccupancyGrid()
        msg.header.frame_id = self.config.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(self.config.grid_resolution_m)
        msg.info.width = int(self.config.grid_width_cells)
        msg.info.height = int(self.config.grid_height_cells)
        msg.info.origin.position.x = float(self.config.grid_origin_x_m)
        msg.info.origin.position.y = float(self.config.grid_origin_y_m)
        msg.info.origin.orientation.w = 1.0
        if output.escape_directions is not None and output.escape_probabilities is not None:
            grid = _rasterize_escape_probabilities(
                output.target_position, output.escape_directions, output.escape_probabilities,
                self.core.grid_map,
            )
        else:
            grid = np.zeros((self.config.grid_height_cells, self.config.grid_width_cells))
        msg.data = _prob_grid_to_flat_int8(grid)
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HerdingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
