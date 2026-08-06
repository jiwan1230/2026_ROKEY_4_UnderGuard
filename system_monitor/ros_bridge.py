"""ROS 2 토픽 메시지를 웹 모니터의 공통 상태 모델로 변환한다."""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from .config import RobotConfig, RosInterfaceConfig
from .detection_service import (
    process_detection,
    record_low_battery,
    record_target_lost,
    record_trap_installed,
)
from .risk_signals import ENTRY_POINT, LIVE_RODENT, is_live_rodent, normalize_risk_signal
from .state_manager import StateManager

try:
    import rclpy
    from std_msgs.msg import String
except ImportError:  # 일반 PC의 Mock 모드에서는 ROS 패키지가 없어도 실행 가능
    rclpy = None
    String = None

try:  # Fleet 계약 외 센서 토픽은 설치된 메시지 타입만 선택적으로 구독한다.
    from nav_msgs.msg import Odometry
except ImportError:
    Odometry = None

try:
    from sensor_msgs.msg import BatteryState
except ImportError:
    BatteryState = None

try:
    from vision_msgs.msg import Detection3DArray
except ImportError:
    Detection3DArray = None

try:  # colcon 설치 환경과 저장소에서 직접 실행하는 환경을 모두 지원한다.
    from turtle_project import fleet_msg
except ImportError:
    from turtle_project.turtle_project import fleet_msg


FLEET_STATE_MAP = {
    "IDLE": ("IDLE", "임무 대기"),
    "PATROLLING": ("SEARCHING", "창고 순찰 중"),
    "RETURNING": ("RETURNING", "도킹 위치 복귀 중"),
    "DOCKED": ("COMPLETED", "도킹 완료 · 다음 지시 대기"),
    "TRACKING": ("TRACKING", "쥐 추적 중"),
    "HERDING": ("NAVIGATING", "쥐 몰이 중"),
}

class RosBridge:
    """main Fleet 인터페이스와 보조 센서 토픽을 웹 상태로 변환한다.

    main 기준 1순위 인터페이스:
      - /fleet/status  : ``robot:state:battery``
      - /fleet/event   : ``event_name:x:y``

    선택적 보조 입력:
      - /<namespace>/webcam/detections : vision_msgs/Detection3DArray
      - /<namespace>/oakd/detections   : vision_msgs/Detection3DArray
      - /<namespace>/odom, battery_state
    """

    def __init__(
        self,
        state: StateManager,
        robots: tuple[RobotConfig, ...],
        *,
        target_loss_timeout_sec: float = 1.5,
        low_battery_threshold: float = 15.0,
        interface: RosInterfaceConfig | None = None,
    ) -> None:
        self.state = state
        self.robots = robots
        self.target_loss_timeout_sec = target_loss_timeout_sec
        self.low_battery_threshold = low_battery_threshold
        self.interface = interface or RosInterfaceConfig()
        self._thread: threading.Thread | None = None
        self._node = None
        # 여러 ROS 콜백이 역할 배정·경고 상태를 동시에 바꾸지 않도록 보호한다.
        self._data_lock = threading.RLock()
        self._last_live_rodent_at: dict[str, float] = {}
        self._target_lost_reported: set[str] = set()
        self._low_battery_reported: set[str] = set()
        self._last_active_robot_id: str | None = None

    @property
    def available(self) -> bool:
        return rclpy is not None and String is not None

    @property
    def running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def status(self) -> dict[str, Any]:
        """Mock과 동일한 필드로 ROS 런타임 상태와 지원 기능을 반환한다."""

        return {
            "mode": "ros",
            "available": self.available,
            "running": self.running,
            "read_only": True,
            "low_battery_threshold": self.low_battery_threshold,
        }

    def start(self) -> None:
        """ROS 노드와 spin 스레드를 시작한다.

        입력: 없음. 생성자에서 전달된 로봇별 토픽 설정을 사용한다.
        출력: 없음. ROS 패키지가 없으면 ``RuntimeError``를 발생시킨다.
        사용: ``MONITOR_MODE=ros``일 때 앱 시작 과정에서 한 번 호출한다.
        """

        if not self.available:
            raise RuntimeError("ROS 모드에는 ROS 2 Humble의 rclpy가 필요합니다.")
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._spin, daemon=True, name="ros-bridge")
        self._thread.start()

    def stop(self) -> None:
        """ROS spin을 종료하고 백그라운드 스레드를 정리한다."""
        if not self._thread:
            return
        if self.available and rclpy.ok():
            rclpy.shutdown()
        self._thread.join(timeout=2.0)
        self._thread = None

    def _spin(self) -> None:
        """로봇별 ROS 구독을 생성하고 종료 요청까지 콜백을 처리한다.

        입력: 생성자에 등록된 로봇 namespace와 역할 설정이다.
        출력: 없음. ROS 노드를 생성·spin하고 종료 시 자원을 정리한다.
        사용: ``start()``가 만든 전용 스레드의 진입점이다.
        """

        rclpy.init(args=None)
        from rclpy.node import Node

        bridge = self

        class MonitorNode(Node):
            def __init__(self) -> None:
                super().__init__("system_monitor_bridge")
                self.create_subscription(
                    String,
                    bridge.interface.fleet_status_topic,
                    bridge._on_fleet_status,
                    10,
                )
                self.create_subscription(
                    String,
                    bridge.interface.fleet_event_topic,
                    bridge._on_fleet_event,
                    10,
                )
                for robot in bridge.robots:
                    rid = robot.robot_id
                    # 기본 인자로 rid를 고정해 모든 lambda가 마지막 로봇을
                    # 참조하는 파이썬 late-binding 문제를 방지한다.
                    if Detection3DArray is not None:
                        self.create_subscription(
                            Detection3DArray,
                            robot.topic(bridge.interface.webcam_detections_topic),
                            lambda msg, robot_id=rid: bridge._on_detection(
                                robot_id, "WEBCAM", msg
                            ),
                            10,
                        )
                        self.create_subscription(
                            Detection3DArray,
                            robot.topic(bridge.interface.oakd_detections_topic),
                            lambda msg, robot_id=rid: bridge._on_detection(
                                robot_id, "OAK-D", msg
                            ),
                            10,
                        )
                    if Odometry is not None:
                        self.create_subscription(
                            Odometry,
                            robot.topic(bridge.interface.odometry_topic),
                            lambda msg, robot_id=rid: bridge._on_odom(robot_id, msg),
                            10,
                        )
                    if BatteryState is not None:
                        self.create_subscription(
                            BatteryState,
                            robot.topic(bridge.interface.battery_topic),
                            lambda msg, robot_id=rid: bridge._on_battery(robot_id, msg),
                            10,
                        )
                self.create_timer(0.5, bridge._check_target_timeouts)

        self._node = MonitorNode()
        try:
            rclpy.spin(self._node)
        finally:
            self._node.destroy_node()
            self._node = None
            if rclpy.ok():
                rclpy.shutdown()

    def _on_fleet_status(self, msg: Any) -> None:
        """main의 ``robot:state:battery`` 상태를 관제 상태로 변환한다.

        입력: ``std_msgs/String`` Fleet status다. 출력은 없다.
        사용: `/fleet/status` 콜백이며 알 수 없는 로봇·잘못된 포맷은 무시한다.
        """

        try:
            robot_id, fleet_state, battery = fleet_msg.parse_status(msg.data)
            self.state.get_robot(robot_id)
        except (AttributeError, KeyError, TypeError, ValueError):
            return

        fleet_state = fleet_state.upper()
        mapped = FLEET_STATE_MAP.get(fleet_state)
        if mapped is None:
            mapped = ("ERROR", f"알 수 없는 Fleet 상태: {fleet_state}")
        state_name, task = mapped
        battery_value = max(0.0, min(100.0, float(battery)))
        moving = fleet_state in {"PATROLLING", "RETURNING", "TRACKING", "HERDING"}

        with self._data_lock:
            below_threshold = battery_value < self.low_battery_threshold
            not_reported = robot_id not in self._low_battery_reported
            is_new_low_battery = below_threshold and not_reported
            if is_new_low_battery:
                record_low_battery(self.state, robot_id, battery_value)
                self._low_battery_reported.add(robot_id)
            elif battery_value >= self.low_battery_threshold + 2:
                self._low_battery_reported.discard(robot_id)

            self.state.update_robot(
                robot_id,
                state=state_name,
                battery=battery_value,
                current_task=task,
                nav_status="MOVING" if moving else "STOPPED",
            )
            if moving:
                self._last_active_robot_id = robot_id
                self.state.mark_mission_started()

    def _on_fleet_event(self, msg: Any) -> None:
        """main의 ``event_name:x:y`` 사건을 탐지·트랩 기록으로 변환한다.

        입력: map 좌표를 담은 ``std_msgs/String`` Fleet event다. 출력은 없다.
        사용: `/fleet/event` 콜백이다. 현재 계약에 robot_id가 없어 최근 활동
        로봇을 사용하고, 상태가 없으면 설정된 첫 로봇을 사용한다.
        """

        try:
            name, map_x, map_y = fleet_msg.parse_event(msg.data)
        except (AttributeError, TypeError, ValueError):
            return
        name = name.lower()
        robot_id = self._event_robot_id()

        with self._data_lock:
            if name in {"rat_detected", "opening_confirmed"}:
                object_type = LIVE_RODENT if name == "rat_detected" else ENTRY_POINT
                process_detection(
                    self.state,
                    {
                        "robot_id": robot_id,
                        "object_type": object_type,
                        "confidence": None,
                        "distance": None,
                        "map_x": map_x,
                        "map_y": map_y,
                        "source": "FLEET",
                        "review_status": "UNREVIEWED",
                    },
                    event_message=f"Fleet에서 {object_type} 사건을 수신했습니다.",
                    fallback_state="SEARCHING",
                    fallback_task="탐지 위치 확인 중",
                    camera_status="NORMAL",
                )
                return
            if name == "trap_ok":
                record_trap_installed(
                    self.state,
                    robot_id,
                    map_frame=self.interface.map_frame,
                    map_x=map_x,
                    map_y=map_y,
                )

    def _event_robot_id(self) -> str:
        """robot_id가 없는 main Fleet event의 표시 대상을 결정한다."""

        if self._last_active_robot_id is not None:
            return self._last_active_robot_id
        return self.robots[0].robot_id

    def _on_detection(self, robot_id: str, source: str, msg: Any) -> None:
        """Detection3DArray의 유효 항목을 공통 탐지 서비스로 전달한다.

        입력: 구독 토픽의 로봇 ID, 센서 이름, ``Detection3DArray`` 메시지다.
        출력: 없음. 상태 heartbeat와 현재 세션의 탐지·타임라인을 갱신한다.
        사용: webcam/oakd detection 구독 콜백으로 등록한다.
        """

        with self._data_lock:
            self.state.mark_heartbeat(robot_id)
            for detection_msg in msg.detections:
                data = self._to_detection_data(robot_id, source, msg, detection_msg)
                if data is None:
                    continue
                item = process_detection(
                    self.state,
                    data,
                    event_message=f"{source}에서 {data['object_type']}를 탐지했습니다.",
                    fallback_state=(
                        "APPROACHING" if source != "OAK-D" else "SEARCHING"
                    ),
                    fallback_task=f"{data['object_type']} 탐지 확인 중",
                    camera_status="NORMAL",
                )
                if source.upper() == "OAK-D" and is_live_rodent(
                    item["object_type"]
                ):
                    self._last_live_rodent_at[robot_id] = time.monotonic()
                    self._target_lost_reported.discard(robot_id)

    def _to_detection_data(
        self,
        robot_id: str,
        source: str,
        array_msg: Any,
        detection_msg: Any,
    ) -> dict[str, Any] | None:
        """ROS Detection3D 한 건을 모니터의 공통 탐지 형식으로 변환한다.

        입력: 로봇/센서 식별자, 배열 메시지 header, 개별 Detection3D다.
        출력: 첫 hypothesis 기반 탐지 딕셔너리이며 결과가 없으면 ``None``이다.
        사용: ``_on_detection``에서 각 detection 항목마다 호출한다.
        """

        if not detection_msg.results:
            return None

        hypothesis = detection_msg.results[0].hypothesis
        center = detection_msg.bbox.center.position
        distance = math.hypot(center.x, center.y) if center.z == 0 else abs(center.z)

        # Bounding box 중심은 header 좌표계에 속하므로 map 좌표만 지도에 쓴다.
        # 다른 좌표계는 TF 변환을 연결하기 전까지 지도 좌표에서 제외한다.
        header = getattr(array_msg, "header", None)
        frame_id = str(getattr(header, "frame_id", "")).strip("/")
        in_map_frame = frame_id == self.interface.map_frame.strip("/")
        return {
            "robot_id": robot_id,
            "object_type": normalize_risk_signal(hypothesis.class_id),
            "confidence": float(hypothesis.score),
            "distance": float(distance),
            "map_x": float(center.x) if in_map_frame else None,
            "map_y": float(center.y) if in_map_frame else None,
            "source": source,
            "review_status": "UNREVIEWED",
        }

    def _on_odom(self, robot_id: str, msg: Any) -> None:
        """Odometry를 화면용 x/y/yaw/speed로 변환한다.

        입력: 구독 토픽의 로봇 ID와 ``nav_msgs/Odometry`` 메시지다.
        출력: 없음. 해당 로봇의 위치, 속도, 이동 상태를 갱신한다.
        사용: 로봇별 ``/<namespace>/odom`` 구독 콜백으로 등록한다.
        """

        # 현재 위치는 Odometry header 좌표계 기준이다. 실제 지도 표시는 후속 TF
        # 연동에서 map -> base_link 위치로 교체해야 한다.
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        header = getattr(msg, "header", None)
        position_frame = str(getattr(header, "frame_id", "odom") or "odom").strip("/")
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear
        speed = math.hypot(v.x, v.y)
        self.state.update_robot(
            robot_id,
            position={"x": p.x, "y": p.y, "yaw": yaw},
            position_frame=position_frame,
            speed=speed,
            nav_status="MOVING" if speed > 0.02 else "STOPPED",
            slam_status="NORMAL",
        )

    def _on_battery(self, robot_id: str, msg: Any) -> None:
        """BatteryState 값을 화면용 백분율로 정규화한다.

        입력: 구독 토픽의 로봇 ID와 ``sensor_msgs/BatteryState`` 메시지다.
        출력: 없음. 유효한 배터리 값을 0~100 범위로 상태에 반영한다.
        사용: 로봇별 ``/<namespace>/battery_state`` 구독 콜백으로 등록한다.
        """

        percentage = msg.percentage
        if percentage is None or math.isnan(percentage):
            return
        value = percentage * 100 if percentage <= 1.0 else percentage
        value = round(max(0.0, min(100.0, value)), 3)
        with self._data_lock:
            below_threshold = value < self.low_battery_threshold
            not_reported = robot_id not in self._low_battery_reported
            is_new_low_battery = below_threshold and not_reported
            if is_new_low_battery:
                record_low_battery(self.state, robot_id, value)
                self._low_battery_reported.add(robot_id)
                return

            self.state.update_robot(robot_id, battery=value)
            if value >= self.low_battery_threshold + 2:
                self._low_battery_reported.discard(robot_id)

    def _check_target_timeouts(self) -> None:
        """OAK-D 쥐 탐지가 끊긴 추적 로봇을 한 번만 대상 유실로 전환한다."""

        with self._data_lock:
            now = time.monotonic()
            for robot_id, last_seen in tuple(self._last_live_rodent_at.items()):
                timed_out = now - last_seen >= self.target_loss_timeout_sec
                if not timed_out or robot_id in self._target_lost_reported:
                    continue
                robot = self.state.get_robot(robot_id)
                if robot["state"] != "TRACKING":
                    continue
                record_target_lost(self.state, robot_id)
                self._target_lost_reported.add(robot_id)
