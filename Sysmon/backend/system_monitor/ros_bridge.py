"""ROS 2 토픽 메시지를 웹 모니터의 공통 상태 모델로 변환한다."""

from __future__ import annotations

import json
import math
import threading
import time
from typing import TYPE_CHECKING, Any

from .camera_service import CameraFrameStore
from .config import RobotConfig, RosInterfaceConfig
from .detection_service import (
    process_detection,
    record_low_battery,
    record_target_lost,
    record_trap_installed,
)
from .risk_signals import ENTRY_POINT, LIVE_RODENT, is_live_rodent, normalize_risk_signal
from .state_manager import StateManager

# ROS 2 패키지는 일반 PC의 Mock 모드에는 없어도 실행 가능해야 해서 전부
# 선택적으로 가져온다. TYPE_CHECKING 분기는 실행되지 않고 타입 체커(Pylance)만
# 읽는다 — 그래서 정적 분석에서는 "항상 성공하는 import"로 보여 이후 코드에서
# rclpy.ok() 같은 호출에 "None일 수도 있다"는 오탐이 안 뜨고, 실제 실행
# 시점에는 아래 else 블록의 try/except가 그대로 동작해 없으면 None으로
# 안전하게 빠진다.
if TYPE_CHECKING:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState, CompressedImage
    from std_msgs.msg import String
    from vision_msgs.msg import Detection3DArray

    from turtle_interfaces.msg import DetectionEvent
    from turtle_interfaces.srv import DbQuery
    from turtle_project.turtle_project import fleet_msg
else:
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
        from sensor_msgs.msg import BatteryState, CompressedImage
    except ImportError:
        BatteryState = None
        CompressedImage = None

    try:
        from rclpy.qos import qos_profile_sensor_data
    except ImportError:
        qos_profile_sensor_data = None

    try:
        from vision_msgs.msg import Detection3DArray
    except ImportError:
        Detection3DArray = None

    try:  # main의 커스텀 인터페이스 — 기록 조회(DbQuery)와 상세 탐지(DetectionEvent).
        from turtle_interfaces.msg import DetectionEvent
        from turtle_interfaces.srv import DbQuery
    except ImportError:
        DetectionEvent = None
        DbQuery = None

    try:  # colcon 설치 환경과 저장소에서 직접 실행하는 환경을 모두 지원한다.
        from turtle_project import fleet_msg
    except ImportError:
        import sys
        from pathlib import Path

        _TURTLE_SRC_DIR = Path(__file__).resolve().parents[3] / "src"
        if str(_TURTLE_SRC_DIR) not in sys.path:
            sys.path.insert(0, str(_TURTLE_SRC_DIR))
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

    def __init__(
        self,
        state: StateManager,
        robots: tuple[RobotConfig, ...],
        *,
        target_loss_timeout_sec: float = 1.5,
        low_battery_threshold: float = 15.0,
        interface: RosInterfaceConfig | None = None,
        camera_frame_store: CameraFrameStore | None = None,
    ) -> None:
        self.state = state
        self.robots = robots
        self.target_loss_timeout_sec = target_loss_timeout_sec
        self.low_battery_threshold = low_battery_threshold
        self.interface = interface or RosInterfaceConfig()
        self.camera_frame_store = camera_frame_store or CameraFrameStore()
        self._thread: threading.Thread | None = None
        self._node = None
        # db_node의 기록 조회 서비스 클라이언트 — spin 스레드에서 만들고
        # Flask 요청 스레드에서 호출한다(query_db 참고).
        self._db_client = None
        # 시스템 시작/정지 버튼용 /fleet/command 발행자 — Sysmon의 유일한 쓰기
        # 경로. spin 스레드에서 만들고 Flask 요청 스레드에서 publish한다.
        self._cmd_pub = None
        # 여러 ROS 콜백이 역할 배정·경고 상태를 동시에 바꾸지 않도록 보호한다.
        self._data_lock = threading.RLock()
        self._last_live_rodent_at: dict[str, float] = {}
        self._target_lost_reported: set[str] = set()
        self._low_battery_reported: set[str] = set()
        self._last_active_robot_id: str | None = None
        # /fleet/detection에서 받아 둔 최근 값 — object_type -> (robot_id,
        # confidence, 수신시각). robot_id 추측을 실제 값으로 대체하는 데 쓴다.
        self._last_detection_meta: dict[str, tuple[str, float, float]] = {}

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
                if DetectionEvent is not None:
                    # robot_id·confidence가 실제로 실려 오는 상세 탐지.
                    self.create_subscription(
                        DetectionEvent,
                        bridge.interface.fleet_detection_topic,
                        bridge._on_fleet_detection,
                        10,
                    )
                if DbQuery is not None:
                    bridge._db_client = self.create_client(
                        DbQuery, bridge.interface.db_query_service
                    )
                bridge._cmd_pub = self.create_publisher(
                    String, bridge.interface.fleet_command_topic, 10
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
                    if CompressedImage is not None:
                        self.create_subscription(
                            CompressedImage,
                            robot.topic(bridge.interface.camera_frame_topic),
                            lambda msg, robot_id=rid: bridge._on_camera_frame(
                                robot_id, msg
                            ),
                            qos_profile_sensor_data,
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

    def _on_fleet_event(self, msg: Any) -> None:

        try:
            name, map_x, map_y = fleet_msg.parse_event(msg.data)
        except (AttributeError, TypeError, ValueError):
            return
        name = name.lower()
        robot_id, confidence = self._detection_meta(name)

        with self._data_lock:
            if name in {"rat_detected", "opening_confirmed"}:
                object_type = LIVE_RODENT if name == "rat_detected" else ENTRY_POINT
                process_detection(
                    self.state,
                    {
                        "robot_id": robot_id,
                        "object_type": object_type,
                        "confidence": confidence,
                        "distance": None,
                        "map_x": map_x,
                        "map_y": map_y,
                        "source": "FLEET",
                        "review_status": "UNREVIEWED",
                        "image_url": self.camera_frame_store.image_url_for(robot_id),
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

    def _on_fleet_detection(self, msg: Any) -> None:
        """/fleet/detection — 화면 갱신은 /fleet/event가 그대로 담당하고, 여기서는
        거기에 실을 수 없는 robot_id·confidence만 받아 둔다.

        두 토픽은 같은 탐지에 대해 함께 오므로 여기서도 process_detection을 부르면
        같은 사건이 두 번 쌓인다. 그래서 보강만 하고 화면 흐름은 건드리지 않는다.
        """

        try:
            object_type = str(msg.object_type).upper()
            robot_id = str(msg.robot_id).strip()
            confidence = float(msg.confidence)
        except (AttributeError, TypeError, ValueError):
            return
        if not robot_id:
            return
        with self._data_lock:
            self._last_detection_meta[object_type] = (
                robot_id,
                confidence,
                time.monotonic(),
            )

    def _detection_meta(self, event_name: str) -> tuple[str, float | None]:
        """Fleet event에 붙일 robot_id·confidence. /fleet/detection이 안 붙어 있거나
        값이 오래됐으면 기존처럼 최근 활동 로봇으로 되돌아간다."""

        key = {"rat_detected": "RAT", "opening_confirmed": "OPENING"}.get(event_name)
        meta = self._last_detection_meta.get(key) if key else None
        if meta is not None and time.monotonic() - meta[2] <= 5.0:
            return meta[0], meta[1]
        return self._event_robot_id(), None

    def publish_command(self, data: str) -> str | None:
        """/fleet/command로 명령 1회 발행 — 성공이면 None, 실패면 사유 문자열.

        시스템 시작/정지 버튼 전용(Sysmon의 유일한 쓰기). publisher는 spin
        스레드가 만들지만 publish 자체는 스레드 안전해 Flask 스레드에서 불러도
        된다.
        """
        if String is None or self._cmd_pub is None or not self.running:
            return "ros_unavailable"
        msg = String()
        msg.data = data
        self._cmd_pub.publish(msg)
        return None

    def query_db(
        self,
        query_name: str,
        params: dict[str, Any] | None = None,
        timeout_sec: float = 3.0,
    ) -> tuple[Any, str | None]:
        """db_node의 기록 조회 서비스를 호출한다 — 읽기 전용, (결과, 오류) 반환.

        Flask 요청 스레드에서 부르고 응답은 ROS spin 스레드가 채우므로 future를
        폴링으로 기다린다. DB가 없거나 느려도 여기서만 실패하고 실시간 화면은
        그대로 돈다 — 관제 화면이 DB에 묶이지 않게 하려는 것이다.
        """

        if DbQuery is None or self._db_client is None:
            return None, "ros_unavailable"
        if not self._db_client.service_is_ready():
            return None, "db_node_unavailable"

        request = DbQuery.Request()
        request.query_name = query_name
        request.params_json = json.dumps(params or {})
        future = self._db_client.call_async(request)

        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            return None, "timeout"

        response = future.result()
        if response is None:
            return None, "no_response"
        if not response.ok:
            return None, response.error or "query_failed"
        try:
            return json.loads(response.result_json), None
        except ValueError as exc:
            return None, f"bad_json: {exc}"

    def _on_detection(self, robot_id: str, source: str, msg: Any) -> None:

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
            # 탐지 시점과 정확히 동기화된 프레임은 아니고, 그 로봇의 가장
            # 최근 캐시 프레임을 가리키는 최소 구현이다(카메라 미연결/Mock
            # 이면 None).
            "image_url": self.camera_frame_store.image_url_for(robot_id),
        }

    def _on_odom(self, robot_id: str, msg: Any) -> None:

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

    def _on_camera_frame(self, robot_id: str, msg: Any) -> None:

        self.camera_frame_store.update(
            robot_id, bytes(msg.data), msg.format or "jpeg"
        )

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
