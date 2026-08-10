"""ROS 2와 웹 관제 서버 사이에서 데이터 형식을 바꾸어 주는 연결부.

전체 데이터 흐름은 다음과 같다.

    ROS 2 토픽 -> RosBridge의 콜백 -> 데이터 변환/판단
        -> StateManager / DetectionService / CameraFrameStore
        -> Flask API -> 웹 관제 화면

ROS에서 메시지가 도착하면 해당 토픽에 등록된 ``_on_*`` 콜백이 실행된다.
콜백은 ROS 메시지를 웹 서버가 공통으로 사용하는 형태로 바꾼 뒤 저장한다.
이 클래스는 토픽을 구독하기만 하는 읽기 전용 다리이며, 로봇에 명령을
발행하지 않는다.
"""

from __future__ import annotations

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


# ROS가 설치되지 않은 개발 PC에서도 Mock 모드는 실행할 수 있어야 한다.
# TYPE_CHECKING 블록은 Pylance 같은 타입 검사 도구만 읽고 실제로는 실행하지
# 않는다. 실제 실행에서는 아래 try/except로 패키지를 선택적으로 가져온다.
if TYPE_CHECKING:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState, CompressedImage
    from std_msgs.msg import String
    from vision_msgs.msg import Detection3DArray

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

    try:  # colcon 설치 환경과 저장소에서 직접 실행하는 환경을 모두 지원한다.
        from turtle_project import fleet_msg
    except ImportError:
        import sys
        from pathlib import Path

        _TURTLE_SRC_DIR = Path(__file__).resolve().parents[3] / "src"
        if str(_TURTLE_SRC_DIR) not in sys.path:
            sys.path.insert(0, str(_TURTLE_SRC_DIR))
        from turtle_project.turtle_project import fleet_msg


# 로봇 제어 프로그램의 상태 이름을 웹 관제 화면의 상태와 설명으로 변환한다.
# 예: Fleet의 PATROLLING -> 화면의 SEARCHING + "창고 순찰 중"
FLEET_STATE_MAP = {
    "IDLE": ("IDLE", "임무 대기"),
    "PATROLLING": ("SEARCHING", "창고 순찰 중"),
    "RETURNING": ("RETURNING", "도킹 위치 복귀 중"),
    "DOCKED": ("COMPLETED", "도킹 완료 · 다음 지시 대기"),
    "TRACKING": ("TRACKING", "쥐 추적 중"),
    "HERDING": ("NAVIGATING", "쥐 몰이 중"),
}


class RosBridge:
    """ROS 메시지를 받아 관제 서버의 공통 상태로 반영한다.

    ROS의 ``spin``은 메시지를 계속 기다리는 작업이라 메인 웹 서버와 같은
    스레드에서 돌리면 Flask 요청 처리가 멈춘다. 따라서 ``start()``가 별도
    백그라운드 스레드를 만들고, 그 안에서 ROS 노드를 실행한다.
    """

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
        # 외부에서 주입받은 저장소와 설정을 보관한다. 이렇게 하면 실제 ROS가
        # 없는 테스트에서도 가짜 메시지를 콜백에 직접 넣어 검증할 수 있다.
        self.state = state
        self.robots = robots
        self.target_loss_timeout_sec = target_loss_timeout_sec
        self.low_battery_threshold = low_battery_threshold
        self.interface = interface or RosInterfaceConfig()
        self.camera_frame_store = camera_frame_store or CameraFrameStore()
        self._thread: threading.Thread | None = None
        self._node = None
        # 타이머와 여러 콜백이 역할 배정·경고 기록을 동시에 바꾸지 않도록
        # 공유 데이터를 잠근다. RLock은 잠금 안에서 다른 잠금 코드를 다시
        # 호출해도 같은 스레드라면 교착 상태가 나지 않는 재진입 잠금이다.
        self._data_lock = threading.RLock()
        # 아래 자료구조들은 '마지막 탐지 시각'과 '이미 보고한 경고'를 기억해
        # 같은 대상 유실/저전압 경고가 메시지마다 반복 생성되는 것을 막는다.
        self._last_live_rodent_at: dict[str, float] = {}
        self._target_lost_reported: set[str] = set()
        self._low_battery_reported: set[str] = set()
        self._last_active_robot_id: str | None = None

    @property
    def available(self) -> bool:
        """현재 환경에서 최소 ROS 의존성을 사용할 수 있는지 알려 준다."""

        return rclpy is not None and String is not None

    @property
    def running(self) -> bool:
        """ROS 수신용 백그라운드 스레드가 살아 있는지 알려 준다."""

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
        """웹 서버를 막지 않도록 별도 스레드에서 ROS 수신을 시작한다."""

        if not self.available:
            raise RuntimeError("ROS 모드에는 ROS 2 Humble의 rclpy가 필요합니다.")
        if self._thread and self._thread.is_alive():
            return
        # daemon 스레드는 애플리케이션 종료 시 이 스레드만 남아서 프로세스가
        # 계속 살아 있는 일을 막는다.
        self._thread = threading.Thread(
            target=self._spin, daemon=True, name="ros-bridge"
        )
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
        """ROS 노드를 만들고 종료 요청이 올 때까지 메시지를 처리한다."""

        # rclpy.init -> Node 생성 -> rclpy.spin 순서가 ROS 2 노드의 기본
        # 생명주기다. spin이 구독 메시지와 타이머 콜백을 반복 실행한다.
        rclpy.init(args=None)
        from rclpy.node import Node

        bridge = self

        class MonitorNode(Node):
            def __init__(self) -> None:
                super().__init__("system_monitor_bridge")
                # 공용 Fleet 토픽은 로봇별 namespace가 붙지 않는다. 숫자 10은
                # 콜백이 처리하기 전까지 보관할 메시지 큐의 기본 깊이다.
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
                    if CompressedImage is not None:
                        # 카메라는 데이터가 크고 최신 프레임이 중요하므로 일반
                        # 큐 대신 ROS의 센서 데이터용 QoS 정책을 사용한다.
                        self.create_subscription(
                            CompressedImage,
                            robot.topic(bridge.interface.camera_frame_topic),
                            lambda msg, robot_id=rid: bridge._on_camera_frame(
                                robot_id, msg
                            ),
                            qos_profile_sensor_data,
                        )
                # 새 메시지가 없어도 0.5초마다 마지막 쥐 탐지 시각을 검사한다.
                self.create_timer(0.5, bridge._check_target_timeouts)

        self._node = MonitorNode()
        try:
            # 종료 요청이 들어올 때까지 대기하면서 등록된 콜백을 실행한다.
            rclpy.spin(self._node)
        finally:
            # 예외가 발생해도 ROS 자원이 남지 않도록 반드시 정리한다.
            self._node.destroy_node()
            self._node = None
            if rclpy.ok():
                rclpy.shutdown()

    def _on_fleet_status(self, msg: Any) -> None:
        """Fleet 상태 문자열을 로봇의 현재 상태와 배터리 정보로 반영한다."""

        # fleet_msg가 ``robot4:PATROLLING:85`` 같은 통신 문자열을 세 값으로
        # 나눈다. 형식이 틀렸거나 등록되지 않은 로봇이면 서버가 죽지 않게
        # 해당 메시지만 무시한다.
        try:
            robot_id, fleet_state, battery = fleet_msg.parse_status(msg.data)
            self.state.get_robot(robot_id)
        except (AttributeError, KeyError, TypeError, ValueError):
            return

        # 대소문자 차이를 없앤 뒤, Fleet와 웹 화면이 사용하는 상태 이름이
        # 다르므로 위의 변환표를 쓴다.
        fleet_state = fleet_state.upper()
        mapped = FLEET_STATE_MAP.get(fleet_state)
        if mapped is None:
            mapped = ("ERROR", f"알 수 없는 Fleet 상태: {fleet_state}")
        state_name, task = mapped
        # 잘못된 입력이 화면에 -10%나 120%로 표시되지 않도록 0~100으로 제한한다.
        battery_value = max(0.0, min(100.0, float(battery)))
        moving = fleet_state in {"PATROLLING", "RETURNING", "TRACKING", "HERDING"}

        with self._data_lock:
            # 저전압 상태 보고가 계속 와도 경고는 한 번만 기록한다. 임계값보다
            # 2% 이상 회복해야 기록 표시를 풀어 경계값 부근의 경고 깜빡임을 막는다.
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
        """Fleet 사건 문자열을 탐지 기록 또는 트랩 설치 기록으로 바꾼다."""

        # 사건 문자열에는 사건 이름과 지도 좌표만 있고 robot_id는 없다.
        try:
            name, map_x, map_y = fleet_msg.parse_event(msg.data)
        except (AttributeError, TypeError, ValueError):
            return
        name = name.lower()
        # 그래서 가장 최근에 움직였던 로봇을 사건의 표시 대상으로 선택한다.
        robot_id = self._event_robot_id()

        with self._data_lock:
            if name in {"rat_detected", "opening_confirmed"}:
                object_type = LIVE_RODENT if name == "rat_detected" else ENTRY_POINT

                # 탐지 저장과 함께 로봇 역할/상태 및 이벤트까지 일관되게
                # 갱신해야 하므로 직접 저장하지 않고 공통 서비스를 거친다.
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
        """robot_id가 없는 Fleet 사건을 어느 로봇 기록에 넣을지 결정한다.

        마지막으로 움직인 로봇이 없으면 설정에 등록된 첫 로봇을 사용한다.
        Fleet 사건 포맷에 robot_id가 추가되면 이 보정 로직은 제거할 수 있다.
        """

        if self._last_active_robot_id is not None:
            return self._last_active_robot_id
        return self.robots[0].robot_id

    def _on_detection(self, robot_id: str, source: str, msg: Any) -> None:
        """카메라의 Detection3DArray 안에 든 탐지를 하나씩 처리한다."""

        with self._data_lock:
            # 탐지 메시지 자체도 로봇이 통신 중이라는 증거이므로 heartbeat를
            # 갱신해 정상 로봇이 화면에서 OFFLINE으로 바뀌지 않게 한다.
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
                    # OAK-D의 살아 있는 쥐 탐지만 추적 유지 신호로 취급한다.
                    # monotonic 시간은 시스템 시계가 바뀌어도 경과 시간 계산이
                    # 거꾸로 흐르지 않아 timeout 측정에 적합하다.
                    self._last_live_rodent_at[robot_id] = time.monotonic()
                    self._target_lost_reported.discard(robot_id)

    def _to_detection_data(
        self,
        robot_id: str,
        source: str,
        array_msg: Any,
        detection_msg: Any,
    ) -> dict[str, Any] | None:
        """ROS 탐지 한 건을 관제 서버가 저장할 사전(dict)으로 변환한다."""

        # 분류 결과가 없다면 무엇을 탐지했는지 알 수 없으므로 저장하지 않는다.
        if not detection_msg.results:
            return None

        # vision_msgs 탐지는 후보를 여러 개 가질 수 있다. 현재 계약에서는
        # 첫 번째 결과가 가장 대표적인 분류 결과라고 보고 사용한다.
        hypothesis = detection_msg.results[0].hypothesis
        center = detection_msg.bbox.center.position
        # 3D 센서는 z를 전방 거리로 제공한다. z가 없는 2D 형태(z == 0)라면
        # x, y로 피타고라스 거리를 계산한다.
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
        """Odometry 메시지에서 위치, 방향, 속도를 계산해 저장한다."""

        # 현재 위치는 Odometry header 좌표계 기준이다. 실제 지도 표시는 후속 TF
        # 연동에서 map -> base_link 위치로 교체해야 한다.
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        header = getattr(msg, "header", None)
        position_frame = str(getattr(header, "frame_id", "odom") or "odom").strip("/")
        # ROS의 방향은 쿼터니언(x, y, z, w)으로 오지만 2D 지도는 수평 회전각
        # yaw만 필요하다. 아래는 쿼터니언을 yaw 라디안으로 바꾸는 공식이다.
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear
        # x축과 y축 속도를 합쳐 실제 평면 이동 속도의 크기를 구한다.
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
        """배터리 값을 백분율로 통일하고 저전압 경고를 관리한다."""

        percentage = msg.percentage
        # 센서가 값을 제공하지 않았거나 NaN(Not a Number)이면 무시한다.
        if percentage is None or math.isnan(percentage):
            return
        # BatteryState 구현에 따라 0.14 또는 14처럼 올 수 있어 둘 다 14%로
        # 해석하고, 최종 값은 항상 0~100 범위로 제한한다.
        value = percentage * 100 if percentage <= 1.0 else percentage
        value = round(max(0.0, min(100.0, value)), 3)
        with self._data_lock:
            below_threshold = value < self.low_battery_threshold
            not_reported = robot_id not in self._low_battery_reported
            is_new_low_battery = below_threshold and not_reported
            if is_new_low_battery:
                # record_low_battery가 경고 이벤트뿐 아니라 현재 배터리 값도
                # 함께 갱신하므로 아래 update_robot을 중복 호출하지 않는다.
                record_low_battery(self.state, robot_id, value)
                self._low_battery_reported.add(robot_id)
                return

            self.state.update_robot(robot_id, battery=value)
            if value >= self.low_battery_threshold + 2:
                self._low_battery_reported.discard(robot_id)

    def _on_camera_frame(self, robot_id: str, msg: Any) -> None:
        """압축 카메라 프레임의 원본 바이트를 최신 프레임 캐시에 넣는다."""

        # 이미 JPEG 등으로 압축된 데이터를 다시 인코딩하지 않는다. API는
        # CameraFrameStore에서 이 최신 바이트를 꺼내 브라우저에 전달한다.
        self.camera_frame_store.update(
            robot_id, bytes(msg.data), msg.format or "jpeg"
        )

    def _check_target_timeouts(self) -> None:
        """OAK-D 쥐 탐지가 끊긴 추적 로봇을 한 번만 대상 유실로 전환한다."""

        with self._data_lock:
            now = time.monotonic()
            # 순회 중 다른 콜백이 원본 dict를 수정해도 안전하도록 현재 항목을
            # tuple로 복사해서 검사한다.
            for robot_id, last_seen in tuple(self._last_live_rodent_at.items()):
                timed_out = now - last_seen >= self.target_loss_timeout_sec
                if not timed_out or robot_id in self._target_lost_reported:
                    continue
                robot = self.state.get_robot(robot_id)
                if robot["state"] != "TRACKING":
                    continue
                record_target_lost(self.state, robot_id)
                self._target_lost_reported.add(robot_id)
