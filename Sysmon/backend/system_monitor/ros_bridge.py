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

import json
import math
import sqlite3
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
from .history_store import HistoryStore
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
        history_store: HistoryStore | None = None,
        detection_record_interval_sec: float = 1.0,
        trail_record_interval_sec: float = 0.5,
    ) -> None:
        # 외부에서 주입받은 저장소와 설정을 보관한다. 이렇게 하면 실제 ROS가
        # 없는 테스트에서도 가짜 메시지를 콜백에 직접 넣어 검증할 수 있다.
        self.state = state
        self.robots = robots
        self.target_loss_timeout_sec = target_loss_timeout_sec
        self.low_battery_threshold = low_battery_threshold
        self.interface = interface or RosInterfaceConfig()
        self.camera_frame_store = camera_frame_store or CameraFrameStore()
        self.history_store = history_store
        self.detection_record_interval_sec = max(
            0.0, detection_record_interval_sec
        )
        self.trail_record_interval_sec = max(0.0, trail_record_interval_sec)
        self._thread: threading.Thread | None = None
        self._node = None
        # db_node의 기록 조회 서비스 클라이언트 — spin 스레드에서 만들고
        # Flask 요청 스레드에서 호출한다(query_db 참고).
        self._db_client = None
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
        self._last_detection_recorded_at: dict[tuple[str, str], float] = {}
        self._last_trail_recorded_at: dict[str, float] = {}
        # /fleet/detection에서 받아 둔 최근 값 — object_type -> (robot_id,
        # confidence, 수신시각). robot_id 추측을 실제 값으로 대체하는 데 쓴다.
        self._last_detection_meta: dict[str, tuple[str, float, float]] = {}

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

    # 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 #
    def _spin(self) -> None:
        """ROS 노드를 만들어서 실행하고, 종료 요청이 올 때까지 메시지를 처리한다."""

        # rclpy.init -> Node 생성 -> rclpy.spin 순서가 ROS 2 노드의 기본
        # ROS 2 통신 기능을 사용하기 위해 ROS 2를 초기화
        rclpy.init(args=None)
        # ROS 2의 노드를 만들기 위한 Node 클래스를 가져옴
        from rclpy.node import Node

        # 현재 객체인 self를 bridge라는 이름으로 하나 더 참조
        bridge = self

        # 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 #
        # ROS 2에서 Fleet 상태 메시지를 받기 위한 노드와 구독자를 만드는 부분
        class MonitorNode(Node):
            def __init__(self) -> None:
                # 노드 이름을 system_monitor_bridge로 설정
                super().__init__("system_monitor_bridge")
                self.create_subscription(
                    String,
                    # 로봇의 현재 상태와 배터리 정보를 받는 토픽
                    bridge.interface.fleet_status_topic,
                    # 문자열을 로봇 ID / 상태 / 배터리로 분리하고, Fleet 상태를 웹 화면에서 사용하는 상태로 변환
                    # 이후 로봇의 상태, 배터리, 현재 작업, 이동 여부를 StateManager에 반영
                    bridge._on_fleet_status,
                    10,
                )
                self.create_subscription(
                    String,
                    # Fleet에서 발생한 사건/이벤트 정보를 받는 토픽
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
                for robot in bridge.robots:
                    # 현재 로봇의 ID 저장
                    rid = robot.robot_id
                    # 기본 인자로 rid를 고정해 모든 lambda가 마지막 로봇을
                    # 참조하는 파이썬 late-binding 문제를 방지한다.
                    if Detection3DArray is not None:
                        self.create_subscription(
                            Detection3DArray,
                            # 웹캠의 객체 탐지 결과를 받는 토픽
                            robot.topic(bridge.interface.webcam_detections_topic),
                            # 탐지 메시지가 오면 어떤 로봇의 웹캠 탐지인지 구분해서 _on_detection()으로 전달
                            lambda msg, robot_id=rid: bridge._on_detection(
                                robot_id, "WEBCAM", msg
                            ),
                            10,
                        )
                        self.create_subscription(
                            Detection3DArray,
                            # OAK-D의 객체 탐지 결과를 받는 토픽
                            robot.topic(bridge.interface.oakd_detections_topic),
                            lambda msg, robot_id=rid: bridge._on_detection(
                                robot_id, "OAK-D", msg
                            ),
                            10,
                        )
                    if Odometry is not None:
                        self.create_subscription(
                            Odometry,
                            # 로봇의 위치·방향·속도 정보를 받는 토픽
                            robot.topic(bridge.interface.odometry_topic),
                            lambda msg, robot_id=rid: bridge._on_odom(robot_id, msg),
                            10,
                        )
                    # BatteryState는 ROS 2의 sensor_msgs 패키지에서 제공하는 메시지 타입
                    if BatteryState is not None:
                        self.create_subscription(
                            BatteryState,
                            # 로봇의 배터리 정보를 받는 토픽
                            robot.topic(bridge.interface.battery_topic),
                            lambda msg, robot_id=rid: bridge._on_battery(robot_id, msg),
                            10,
                        )
                    # CompressedImage는 JPEG 같은 형태로 압축된 카메라 이미지 데이터를 전달할 때 사용하는 ROS 메시지 타입
                    if CompressedImage is not None:
                        self.create_subscription(
                            CompressedImage,
                            # 카메라 영상 프레임을 받는 토픽
                            robot.topic(bridge.interface.camera_frame_topic),
                            lambda msg, robot_id=rid: bridge._on_camera_frame(
                                robot_id, msg
                            ),
                            # 카메라 데이터는 크고 최신 프레임이 중요하기 때문에,
                            # 일반 큐 크기 대신 ROS 2의 센서 데이터용 QoS인 qos_profile_sensor_data를 사용
                            qos_profile_sensor_data,
                        )
                # 0.5초마다 _check_target_timeouts()를 실행해서,
                # 일정 시간 동안 쥐가 다시 탐지되지 않으면 대상 유실 상태인지 확인
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
        # 상세 탐지 메타데이터가 있으면 실제 탐지 로봇과 confidence를 쓰고,
        # 없으면 가장 최근에 움직인 로봇으로 안전하게 되돌아간다.
        robot_id, confidence = self._detection_meta(name)

        with self._data_lock:
            if name in {"rat_detected", "opening_confirmed"}:
                object_type = LIVE_RODENT if name == "rat_detected" else ENTRY_POINT

                # 탐지 저장과 함께 로봇 역할/상태 및 이벤트까지 일관되게
                # 갱신해야 하므로 직접 저장하지 않고 공통 서비스를 거친다.
                item = process_detection(
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
                self._record_detection_history(item)
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
        """카메라의 Detection3DArray 안에 든 탐지를 하나씩 처리한다."""

        # 여러 콜백이 동시에 데이터를 바꾸지 않도록 안전하게 잠금
        with self._data_lock:
            # 탐지 메시지가 들어왔으므로 이 로봇이 현재 통신 중이라고 표시
            self.state.mark_heartbeat(robot_id)
            # 한 메시지 안에 들어있는 여러 객체 탐지 결과를 하나씩 꺼내서 처리
            for detection_msg in msg.detections:

                # 중요 #
                # 탐지 결과를 robot_id, 객체 종류, 신뢰도, 거리, 좌표 같은 관제용 데이터 형태(dict)로 변환
                data = self._to_detection_data(robot_id, source, msg, detection_msg)
                if data is None:
                    continue

                # 중요 #
                # detection_service의 process_detection 함수를 실행하여,
                # 변환된 탐지 데이터를 실제 관제 시스템 상태와 이벤트에 반영하기 위해 결과를 변수에 저장
                item = process_detection(
                    # 현재 로봇 상태를 관리하는 객체
                    self.state,
                    # 앞에서 변환한 탐지 데이터
                    data,
                    event_message=f"{source}에서 {data['object_type']}를 탐지했습니다.",
                    fallback_state=(
                        "APPROACHING" if source != "OAK-D" else "SEARCHING"
                    ),
                    # 현재 작업을 "쥐 탐지 확인 중" 같은 형태로 설정
                    fallback_task=f"{data['object_type']} 탐지 확인 중",
                    camera_status="NORMAL",
                )
                self._record_detection_history(item)

                # 중요 #
                # 탐지 출처가 OAK-D이고, 탐지한 객체가 살아있는 쥐인지 확인
                if source.upper() == "OAK-D" and is_live_rodent(
                    item["object_type"]
                ):
                    # 해당 로봇이 쥐를 마지막으로 본 시간을 현재 시간으로 저장
                    self._last_live_rodent_at[robot_id] = time.monotonic()
                    # 이전에 이 로봇이 대상 유실로 기록되어 있었다면 그 표시를 제거
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

    # 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 #
    def _on_odom(self, robot_id: str, msg: Any) -> None:
        """
        Odometry 메시지에서 로봇의 위치, 방향, 이동 속도를 계산해서 StateManager에 갱신.
        이 값이 관제 화면에서 로봇 위치와 이동 상태를 표시하는 데 사용.
        """

        # 로봇의 현재 위치(x, y 등)
        p = msg.pose.pose.position
        # 로봇의 방향 정보(쿼터니언)
        q = msg.pose.pose.orientation
        header = getattr(msg, "header", None)
        # 이 위치가 어느 좌표계 기준인지 확인
        position_frame = str(getattr(header, "frame_id", "odom") or "odom").strip("/")
        # ROS의 방향은 쿼터니언(x, y, z, w)으로 오지만 2D 지도는 수평 회전각
        # yaw만 필요하다. 아래는 쿼터니언을 yaw 라디안으로 바꾸는 공식이다.
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        # 로봇의 x, y축 이동 속도
        v = msg.twist.twist.linear
        # x축과 y축 속도를 합쳐 실제 평면 이동 속도의 크기를 계산
        speed = math.hypot(v.x, v.y)
        self.state.update_robot(
            robot_id,
            position={"x": p.x, "y": p.y, "yaw": yaw},
            position_frame=position_frame,
            speed=speed,
            nav_status="MOVING" if speed > 0.02 else "STOPPED",
            slam_status="NORMAL",
        )
        self._record_trail_history(
            robot_id,
            position_frame=position_frame,
            map_x=float(p.x),
            map_y=float(p.y),
        )

    def _record_detection_history(self, detection: dict[str, Any]) -> None:
        """실시간 탐지를 영속 이력에 저장하되 고주파 중복 기록을 제한한다."""

        if self.history_store is None:
            return
        robot_id = str(detection.get("robot_id") or "")
        object_type = str(detection.get("object_type") or "")
        key = (robot_id, object_type)
        now = time.monotonic()
        last = self._last_detection_recorded_at.get(key)
        if last is not None and now - last < self.detection_record_interval_sec:
            return

        frame = self.camera_frame_store.get(robot_id)
        image_bytes = frame.content if frame is not None else None
        image_ext = (
            "png" if frame is not None and "png" in frame.format.lower() else "jpg"
        )
        try:
            self.history_store.record_detection(
                robot_id=robot_id or None,
                object_type=object_type or None,
                map_x=detection.get("map_x"),
                map_y=detection.get("map_y"),
                confidence=detection.get("confidence"),
                timestamp=detection.get("timestamp"),
                image_bytes=image_bytes,
                image_ext=image_ext,
            )
        except (OSError, sqlite3.Error, ValueError) as exc:
            self._log_history_error(f"탐지 기록 저장 실패: {exc}")
            return
        self._last_detection_recorded_at[key] = now

    def _record_trail_history(
        self,
        robot_id: str,
        *,
        position_frame: str,
        map_x: float,
        map_y: float,
    ) -> None:
        """map 좌표의 odom 위치를 제한된 주기로 이동 경로 DB에 저장한다."""

        if self.history_store is None:
            return
        expected_frame = self.interface.map_frame.strip("/") or "map"
        if position_frame != expected_frame:
            return
        now = time.monotonic()
        last = self._last_trail_recorded_at.get(robot_id)
        if last is not None and now - last < self.trail_record_interval_sec:
            return
        try:
            self.history_store.record_trail_point(
                robot_id=robot_id,
                map_x=map_x,
                map_y=map_y,
            )
        except (OSError, sqlite3.Error, ValueError) as exc:
            self._log_history_error(f"이동 경로 저장 실패: {exc}")
            return
        self._last_trail_recorded_at[robot_id] = now

    def _log_history_error(self, message: str) -> None:
        """기록 실패가 실시간 관제를 중단하지 않도록 ROS 로그만 남긴다."""

        if self._node is not None:
            self._node.get_logger().error(message)

    # 중요 #
    def _on_battery(self, robot_id: str, msg: Any) -> None:
        """
        배터리 메시지를 받으면 값을 0~100% 형태로 정리해서 로봇 상태에 반영하고,
        임계값보다 낮아지면 저전압 경고를 기록
        """

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

    # 중요 #
    def _on_camera_frame(self, robot_id: str, msg: Any) -> None:
        """
        카메라에서 받은 압축 이미지 데이터를 최신 프레임으로 저장하고,
        이후 웹 관제 화면에서 사용할 수 있도록 함
        """

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
