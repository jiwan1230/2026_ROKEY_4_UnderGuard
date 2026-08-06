"""여러 로봇 상태를 웹 대시보드용 스냅샷으로 집계한다."""

from __future__ import annotations

import copy
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Any

from .risk_signals import is_live_rodent

VALID_STATES = {
    "IDLE",
    "SEARCHING",
    "APPROACHING",
    "TRACKING",
    "TARGET_LOST",
    "NAVIGATING",
    "INSTALLING_TRAP",
    "RETURNING",
    "COMPLETED",
    "PAUSED",
    "ERROR",
    "OFFLINE",
}

VALID_ROLES = {"SCOUT", "RAT_TRACKER", "SURVEY_TRAP", "UNASSIGNED"}

COMMAND_TRANSITIONS = {
    "START_SCOUTING": ("SEARCHING", "쥐 공동 탐색"),
    "START_TRACKING": ("TRACKING", "쥐 추적"),
    "START_SEARCH": ("SEARCHING", "쥐구멍 탐색"),
    "PAUSE": ("PAUSED", "임무 일시정지"),
    "RESUME": ("NAVIGATING", "임무 재개"),
    "RETURN_HOME": ("RETURNING", "시작 위치 복귀"),
    "INSTALL_TRAP": ("INSTALLING_TRAP", "쥐덫 설치"),
    "STOP": ("IDLE", "임무 중단"),
}

COMMAND_REQUIRED_ROLES = {
    "START_SCOUTING": "SCOUT",
    "START_TRACKING": "RAT_TRACKER",
    "START_SEARCH": "SURVEY_TRAP",
    "INSTALL_TRAP": "SURVEY_TRAP",
}


def is_rat_object(object_type: str | None) -> bool:
    """객체 라벨이 현재 데모에서 쥐로 취급되는지 반환한다."""

    return is_live_rodent(object_type)


@dataclass
class Position:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class Target:
    object_type: str | None = None
    confidence: float | None = None
    distance: float | None = None
    map_x: float | None = None
    map_y: float | None = None
    source: str | None = None


@dataclass
class RobotState:
    robot_id: str
    role: str
    connection: str = "OFFLINE"
    state: str = "OFFLINE"
    battery: float | None = None
    speed: float = 0.0
    current_task: str = "대기"
    position: Position = field(default_factory=Position)
    position_frame: str = "unknown"
    target: Target = field(default_factory=Target)
    nav_status: str = "UNKNOWN"
    camera_status: str = "UNKNOWN"
    slam_status: str = "UNKNOWN"
    last_update: float = 0.0


@dataclass
class Event:
    id: int
    timestamp: float
    robot_id: str | None
    severity: str
    event_type: str
    message: str


class StateManager:
    """ROS/Mock 데이터를 웹용 단일 상태로 집계한다.

    모든 공개 변경/조회는 재진입 잠금으로 보호되어 ROS 콜백, Mock 스레드,
    Flask 요청이 같은 상태를 동시에 다뤄도 중간 상태가 노출되지 않는다.
    """

    def __init__(
        self,
        robots: list[tuple[str, str]],
        offline_timeout_sec: float = 3.0,
        low_battery_threshold: float = 15.0,
    ):
        self._lock = threading.RLock()
        self._offline_timeout_sec = offline_timeout_sec
        self._low_battery_threshold = low_battery_threshold
        self._initial_robots = tuple(robots)
        self._reset_unlocked()

    def _reset_unlocked(self) -> None:
        self._robots = {
            robot_id: RobotState(robot_id=robot_id, role=role)
            for robot_id, role in self._initial_robots
        }
        self._events: list[Event] = []
        self._detections: list[dict[str, Any]] = []
        self._traps: list[dict[str, Any]] = []
        self._next_event_id = 1
        self._mission = {
            "status": "READY",
            "progress": 0,
            "elapsed_sec": 0,
            "started_at": None,
            "role_assignment_status": "WAITING",
            "tracker_robot_id": None,
            "support_robot_ids": [],
        }

    def reset(self) -> None:
        """로봇·임무·탐지·사건·트랩을 생성 직후 상태로 되돌린다.

        입력과 출력은 없다. 관리자용 Mock 초기화에서 DB 삭제 후 사용하며,
        잠금 안에서 전체 상태를 교체해 폴링 요청이 중간 상태를 보지 않게 한다.
        """

        with self._lock:
            self._reset_unlocked()

    def clear_operational_history(self) -> None:
        """현재 로봇·임무 상태는 유지하고 수집 이력만 비운다.

        입력과 출력은 없다. ROS 운영 데이터 초기화에서 탐지·사건·트랩의
        메모리 사본을 DB와 함께 비우되 마지막 ROS 위치와 연결은 보존한다.
        """

        with self._lock:
            self._events.clear()
            self._detections.clear()
            self._traps.clear()
            self._next_event_id = 1

    def update_robot(self, robot_id: str, **changes: Any) -> None:
        """로봇 상태 일부를 갱신하고 통신 시각을 기록한다.

        입력: 로봇 ID와 상태 변경값이다. 위치/대상은 딕셔너리다.
        출력: 없음. 잘못된 상태·역할·필드는 예외로 거부한다.
        사용: Mock 갱신 및 ROS 콜백에서 수신한 최신 값만 전달한다.
        """

        with self._lock:
            robot = self._require_robot(robot_id)
            now = time.time()
            position = changes.pop("position", None)
            target = changes.pop("target", None)
            state = changes.get("state")
            if state is not None and state not in VALID_STATES:
                raise ValueError(f"지원하지 않는 state: {state}")
            role = changes.get("role")
            if role is not None and role not in VALID_ROLES:
                raise ValueError(f"지원하지 않는 role: {role}")
            if position:
                for key, value in position.items():
                    if hasattr(robot.position, key):
                        setattr(robot.position, key, float(value))
            if target:
                for key, value in target.items():
                    if hasattr(robot.target, key):
                        setattr(robot.target, key, value)
            for key, value in changes.items():
                if not hasattr(robot, key):
                    raise KeyError(f"RobotState에 없는 필드: {key}")
                setattr(robot, key, value)
            robot.last_update = now
            robot.connection = "ONLINE"
            if robot.state == "OFFLINE":
                robot.state = "IDLE"

    def mark_heartbeat(self, robot_id: str) -> None:
        self.update_robot(robot_id)

    def add_event(
        self,
        message: str,
        *,
        robot_id: str | None = None,
        severity: str = "INFO",
        event_type: str = "SYSTEM",
    ) -> dict[str, Any]:
        """타임라인 사건을 추가하고 직렬화 가능한 복사본을 반환한다."""

        with self._lock:
            event = Event(
                id=self._next_event_id,
                timestamp=time.time(),
                robot_id=robot_id,
                severity=severity.upper(),
                event_type=event_type,
                message=message,
            )
            self._next_event_id += 1
            self._events.append(event)
            self._events = self._events[-200:]
            return asdict(event)

    def add_detection(self, detection: dict[str, Any]) -> dict[str, Any]:
        """최근 탐지 목록에 항목을 추가하고 안전한 복사본을 반환한다."""

        with self._lock:
            item = {
                "timestamp": time.time(),
                "status": "UNREVIEWED",
                **detection,
            }
            self._detections.append(item)
            self._detections = self._detections[-100:]
            return copy.deepcopy(item)

    def add_trap(self, trap: dict[str, Any]) -> dict[str, Any]:
        """설치된 트랩 위치를 최근 목록에 추가하고 복사본을 반환한다."""

        with self._lock:
            item = {"timestamp": time.time(), "status": "INSTALLED", **trap}
            self._traps.append(item)
            self._traps = self._traps[-100:]
            return copy.deepcopy(item)

    def set_mission(self, **changes: Any) -> None:
        with self._lock:
            self._mission.update(changes)

    def mark_mission_started(self) -> None:
        """첫 활동 시 임무 시작 시각과 실행 상태를 한 번만 기록한다."""

        with self._lock:
            if self._mission["started_at"] is None:
                self._mission["started_at"] = time.time()
            self._mission["status"] = "RUNNING"

    def get_robot(self, robot_id: str) -> dict[str, Any]:
        """외부 컴포넌트가 안전하게 읽도록 로봇 상태 복사본을 반환한다."""
        with self._lock:
            return asdict(self._require_robot(robot_id))

    def assign_roles_from_rat_detection(self, detector_robot_id: str) -> dict[str, Any] | None:
        """최초 탐지 로봇은 추적, 나머지는 지원 역할로 배정한다.

        동시 탐지에서도 첫 호출만 역할을 확정하며, 이후 호출은 기존
        배정을 유지하기 위해 ``None``을 반환한다.

        입력: 최초로 쥐를 탐지한 등록 로봇 ID다.
        출력: 추적/지원 로봇 배정 결과이며, 이미 배정됐으면 ``None``이다.
        사용: 쥐 계열 탐지를 처리하는 ``process_detection``에서 호출한다.
        """
        with self._lock:
            self._require_robot(detector_robot_id)
            if self._mission["role_assignment_status"] == "ASSIGNED":
                return None

            support_robot_ids = [
                robot_id for robot_id in self._robots
                if robot_id != detector_robot_id
            ]
            for robot_id, robot in self._robots.items():
                if robot_id == detector_robot_id:
                    robot.role = "RAT_TRACKER"
                    if robot.connection == "ONLINE":
                        robot.state = "TRACKING"
                        robot.current_task = "최초 쥐 발견 · 추적 전환"
                else:
                    robot.role = "SURVEY_TRAP"
                    if robot.connection == "ONLINE":
                        robot.state = "SEARCHING"
                        robot.current_task = "쥐구멍 탐색 및 트랩 설치"

            self._mission.update(
                role_assignment_status="ASSIGNED",
                tracker_robot_id=detector_robot_id,
                support_robot_ids=support_robot_ids,
            )
            return {
                "tracker_robot_id": detector_robot_id,
                "support_robot_ids": support_robot_ids,
                "roles": {
                    robot_id: robot.role for robot_id, robot in self._robots.items()
                },
            }

    def apply_command(self, robot_id: str, command: str) -> dict[str, Any]:
        """Mock 이동 명령을 상태 전이로 적용한다.

        입력: 등록 로봇 ID와 ``START_SCOUTING`` 같은 허용 명령 문자열이다.
        출력: 대시보드와 DB에 기록할 명령 사건 딕셔너리다.
        사용: Mock 모드의 ``POST /api/commands``에서만 호출한다.
        """

        with self._lock:
            state, task = self.validate_command(robot_id, command)
            self.update_robot(robot_id, state=state, current_task=task)
            return self.add_event(
                f"{task} 명령이 요청되었습니다.",
                robot_id=robot_id,
                event_type="COMMAND",
            )

    def validate_command(self, robot_id: str, command: str) -> tuple[str, str]:
        """모드와 무관하게 로봇 ID·명령·역할 조합을 검증한다.

        입력: 명령 대상 로봇 ID와 명령 이름이다.
        출력: 명령이 의미하는 다음 상태와 작업 문구다.
        사용: Mock은 적용 전, ROS는 송신 가능 여부 응답 전에 호출한다.
        """

        with self._lock:
            if command not in COMMAND_TRANSITIONS:
                raise ValueError(f"지원하지 않는 명령: {command}")
            robot = self._require_robot(robot_id)
            required_role = COMMAND_REQUIRED_ROLES.get(command)
            if required_role and robot.role != required_role:
                raise ValueError(
                    f"{command} 명령은 {required_role} 역할에서만 사용할 수 있습니다."
                )
            return COMMAND_TRANSITIONS[command]

    def snapshot(self) -> dict[str, Any]:
        """현재 시스템 전체 상태를 대시보드 응답 형태로 반환한다.

        입력: 없음. 조회 시 마지막 갱신 시각으로 Offline 여부를 재계산한다.
        출력: 요약, 임무, 로봇, 사건, 최근 탐지를 포함한 독립 복사본이다.
        사용: ``GET /api/snapshot`` 폴링 응답으로 전달한다.
        """

        with self._lock:
            self._refresh_connections()
            robots = [asdict(robot) for robot in self._robots.values()]
            mission = copy.deepcopy(self._mission)
            if mission["started_at"] is not None and mission["status"] == "RUNNING":
                mission["elapsed_sec"] = int(time.time() - mission["started_at"])
            online = sum(robot["connection"] == "ONLINE" for robot in robots)
            active_alerts = sum(
                robot["connection"] == "OFFLINE"
                or robot["state"] in {"TARGET_LOST", "ERROR"}
                or (
                    robot["battery"] is not None
                    and robot["battery"] < self._low_battery_threshold
                )
                for robot in robots
            )
            return {
                "server_time": time.time(),
                "summary": {
                    "robots_online": online,
                    "robots_total": len(robots),
                    "active_alerts": active_alerts,
                    "detections": len(self._detections),
                },
                "mission": mission,
                "robots": robots,
                "events": [asdict(event) for event in reversed(self._events[-30:])],
                "detections": list(reversed(copy.deepcopy(self._detections[-30:]))),
                "traps": list(reversed(copy.deepcopy(self._traps[-30:]))),
            }

    def _refresh_connections(self) -> None:
        now = time.time()
        for robot in self._robots.values():
            if robot.last_update <= 0 or now - robot.last_update > self._offline_timeout_sec:
                robot.connection = "OFFLINE"
                robot.state = "OFFLINE"
                robot.speed = 0.0

    def _require_robot(self, robot_id: str) -> RobotState:
        try:
            return self._robots[robot_id]
        except KeyError as exc:
            raise KeyError(f"등록되지 않은 robot_id: {robot_id}") from exc
