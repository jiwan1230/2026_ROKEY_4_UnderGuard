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

# 전체 임무 상태는 개별 로봇 state 중 가장 우선순위가 높은 것을 그대로 반영한다.
# ERROR > TARGET_LOST > TRACKING > (확인 중 이동) > RETURNING 순이며, 해당하는
# 로봇이 하나도 없으면 IDLE(대기 중)로 떨어진다. BRD상 Under-Guard는 설치류를
# 직접 포획·트랩 설치하지 않으므로 라벨은 화면(dashboard.js)에서 "확인/대응"
# 중심 문구로 옮긴다.
_MISSION_STATUS_PRIORITY = (
    ("ERROR", {"ERROR"}),
    ("TARGET_LOST", {"TARGET_LOST"}),
    ("TRACKING", {"TRACKING"}),
    ("VERIFYING", {"SEARCHING", "APPROACHING", "NAVIGATING", "INSTALLING_TRAP"}),
    ("RETURNING", {"RETURNING"}),
)


def _derive_mission_status(states: list[str]) -> str:
    """개별 로봇 state 중 가장 우선순위가 높은 것을 전체 임무 상태로 반환한다.

    입력: 현재 스냅샷에 포함된 로봇들의 state 목록이다.
    출력: 화면에 표시할 전체 임무 상태 코드다. 해당하는 활성 상태가 없으면
    ``IDLE``이다(모든 로봇이 대기·오프라인·임무 완료 상태인 경우 포함).
    사용: ``StateManager.snapshot()``에서 매 호출마다 다시 계산해, 로봇 카드와
    전체 임무 배너가 서로 다른 이야기를 하지 않게 한다.
    """

    for status, member_states in _MISSION_STATUS_PRIORITY:
        if any(state in member_states for state in states):
            return status
    return "IDLE"

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
    ):
        self._lock = threading.RLock()
        self._offline_timeout_sec = offline_timeout_sec
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
        self._next_detection_id = 1
        self._next_trap_id = 1
        # "status"는 저장하지 않는다 — snapshot()이 매번 로봇 state로부터
        # 다시 계산해 덮어쓰므로, 여기 시드 값을 두면 절대 관측되지 않는
        # 죽은 값만 남는다(_derive_mission_status 참고).
        self._mission = {
            "role_assignment_status": "WAITING",
            "tracker_robot_id": None,
            "support_robot_ids": [],
        }

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
                "id": self._next_detection_id,
                "timestamp": time.time(),
                "status": "UNREVIEWED",
                **detection,
            }
            self._next_detection_id += 1
            self._detections.append(item)
            self._detections = self._detections[-100:]
            return copy.deepcopy(item)

    def add_trap(self, trap: dict[str, Any]) -> dict[str, Any]:
        """설치된 트랩 위치를 최근 목록에 추가하고 복사본을 반환한다."""

        with self._lock:
            item = {
                "id": self._next_trap_id,
                "timestamp": time.time(),
                "status": "INSTALLED",
                **trap,
            }
            self._next_trap_id += 1
            self._traps.append(item)
            self._traps = self._traps[-100:]
            return copy.deepcopy(item)

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
                        # BRD상 로봇은 트랩을 직접 설치하지 않고 침입구·트랩
                        # 상태를 확인해 관리자·방제업체 대응을 지원한다.
                        robot.current_task = "침입구·트랩 상태 확인"

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
            mission["status"] = _derive_mission_status(
                [robot["state"] for robot in robots]
            )
            online = sum(robot["connection"] == "ONLINE" for robot in robots)
            return {
                "server_time": time.time(),
                "summary": {
                    "robots_online": online,
                    "robots_total": len(robots),
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
