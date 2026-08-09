"""여러 로봇 상태를 웹 대시보드용 스냅샷으로 집계한다."""

from __future__ import annotations

import copy
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Any

from .risk_signals import is_live_rodent

# 로봇이 가질 수 있는 상태들
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

# 로봇이 가질 수 있는 역할을 제한
VALID_ROLES = {"SCOUT", "RAT_TRACKER", "SURVEY_TRAP", "UNASSIGNED"}

# 여러 로봇 상태 중에서 UI에 어떤 상태를 대표로 보여줄지 결정하는 우선순위표
# 예를 들어 한 로봇이 TRACKING 중이어도 다른 로봇에서 ERROR가 발생하면 ERROR를 우선 표시
_MISSION_STATUS_PRIORITY = (
    ("ERROR", {"ERROR"}),
    ("TARGET_LOST", {"TARGET_LOST"}),
    ("TRACKING", {"TRACKING"}),
    ("VERIFYING", {"SEARCHING", "APPROACHING", "NAVIGATING", "INSTALLING_TRAP"}),
    ("RETURNING", {"RETURNING"}),
)

# states는 현재 여러 로봇의 상태들을 모아놓은 리스트
def _derive_mission_status(states: list[str]) -> str:

    for status, member_states in _MISSION_STATUS_PRIORITY:
        # 리스트의 여러 로봇 중에서 한 대라도 해당 상태인지 확인
        if any(state in member_states for state in states):
            return status
    return "IDLE"

# YOLO 등에서 전달받은 객체 종류가 쥐 관련 객체인지 확인해서, 
# 결과를 bool 값으로 반환하는 함수
def is_rat_object(object_type: str | None) -> bool:
    # True False 반환
    return is_live_rodent(object_type)


# 로봇의 위치를 저장
@dataclass
class Position:
    x: float = 0.0
    y: float = 0.0
    # 로봇이 바라보는 방향
    yaw: float = 0.0


@dataclass
class Target:
    object_type: str | None = None
    confidence: float | None = None
    distance: float | None = None
    map_x: float | None = None
    map_y: float | None = None
    # source가 필요한 가장 큰 이유는 데이터의 출처를 추적하기 위해서
    # 실제 카메라·ROS 데이터인지, 테스트용 Mock 데이터인지 등을 구분
    source: str | None = None


# 굉장히 중요
@dataclass
class RobotState:
    robot_id: str
    role: str
    connection: str = "OFFLINE"
    state: str = "OFFLINE"
    battery: float | None = None
    speed: float = 0.0
    # 화면에 보여줄 현재 작업 설명
    current_task: str = "대기"
    # 그 로봇만의 Position 객체를 새로 하나 만들어서 넣어줌
    position: Position = field(default_factory=Position)
    position_frame: str = "unknown"
    target: Target = field(default_factory=Target)
    nav_status: str = "UNKNOWN"
    camera_status: str = "UNKNOWN"
    slam_status: str = "UNKNOWN"
    # 중요
    # 이 로봇으로부터 마지막 데이터를 받은 시각.
    # Offline 판단에 핵심적으로 사용
    last_update: float = 0.0


# 웹의 최근 이벤트 타임라인 하나를 표현
@dataclass
class Event:
    id: int
    timestamp: float
    robot_id: str | None
    # 이벤트의 심각도
    severity: str
    event_type: str
    # 이벤트 내용을 사람이 읽을 수 있게 설명한 문장
    message: str


# 중요
# ROS/Mock 데이터를 하나의 웹 상태로 집계
# ROS callback·Mock thread·Flask 요청의 동시 접근을 Lock으로 보호
class StateManager:

    def __init__(
        self,
        robots: list[tuple[str, str]],
        # 중요
        # 로봇 데이터가 몇 초 동안 안 들어왔을 때 Offline으로 볼지
        offline_timeout_sec: float = 3.0,
    ):
        # 중요 (가장 메인 코드에서 다시 나옴)
        # 공유 상태 접근용 Lock 생성
        # threading → 여러 작업(스레드)을 다루는 파이썬 기본 모듈
        # RLock()   → 공유 데이터를 안전하게 수정하기 위한 잠금장치
        self._lock = threading.RLock()
        self._offline_timeout_sec = offline_timeout_sec
        self._initial_robots = tuple(robots)
        # 실제 로봇/이벤트/탐지 데이터들을 초기 상태로 만듦
        self._reset_unlocked()

    # StateManager 내부에서 사용하는 함수
    # 전체 임무 상태를 별도로 저장하면 로봇 상태와 Mission 상태가 서로 달라지는 문제가 생길 수 있기 때문에,
    # snapshot을 생성할 때 현재 로봇 상태를 기준으로 다시 계산하도록
    def _reset_unlocked(self) -> None:
        self._robots = {
            robot_id: RobotState(robot_id=robot_id, role=role)
            for robot_id, role in self._initial_robots
        }
        # 이벤트 목록 초기화
        self._events: list[Event] = []
        # 탐지 목록 초기화
        self._detections: list[dict[str, Any]] = []
        self._traps: list[dict[str, Any]] = []
        self._next_event_id = 1
        self._next_detection_id = 1
        self._next_trap_id = 1
        # 전체 임무 정보를 저장
        self._mission = {
            "role_assignment_status": "WAITING",
            "tracker_robot_id": None,
            "support_robot_ids": [],
        }

    # 가장 중요
    def update_robot(self, robot_id: str, **changes: Any) -> None:

        # 지금부터 StateManager 데이터를 수정하므로 Lock을 획득
        # 두 작업이 있을 때, 동시에 StateManager의 같은 값을 바꾸면 데이터가 꼬일 수 있음
        with self._lock:
            # 전달받은 ID의 로봇이 실제 등록된 로봇인지 확인
            robot = self._require_robot(robot_id)
            now = time.time()
            # pap -> position, target은 단순 문자열이나 숫자가 아니라
            # 객체이고, 그 안에 여러 속성이 있으므로 별도로 꺼내서 처리
            position = changes.pop("position", None)
            target = changes.pop("target", None)
            state = changes.get("state")
            if state is not None and state not in VALID_STATES:
                # ROS나 Mock에서 예상하지 않은 state 값이 들어와 UI 상태가 깨지는 것을 방지
                raise ValueError(f"지원하지 않는 state: {state}")
            role = changes.get("role")
            if role is not None and role not in VALID_ROLES:
                raise ValueError(f"지원하지 않는 role: {role}")
            # Position 값은 dictionary 형태로 들어오고, x·y·yaw 필드가 존재하는지 확인한 뒤 float으로 변환
            if position:
                for key, value in position.items():
                    # 전달받은 위치 항목이 Position 객체에 실제로 존재하는지 확인
                    if hasattr(robot.position, key):
                        # 존재한다면 값을 float으로 변환해서 해당 위치값에 저장
                        setattr(robot.position, key, float(value))
            if target:
                for key, value in target.items():
                    if hasattr(robot.target, key):
                        setattr(robot.target, key, value)
            # position과 target을 제외한 값들을 처리          
            for key, value in changes.items():
                if not hasattr(robot, key):
                    raise KeyError(f"RobotState에 없는 필드: {key}")
                # 정상적인 필드는 실제 RobotState에 저장
                setattr(robot, key, value)
            # 중요
            # Offline 처리와 직접 연결되는 중요 코드
            # 마지막으로 통신된 시간 = 현재 시간으로 변경함
            robot.last_update = now
            robot.connection = "ONLINE"
            # 통신이 다시 들어왔는데 기존 상태가 아직 OFFLINE이면 -> IDLE 로 복구
            if robot.state == "OFFLINE":
                robot.state = "IDLE"

    # 특정 로봇으로부터 통신이 들어왔다는 것을 기록하는 함수
    # 로봇의 온라인·오프라인 상태를 판단하는 데 사용
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
