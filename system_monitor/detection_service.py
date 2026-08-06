"""Mock과 ROS 탐지를 동일한 상태·DB·사건 처리 순서로 통합한다.

주의: 이 파일은 탐지가 "새로 들어왔을 때" 저장하는 쓰기(write) 경로만
담당한다. 탐지 화면(detections 페이지)의 검색·조회는 이 파일을 거치지
않고 ``app.py``의 ``/api/detections`` 라우트가 ``database.py``의
``search_detections()``를 직접 호출한다. 즉 데이터 흐름은
``mock_manager.py``/``ros_bridge.py`` → (여기) ``process_detection`` →
``database.py`` (저장) 이고,
``detections.js`` → ``app.py`` → ``database.py`` (조회) 는 별도 경로다.
"""

from __future__ import annotations

from typing import Any

from .database import Database
from .risk_signals import normalize_risk_signal
from .state_manager import StateManager, is_rat_object

TARGET_FIELDS = ("object_type", "confidence", "distance", "map_x", "map_y", "source")


def process_detection(
    state: StateManager,
    db: Database,
    detection: dict[str, Any],
    *,
    event_message: str,
    fallback_state: str | None = None,
    fallback_task: str | None = None,
    camera_status: str | None = None,
) -> dict[str, Any]:
    """탐지 한 건을 영속화하고 로봇 상태와 사건 타임라인에 반영한다.

    입력: 상태/DB 객체, ``robot_id``·``object_type``을 포함한 탐지와 표시 문구다.
    출력: DB ID와 수신 시각이 추가된 실시간 탐지 딕셔너리다.
    사용: Mock과 ROS 콜백이 호출해 처리 규칙을 공유한다.
    """

    data = dict(detection)
    robot_id = str(data["robot_id"])
    object_type = normalize_risk_signal(data["object_type"])
    data["object_type"] = object_type

    # DB에 쓰기 전에 robot_id를 확인해 부분 저장된 탐지가 생기지 않게 한다.
    state.get_robot(robot_id)

    data["id"] = db.insert_detection(data)
    item = state.add_detection(data)
    state.mark_mission_started()

    # 최초 쥐 탐지에서 역할을 확정하고 DB와 실시간 상태를 함께 갱신한다.
    assignment = (
        state.assign_roles_from_rat_detection(robot_id)
        if is_rat_object(object_type)
        else None
    )

    robot = state.get_robot(robot_id)
    _update_robot_target(
        state,
        robot_id,
        robot["role"],
        data,
        fallback_state=fallback_state,
        fallback_task=fallback_task,
        camera_status=camera_status,
    )

    if assignment:
        _record_role_assignment(state, db, robot_id, assignment)

    event = state.add_event(
        event_message,
        robot_id=robot_id,
        event_type="DETECTION",
    )
    db.insert_event(event)
    return item


def record_target_lost(
    state: StateManager,
    db: Database,
    robot_id: str,
) -> dict[str, Any]:
    """대상 유실을 적용하고 마지막 target을 유지한 채 사건을 저장한다."""

    state.update_robot(
        robot_id,
        state="TARGET_LOST",
        speed=0.0,
        nav_status="CANCELED",
        current_task="대상 재탐색 대기",
    )
    event = state.add_event(
        "추적 대상을 놓쳤습니다. 마지막 탐지 위치를 유지합니다.",
        robot_id=robot_id,
        severity="WARNING",
        event_type="TARGET_LOST",
    )
    db.insert_event(event)
    return event


def record_low_battery(
    state: StateManager,
    db: Database,
    robot_id: str,
    battery: float,
) -> dict[str, Any]:
    """배터리 값을 반영하고 저전압 경고를 상태와 DB에 함께 기록한다."""

    state.update_robot(robot_id, battery=battery)
    event = state.add_event(
        f"배터리가 부족합니다({battery:.1f}%). 복귀를 권장합니다.",
        robot_id=robot_id,
        severity="WARNING",
        event_type="LOW_BATTERY",
    )
    db.insert_event(event)
    return event


def record_trap_installed(
    state: StateManager,
    db: Database,
    robot_id: str,
    *,
    map_frame: str = "map",
) -> dict[str, Any]:
    """트랩 설치 완료 상태와 사건을 동일한 형식으로 기록한다."""

    robot = state.get_robot(robot_id)
    expected_frame = map_frame.strip("/") or "map"
    if robot["position_frame"] != expected_frame:
        raise ValueError(f"트랩 위치 저장에는 {expected_frame} frame 좌표가 필요합니다.")
    trap = {
        "robot_id": robot_id,
        "map_x": robot["position"]["x"],
        "map_y": robot["position"]["y"],
        "status": "INSTALLED",
    }
    trap["id"] = db.insert_trap(trap)
    state.add_trap(trap)
    state.update_robot(
        robot_id,
        state="COMPLETED",
        speed=0.0,
        current_task="쥐덫 설치 완료 · 다음 지시 대기",
    )
    event = state.add_event(
        "쥐덫 설치가 완료되었습니다.",
        robot_id=robot_id,
        event_type="TRAP_INSTALLED",
    )
    db.insert_event(event)
    return event


def _update_robot_target(
    state: StateManager,
    robot_id: str,
    robot_role: str,
    detection: dict[str, Any],
    *,
    fallback_state: str | None,
    fallback_task: str | None,
    camera_status: str | None,
) -> None:
    """탐지 종류와 역할을 기준으로 카드에 표시할 다음 상태를 계산한다."""

    object_type = str(detection["object_type"])
    if is_rat_object(object_type):
        next_state = "TRACKING"
        next_task = "쥐 추적 중"
    elif robot_role == "SURVEY_TRAP":
        next_state = "SEARCHING"
        next_task = "쥐구멍 탐색 및 트랩 설치"
    else:
        next_state = fallback_state
        next_task = fallback_task

    if next_state and next_task:
        changes: dict[str, Any] = {
            "state": next_state,
            "current_task": next_task,
            "target": {key: detection.get(key) for key in TARGET_FIELDS},
        }
        if camera_status is not None:
            changes["camera_status"] = camera_status
        state.update_robot(robot_id, **changes)


def _record_role_assignment(
    state: StateManager,
    db: Database,
    robot_id: str,
    assignment: dict[str, Any],
) -> None:
    """최초 역할 배정 결과를 실시간 타임라인과 DB에 함께 기록한다."""

    support_text = ", ".join(assignment["support_robot_ids"]) or "없음"
    event = state.add_event(
        f"{robot_id}가 쥐를 최초 발견했습니다. "
        f"{robot_id}는 쥐 추적, {support_text}는 쥐구멍 탐색·트랩 설치 "
        "역할로 자동 배정되었습니다.",
        robot_id=robot_id,
        event_type="ROLE_ASSIGNED",
    )
    db.insert_event(event)
