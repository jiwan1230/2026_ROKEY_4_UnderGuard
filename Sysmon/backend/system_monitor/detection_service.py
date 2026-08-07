"""Mock과 ROS 사건을 영구 저장 없이 실시간 관제 상태로 변환한다."""

from __future__ import annotations

from typing import Any

from .risk_signals import normalize_risk_signal
from .state_manager import StateManager, is_rat_object

TARGET_FIELDS = ("object_type", "confidence", "distance", "map_x", "map_y", "source")


def process_detection(
    state: StateManager,
    detection: dict[str, Any],
    *,
    event_message: str,
    fallback_state: str | None = None,
    fallback_task: str | None = None,
    camera_status: str | None = None,
) -> dict[str, Any]:
    """탐지 한 건을 현재 세션의 지도·카드·최근 이벤트에 반영한다.

    입력: 상태 객체와 로봇·위험신호·좌표를 포함한 탐지다.
    출력: 메모리 ID와 수신 시각이 추가된 탐지 항목이다.
    사용: Mock/ROS 콜백에서 호출하며 서버 재시작 후에는 남기지 않는다.
    """

    data = dict(detection)
    robot_id = str(data["robot_id"])
    data["object_type"] = normalize_risk_signal(data["object_type"])
    state.get_robot(robot_id)
    item = state.add_detection(data)

    assignment = (
        state.assign_roles_from_rat_detection(robot_id)
        if is_rat_object(data["object_type"])
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
        _record_role_assignment(state, robot_id, assignment)

    state.add_event(
        event_message,
        robot_id=robot_id,
        event_type="DETECTION",
    )
    return item


def record_target_lost(state: StateManager, robot_id: str) -> dict[str, Any]:
    """마지막 target을 유지하면서 현재 세션에 대상 유실 사건을 추가한다."""

    state.update_robot(
        robot_id,
        state="TARGET_LOST",
        speed=0.0,
        nav_status="CANCELED",
        current_task="대상 재탐색 대기",
    )
    return state.add_event(
        "추적 대상을 놓쳤습니다. 마지막 탐지 위치를 유지합니다.",
        robot_id=robot_id,
        severity="WARNING",
        event_type="TARGET_LOST",
    )


def record_low_battery(
    state: StateManager,
    robot_id: str,
    battery: float,
) -> dict[str, Any]:
    """배터리 값을 반영하고 현재 세션에 저전압 경고를 추가한다.

    로봇의 실제 이동 상태(state)는 Fleet/ROS가 보고하는 값이 최종 권위를
    가지므로 여기서 임의로 덮어쓰지 않는다. 배터리 기준 운영 의미(복귀 권장·
    신규 확인 임무 제한)는 화면(dashboard.js)에서 배터리 수치로부터 매번
    다시 계산해 표시한다.
    """

    state.update_robot(robot_id, battery=battery)
    return state.add_event(
        f"배터리가 부족합니다({battery:.1f}%). 복귀를 권장하며 "
        "신규 확인 임무 배정을 제한합니다.",
        robot_id=robot_id,
        severity="WARNING",
        event_type="LOW_BATTERY",
    )


def record_battery_recovered(
    state: StateManager,
    robot_id: str,
    battery: float = 80.0,
) -> dict[str, Any]:
    """배터리 값을 정상 범위로 되돌리고 경고 해제 사건을 추가한다.

    Mock 시연에서 "배터리 부족 → 경고 → 복구 → 경고 종료" 흐름의 마지막
    단계다. record_low_battery와 같은 계층(공통 Mock/ROS 처리)에 둬서,
    호출부가 StateManager를 직접 건드리지 않게 한다.
    """

    state.update_robot(robot_id, battery=battery)
    return state.add_event(
        "배터리가 복구되어 신규 확인 임무 제한이 해제되었습니다.",
        robot_id=robot_id,
        event_type="BATTERY_RECOVERED",
    )


def record_trap_installed(
    state: StateManager,
    robot_id: str,
    *,
    map_frame: str = "map",
    map_x: float | None = None,
    map_y: float | None = None,
) -> dict[str, Any]:
    """현재 세션의 지도에 트랩 위치와 완료 사건을 추가한다."""

    robot = state.get_robot(robot_id)
    if (map_x is None) != (map_y is None):
        raise ValueError("트랩 위치에는 map_x와 map_y가 모두 필요합니다.")
    if map_x is None and map_y is None:
        expected_frame = map_frame.strip("/") or "map"
        if robot["position_frame"] != expected_frame:
            raise ValueError(
                f"트랩 위치 표시에는 {expected_frame} frame 좌표가 필요합니다."
            )
        map_x = robot["position"]["x"]
        map_y = robot["position"]["y"]
    state.add_trap(
        {
            "robot_id": robot_id,
            "map_x": float(map_x),
            "map_y": float(map_y),
            "status": "INSTALLED",
        }
    )
    state.update_robot(
        robot_id,
        state="COMPLETED",
        speed=0.0,
        current_task="트랩 상태 확인 완료 · 다음 지시 대기",
    )
    return state.add_event(
        "등록된 트랩 상태 확인이 완료되었습니다.",
        robot_id=robot_id,
        event_type="TRAP_INSTALLED",
    )


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
        next_task = "침입구·트랩 상태 확인"
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
    robot_id: str,
    assignment: dict[str, Any],
) -> None:
    """최초 역할 배정 결과를 현재 세션의 타임라인에 추가한다."""

    support_text = ", ".join(assignment["support_robot_ids"]) or "없음"
    state.add_event(
        f"{robot_id}가 쥐를 최초 발견했습니다. "
        f"{robot_id}는 설치류 추적, {support_text}는 침입구·트랩 상태 확인 "
        "역할로 자동 배정되었습니다.",
        robot_id=robot_id,
        event_type="ROLE_ASSIGNED",
    )
