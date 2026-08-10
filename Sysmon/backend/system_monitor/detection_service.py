'''
ROS에서 들어온 탐지 결과를 받아서
탐지 기록을 추가하고,
로봇의 역할과 상태를 변경하고,
이벤트까지 생성
'''

# 탐지 이후 임무에 영향을 주는 주요 사건들을 관제 상태와 이벤트로 변환하는 역할도 함께
# 배터리가 부족하면 새로운 임무 배정에 영향을 주기 때문에, 저배터리와 배터리 복구 처리도 이곳에 포함


from __future__ import annotations

from typing import Any

from .risk_signals import normalize_risk_signal
from .state_manager import StateManager, is_rat_object

# 탐지 데이터 전체 → 필요한 target 정보만 필터링
TARGET_FIELDS = ("object_type", "confidence", "distance", "map_x", "map_y", "source")

# 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 #
'''
탐지 데이터를 받아서 시스템 상태를 갱신.
탐지 등록 → 쥐 여부 확인 → 필요 시 역할 배정 → 로봇 상태 갱신 → 타임라인 이벤트 추가.
'''
def process_detection(
    # 현재 로봇 상태, 탐지 목록, 이벤트 등을 관리하는 StateManager 객체를 받음
    state: StateManager,
    ## ROS나 다른 곳에서 들어온 탐지 결과 데이터
    detection: dict[str, Any],
    *,
    ## 탐지가 발생했을 때 이벤트 타임라인에 기록할 메시지
    event_message: str,
    fallback_state: str | None = None,
    fallback_task: str | None = None,
    camera_status: str | None = None,
) -> dict[str, Any]:

    # 중요 #
    # 원본 detection을 복사
    data = dict(detection)
    # 탐지 데이터에서 robot_id를 가져옴
    robot_id = str(data["robot_id"])
    # 중요 #
    # 소스마다 다르게 들어올 수 있는 객체 이름을 시스템에서 쓰는 표준 이름으로 바꿈
    # (LIVE_RODENT/ENTRY_POINT/DROPPINGS) 정해진 위험 신호 이름으로 통일
    data["object_type"] = normalize_risk_signal(data["object_type"])
    # robot_id가 실제 등록된 로봇인지 확인
    state.get_robot(robot_id)
    # 중요 #
    # 정리된 탐지 데이터를 StateManager의 탐지 목록에 저장
    item = state.add_detection(data)
    # 중요 #
    # object_type이 쥐 계열인지 확인하고, 쥐라면 해당 로봇을 기준으로 역할 자동 배정을 시도
    assignment = (
        state.assign_roles_from_rat_detection(robot_id)
        if is_rat_object(data["object_type"])
        else None
    )
    # 역할 배정으로 role이 바뀌었을 수 있기 때문에, 로봇 정보를 다시 가져옴
    robot = state.get_robot(robot_id)

    # 중요 #
    # 이 로봇이 무엇을 탐지했고 현재 역할이 무엇인지 보고,
    # _update_robot_target()으로 넘겨 다음 상태·작업·target 정보를 갱신
    _update_robot_target(
        state,
        robot_id,
        robot["role"],
        ## 정리된 탐지 결과
        data,
        ## 로봇에게 적용할 “기본 상태값 문자열”
        fallback_state=fallback_state,
        fallback_task=fallback_task,
        camera_status=camera_status,
    )
    ## 새로운 역할 배정이 실제로 발생했는지 확인
    if assignment:
        # 그 내용을 이벤트로 기록
        _record_role_assignment(state, robot_id, assignment)

    ## 중요 #
    ## 지도 마커와는 별개로, 최근 이벤트 타임라인에도 이 탐지를 남긴다.
    state.add_event(
        # 화면에 보여줄 탐지 메시지
        event_message,
        robot_id=robot_id,
        event_type="DETECTION",
    )
    ## state.add_detection(data)로 저장했던 탐지 결과 item을 최종 반환
    return item


def record_target_lost(state: StateManager, robot_id: str) -> dict[str, Any]:
    """마지막 target을 유지하면서 현재 세션에 대상 유실 사건을 추가한다."""

    # target(마지막으로 본 위치)은 여기서 절대 안 지운다 — 로봇이 대상을
    # 놓쳐도 화면에는 "마지막으로 본 위치"가 계속 남아있어야 재탐색에
    # 도움이 되므로, state/작업 필드만 바꾸고 target은 그대로 둔다.
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
    '''배터리 값을 반영하고 현재 세션에 저전압 경고를 추가한다.'''

    state.update_robot(robot_id, battery=battery)
    # 실제로 임무 배정을 막는 로직은 따로 없다(System Monitor는 로봇에
    # 명령을 내리지 않는 read-only 관제라서) — 대신 이벤트 문구 자체에
    # "신규 확인 임무를 제한한다"는 의미를 담아 운영자에게 전달한다.
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
    '''배터리가 정상 범위로 돌아왔음을 기록한다(Mock의 "배터리 복구" 트리거)'''

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
    # map_x/map_y는 둘 다 주거나 둘 다 생략해야 한다 — 하나만 주면
    # 좌표가 불완전해서 지도에 잘못 찍히므로 여기서 미리 막는다.
    if (map_x is None) != (map_y is None):
        raise ValueError("트랩 위치에는 map_x와 map_y가 모두 필요합니다.")
    if map_x is None and map_y is None:
        # 좌표를 안 주면 로봇의 "현재 위치"를 트랩 위치로 대신 쓴다 —
        # 단 그 위치가 map frame 기준일 때만 허용한다. 다른 frame(예:
        # odom) 좌표를 그대로 쓰면 지도에서 엉뚱한 자리에 찍히기 때문이다.
        expected_frame = map_frame.strip("/") or "map"
        if robot["position_frame"] != expected_frame:
            raise ValueError(
                f"트랩 위치 표시에는 {expected_frame} frame 좌표가 필요합니다."
            )
        map_x = robot["position"]["x"]
        map_y = robot["position"]["y"]
    # 위 두 if문을 조합하면 여기서 map_x/map_y는 항상 float다(둘 다 None이면
    # 방금 채웠고, 하나만 None인 경우는 이미 위에서 예외로 걸렀다) — 다만
    # 타입 체커는 이 두 조건의 조합까지는 못 따라가므로 명시적으로 알려준다.
    assert map_x is not None and map_y is not None
    # 중요 #
    # 트랩은 detections와 별도 목록(add_trap)에 쌓인다 — 지도에서 다른
    # 모양(네모)으로 그려지고 탐지 마커와는 섞이지 않는다.
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


# 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 #
'''
탐지 처리 도중 내부적으로 호출.
(process_detection 과정에서) 탐지 대상과 로봇 역할을 보고 다음 상태와 작업을 결정
'''
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

    # 탐지 결과에서 object_type을 가져옴
    object_type = str(detection["object_type"])
    # 중요 #
    # 쥐를 본 로봇은 바로 추적 상태로 바꿈
    if is_rat_object(object_type):
        next_state = "TRACKING"
        next_task = "쥐 추적 중"
    # 침입구나 트랩을 조사하는 상태로 바꿉
    elif robot_role == "SURVEY_TRAP":
        # BRD상 이 역할의 로봇은 트랩을 직접 설치하지 않고 상태만 확인한다.
        next_state = "SEARCHING"
        next_task = "침입구·트랩 상태 확인"
    else:
        next_state = fallback_state
        next_task = fallback_task

    # fallback도 없으면(둘 다 None) 로봇 상태를 아예 안 바꾼다 —
    # 애매한 상태로 덮어쓰는 것보다 이전 상태를 유지하는 쪽이 안전하다.
    if next_state and next_task:
        # 바꿀 로봇 정보를 changes 딕셔너리에 모음
        changes: dict[str, Any] = {
            "state": next_state,
            "current_task": next_task,
            # 중요 #
            # TARGET_FIELDS에 적어둔 key들값을 detection에서 가져와 target 필드에 저장
            "target": {key: detection.get(key) for key in TARGET_FIELDS},
        }
        if camera_status is not None:
            changes["camera_status"] = camera_status
        # changes에 모은 내용을 해당 로봇에 반영
        state.update_robot(robot_id, **changes)


def _record_role_assignment(
    state: StateManager,
    robot_id: str,
    assignment: dict[str, Any],
) -> None:
    """최초 역할 배정 결과를 현재 세션의 타임라인에 추가한다."""

    support_text = ", ".join(assignment["support_robot_ids"]) or "없음"

    # process_detection이 assignment가 실제로 있을 때(=이번 호출에서
    # 새로 배정됐을 때)만 이 함수를 부르므로, 여기서는 별도 조건 없이
    # 바로 이벤트 하나만 남기면 된다.
    state.add_event(
        f"{robot_id}가 쥐를 최초 발견했습니다. "
        f"{robot_id}는 설치류 추적, {support_text}는 침입구·트랩 상태 확인 "
        "역할로 자동 배정되었습니다.",
        robot_id=robot_id,
        event_type="ROLE_ASSIGNED",
    )

'''
ROS/Mock 탐지 결과
     ↓
process_detection()
     ↓
객체 종류 정규화
     ↓
add_detection()
     ↓
쥐인가?
 ┌───┴────┐
YES       NO
 ↓
역할 자동 배정
     ↓
_update_robot_target()
     ↓
로봇 상태 / 작업 / Target 변경
     ↓
add_event()
     ↓
웹 관제 UI
'''

