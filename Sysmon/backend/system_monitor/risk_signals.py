"""Mock과 ROS의 객체 라벨을 세 가지 표준 위험신호로 정규화한다."""

from __future__ import annotations

LIVE_RODENT = "LIVE_RODENT"
ENTRY_POINT = "ENTRY_POINT"
DROPPINGS = "DROPPINGS"

RISK_SIGNALS = {LIVE_RODENT, ENTRY_POINT, DROPPINGS}

_ALIASES = {
    "live_rodent": LIVE_RODENT,
    "rc_car": LIVE_RODENT,
    "rat": LIVE_RODENT,
    "mouse": LIVE_RODENT,
    "entry_point": ENTRY_POINT,
    "rat_hole": ENTRY_POINT,
    "hole": ENTRY_POINT,
    "droppings": DROPPINGS,
    "dropping": DROPPINGS,
}


def normalize_risk_signal(object_type: str | None) -> str:
    """센서별 객체 라벨을 표준 위험신호 이름으로 변환한다.

    입력: ROS 모델 또는 Mock에서 받은 객체 라벨이다.
    출력: 알려진 별칭은 세 표준값 중 하나이며, 미등록 라벨은 원문을 유지한다.
    사용: 화면 상태 전이 전에 호출해 모드와 무관한 계약을 만든다.
    """

    value = str(object_type or "").strip()
    return _ALIASES.get(value.lower(), value)


def is_live_rodent(object_type: str | None) -> bool:
    """입력 라벨이 살아있는 설치류 위험신호인지 반환한다."""

    return normalize_risk_signal(object_type) == LIVE_RODENT
