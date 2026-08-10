"""
ROS나 탐지 모델에서 서로 다르게 들어올 수 있는 '객체 이름'을
'시스템 모니터링에서 사용하는 표준 이름'으로 통일
"""

from __future__ import annotations

# 중요 #
# 시스템이 사용할 위험 신호 이름을 미리 정해둠
# 살아있는 쥐 탐지
LIVE_RODENT = "LIVE_RODENT"
# 쥐가 드나드는 침입구
ENTRY_POINT = "ENTRY_POINT"
# 쥐 배설물
DROPPINGS = "DROPPINGS"

RISK_SIGNALS = {LIVE_RODENT, ENTRY_POINT, DROPPINGS}

# 중요 #
# 다른 이름으로 들어오는 객체를
# 시스템에서 사용하는 표준 위험신호로 연결해주는 딕셔너리
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


# 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 #
def normalize_risk_signal(object_type: str | None) -> str:
    '''
    들어온 객체 이름을 정리한 뒤,
    _ALIASES를 이용해서 시스템 표준 위험신호로 변환
    '''

    # object_type에 값이 있으면 그 값을 사용
    # str(...)로 무조건 문자열로 변환
    value = str(object_type or "").strip()
    # 위의 문자열 변수를 소문자로 바꾼 뒤, 그 값을 _ALIASES의 key로 찾아서 대응되는 value를 반환
    return _ALIASES.get(value.lower(), value)

# 중요 #
def is_live_rodent(object_type: str | None) -> bool:
    """입력 라벨이 살아있는 설치류 위험신호인지 True/False로 판별"""

    # object_type를 매개변수로 받아 표준 이름으로 바꾼 후 비교
    return normalize_risk_signal(object_type) == LIVE_RODENT


'''
탐지 데이터
   ↓
normalize_risk_signal()
   ↓
LIVE_RODENT 등으로 표준화
   ↓
쥐 여부 판단
   ↓
역할 배정 / 로봇 상태 변경
'''