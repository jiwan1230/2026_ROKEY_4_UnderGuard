# 탐지 기록용 (제어와 무관 — db_node가 detection_event/rat_track에 기록).
# /fleet/event는 name:x:y 문자열이라 confidence/robot_id를 못 실어 별도로 둔다.
string object_type   # RAT | OPENING
float64 x
float64 y
float64 confidence
string robot_id
