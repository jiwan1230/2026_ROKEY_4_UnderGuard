# trap 점검 기록용 (제어와 무관 — db_node가 trap_inspection에 기록).
# /fleet/event의 trap_ok/trap_bad는 hole 좌표만 실어 trap_check_node가 이미
# 계산한 실제 trap 좌표·거리·robot_id는 여기로 별도 전달한다.
float64 hole_x
float64 hole_y
float64 trap_x
float64 trap_y
float64 distance
string result        # OK | MOVED
string robot_id
