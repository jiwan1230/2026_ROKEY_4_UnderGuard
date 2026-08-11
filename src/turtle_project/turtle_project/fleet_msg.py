"""fleet 통신 String 포맷 파싱/조립. 모든 노드가 공용 — 포맷을 한 곳에 둔다.

  status  : "robot4:PATROLLING:85"
  command : "robot6:UNDOCK"
  event   : "rat_detected:1.20:3.40"

문자열 자르기를 노드마다 재구현하면 오타로 깨지므로 여기로 통일. ROS 무관.
"""


def status(robot, state, battery):
    return f'{robot}:{state}:{int(battery)}'


def parse_status(s):
    robot, state, battery = s.split(':')
    return robot, state, int(battery)


def command(robot, cmd):
    return f'{robot}:{cmd}'


def parse_command(s):
    robot, cmd = s.split(':')
    return robot, cmd


def event(name, x, y):
    return f'{name}:{x:.2f}:{y:.2f}'


def parse_event(s):
    name, x, y = s.split(':')
    return name, float(x), float(y)


# 외부/수동 발행(ros2 topic pub 등)은 포맷이 깨져 있을 수 있다 — parse_*가 구독
# 콜백 안에서 ValueError를 내면 노드가 통째로 죽으므로, 콜백은 try_parse_*를 쓴다
# (포맷 불량이면 None). 조립 쪽(status/command/event)은 깨진 걸 만들 수 없어 그대로.
def _try(parser, s):
    try:
        return parser(s)
    except ValueError:
        return None


def try_parse_status(s):
    return _try(parse_status, s)


def try_parse_command(s):
    return _try(parse_command, s)


def try_parse_event(s):
    return _try(parse_event, s)


def _self_check():
    assert status('robot4', 'PATROLLING', 85) == 'robot4:PATROLLING:85'
    assert parse_status('robot4:PATROLLING:85') == ('robot4', 'PATROLLING', 85)
    # round-trip
    assert parse_status(status('robot6', 'DOCKED', 20)) == ('robot6', 'DOCKED', 20)

    assert command('robot6', 'UNDOCK') == 'robot6:UNDOCK'
    assert parse_command('robot6:UNDOCK') == ('robot6', 'UNDOCK')
    assert parse_command(command('robot4', 'PATROL')) == ('robot4', 'PATROL')

    assert event('rat_detected', 1.2, 3.4) == 'rat_detected:1.20:3.40'
    name, x, y = parse_event('rat_detected:1.20:3.40')
    assert name == 'rat_detected' and abs(x - 1.2) < 1e-9 and abs(y - 3.4) < 1e-9
    # round-trip
    n2, x2, y2 = parse_event(event('opening_confirmed', -0.5, 2.0))
    assert n2 == 'opening_confirmed' and abs(x2 + 0.5) < 1e-9 and abs(y2 - 2.0) < 1e-9

    # 포맷 불량 방어 — 콜백용 try_parse_*는 예외 대신 None
    assert try_parse_command('robot4:PATROL') == ('robot4', 'PATROL')
    assert try_parse_command('START') is None               # ':' 없음
    assert try_parse_command('a:b:c') is None               # ':' 과다
    assert try_parse_status('robot4:PATROLLING:85') == ('robot4', 'PATROLLING', 85)
    assert try_parse_status('robot4:PATROLLING:x') is None  # 배터리가 숫자 아님
    assert try_parse_event('rat_detected:1.20:3.40') == ('rat_detected', 1.2, 3.4)
    assert try_parse_event('rat_detected:1.2') is None      # 좌표 누락
    print('fleet_msg self-check ok')


if __name__ == '__main__':
    _self_check()
