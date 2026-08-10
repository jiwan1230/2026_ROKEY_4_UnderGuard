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


def trap_coord(hole_x, hole_y, trap_x, trap_y):
    """확인된 trap 실좌표를 구멍 좌표와 함께 db_node에 전달할 때 쓰는 별도 포맷.

    /fleet/event(event/parse_event)는 central_node 등 다른 노드도 name:x:y
    3필드로 파싱하므로 여기에 필드를 얹으면 그쪽 파싱이 깨진다 — 그래서
    전용 포맷/토픽(/db/trap_coord)으로 분리한다.
    """
    return f'{hole_x:.2f}:{hole_y:.2f}:{trap_x:.2f}:{trap_y:.2f}'


def parse_trap_coord(s):
    hx, hy, tx, ty = s.split(':')
    return float(hx), float(hy), float(tx), float(ty)


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

    assert trap_coord(1.0, 2.0, 1.1, 2.05) == '1.00:2.00:1.10:2.05'
    hx, hy, tx, ty = parse_trap_coord(trap_coord(-0.5, 3.25, -0.4, 3.1))
    assert abs(hx + 0.5) < 1e-9 and abs(hy - 3.25) < 1e-9
    assert abs(tx + 0.4) < 1e-9 and abs(ty - 3.1) < 1e-9
    print('fleet_msg self-check ok')


if __name__ == '__main__':
    _self_check()
