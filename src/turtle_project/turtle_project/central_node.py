"""중앙 조율 — 순찰 교대, 쥐대응 역할배정.

순찰 교대: 로봇A 배터리<임계 → RETURNING → DOCKED 수신하면 로봇B UNDOCK →
로봇B PATROL. 쥐대응: rat_detected 수신 시 PATROLLING 로봇=A, 나머지=B.

전이 판정은 next_command() 순수함수. 나머지 조율은 TODO(팀원).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from turtle_project import fleet_msg

# 로봇 A가 이 상태로 바뀌면 로봇 B에게 보낼 명령 (순찰 교대 핵심 전이).
_TRANSITIONS = {
    'DOCKED': 'UNDOCK',     # A가 도킹 완료 -> B undock (교대 시작)
}


def next_command(new_state):
    """로봇 A의 새 상태 -> 로봇 B에게 보낼 명령. 없으면 None.

    DOCKED 완료 후에만 B를 깨운다 (감시 공백 최소화하되 한 번에 한 대).
    """
    return _TRANSITIONS.get(new_state)


def assign_roles(patroller, robots):
    """쥐 감지 시 역할 배정 -> (robot_a, robot_b). 못 정하면 None.

    순찰 중이던 로봇이 A(쥐 추적), 나머지가 B(몰이). patroller가 아직 없거나
    로봇이 2대가 안 되면 배정 불가(None). robot4/robot6 어느 쪽이든 무관.
    """
    if patroller is None or patroller not in robots:
        return None
    others = [r for r in robots if r != patroller]
    if not others:
        return None
    return patroller, others[0]


class CentralNode(Node):

    def __init__(self):
        super().__init__('central_node')
        self.robots = {}        # robot -> (state, battery)
        self.patroller = None   # 현재 순찰 로봇 (역할 A)
        self.rat_mode = False   # 쥐대응 모드 (중복 트리거 방지)
        self.rat_roles = None   # 쥐대응 중인 (A, B) — 종료 시 복귀 명령에 사용
        self.rat_caught = False  # 마지막 쥐대응 결과 표출 (포획 성공 여부)

        self.cmd_pub = self.create_publisher(String, '/fleet/command', 10)
        self.create_subscription(String, '/fleet/status', self.status_cb, 10)
        self.create_subscription(String, '/fleet/event', self.event_cb, 10)
        self.get_logger().info('중앙 노드 시작 — status/event 대기')

    def status_cb(self, msg):
        robot, state, battery = fleet_msg.parse_status(msg.data)
        prev = self.robots.get(robot)
        self.robots[robot] = (state, battery)
        if state == 'PATROLLING':
            self.patroller = robot
        # 교대 핵심 전이: A가 DOCKED되면 B를 undock.
        cmd = next_command(state)
        if cmd and (prev is None or prev[0] != state):
            other = self._other(robot)
            if other:
                self.send(other, cmd)
        # TODO(팀원): PATROL 이어받기, 재도킹, 상태 정합성 등 나머지 시퀀스.

    def event_cb(self, msg):
        name, x, y = fleet_msg.parse_event(msg.data)
        self.get_logger().info(f'이벤트 {name} at ({x:.2f}, {y:.2f})')
        if name == 'rat_detected':
            self._on_rat()
        elif name == 'rat_captured':
            self._end_rat(caught=True)
        elif name == 'rat_lost':
            self._end_rat(caught=False)
        # TODO(팀원): opening_confirmed/trap_ok 등 나머지 이벤트 처리.

    def _on_rat(self):
        """쥐 감지 -> 역할 배정 후 A는 TRACK, B는 HERD. 이미 대응 중이면 무시.

        쥐는 여러 프레임 연속 감지되므로 rat_mode로 첫 트리거만 받는다.
        """
        if self.rat_mode:
            return                      # 이미 쥐대응 중 — 재배정 안 함
        roles = assign_roles(self.patroller, self.robots)
        if roles is None:
            self.get_logger().warn('쥐 감지했으나 역할 배정 불가 (순찰 로봇/2대 필요)')
            return
        robot_a, robot_b = roles
        self.rat_mode = True
        self.rat_roles = roles          # 종료 시 A→PATROL, B→DOCK에 사용
        self.get_logger().info(f'쥐대응 진입 — A(추적)={robot_a}, B(몰이)={robot_b}')
        self.send(robot_a, 'TRACK')
        self.send(robot_b, 'HERD')

    def _end_rat(self, caught):
        """포획/놓침 -> 쥐대응 종료. A는 순찰 복귀, B는 도킹 복귀.

        detector가 포획(rat_captured)/놓침(rat_lost)을 판정해 보내면 여기서
        받는다. 어느 쪽이든 쥐대응은 끝 — A는 다시 순찰, B는 원래 쉬던 로봇이라
        도킹으로 돌아가 '한 대씩 순찰' 상태로 복귀한다.
        """
        if not self.rat_mode:
            return                      # 쥐대응 중 아님 — 무시 (중복 이벤트)
        robot_a, robot_b = self.rat_roles
        self.rat_mode = False
        self.rat_roles = None
        self.rat_caught = caught        # 결과 표출 (포획 성공/놓침)
        result = '포획 성공' if caught else '놓침'
        self.get_logger().info(f'쥐대응 종료 ({result}) — {robot_a} 순찰, {robot_b} 도킹')
        self.send(robot_a, 'PATROL')
        self.send(robot_b, 'DOCK')

    def send(self, robot, cmd):
        self.cmd_pub.publish(String(data=fleet_msg.command(robot, cmd)))
        self.get_logger().info(f'명령 → {robot}:{cmd}')

    def _other(self, robot):
        """robots 중 이 로봇이 아닌 다른 하나. 없으면 None."""
        others = [r for r in self.robots if r != robot]
        return others[0] if others else None


def main():
    rclpy.init()
    node = CentralNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


def _self_check():
    assert next_command('DOCKED') == 'UNDOCK'   # 교대 핵심 전이
    assert next_command('PATROLLING') is None    # 순찰 중엔 교대 명령 없음
    assert next_command('RETURNING') is None

    robots = {'robot4': ('PATROLLING', 80), 'robot6': ('DOCKED', 100)}
    # 순찰 중인 robot4가 A, 나머지 robot6가 B
    assert assign_roles('robot4', robots) == ('robot4', 'robot6')
    # 반대로 robot6이 순찰 중이면 역할이 뒤바뀐다 (누가 A/B든 무관)
    assert assign_roles('robot6', robots) == ('robot6', 'robot4')
    # 순찰 로봇 없음 -> 배정 불가
    assert assign_roles(None, robots) is None
    # 로봇 1대뿐 -> 배정 불가
    assert assign_roles('robot4', {'robot4': ('PATROLLING', 80)}) is None
    # patroller가 목록에 없음 -> 배정 불가
    assert assign_roles('robot9', robots) is None
    print('central_node self-check ok')


if __name__ == '__main__':
    import sys
    _self_check() if '--check' in sys.argv else main()
