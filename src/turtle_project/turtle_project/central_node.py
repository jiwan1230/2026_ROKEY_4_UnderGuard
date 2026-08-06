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


def coverage_action(robots, waking):
    """전원 상태로 부트스트랩/복구 판단 -> UNDOCK 시킬 로봇 or None.

    아무도 순찰/복귀/대응(busy) 중이 아니고 깨우는 중(waking)도 아니면, 도크에
    있는 로봇 하나를 골라 그 이름을 돌려준다(순찰 시작용). 커버리지가 이미
    있거나 깨우는 중이면 None. 도킹 주행(RETURNING)은 busy로 봐서, 교대 진행
    중엔 개입하지 않는다 (잠깐의 감시 공백은 정상).
    """
    if waking is not None:
        return None
    busy = {'PATROLLING', 'TRACKING', 'HERDING', 'RETURNING'}
    if any(s in busy for s, _ in robots.values()):
        return None
    idle = [r for r, (s, _) in robots.items() if s in ('DOCKED', 'IDLE')]
    return idle[0] if idle else None


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
        self.waking = None      # UNDOCK 보낸 로봇 (순찰 시작할 때까지 중복 방지)
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
            if self.waking == robot:
                self.waking = None      # 깨우라던 로봇이 순찰 시작 — 대기 해제
        if self.rat_mode:
            return                      # 쥐대응 중엔 역할 고정 — 조율 개입 안 함
        # 교대: 순찰하던 A가 DOCKED로 '전이'하면 B를 깨운다. prev is None(첫 status)은
        # 전이가 아니다 — 그건 부트스트랩(_ensure_coverage)이 처리한다. 안 그러면
        # 둘 다 DOCKED로 시작할 때 서로를 깨워 둘 다 순찰하게 된다.
        cmd = next_command(state)       # DOCKED -> UNDOCK
        if cmd and prev is not None and prev[0] != state:
            # 이미 다른 로봇이 순찰 중이면(쥐대응 종료 후 B 도킹 등) 깨우지 않는다.
            if not self._anyone_patrolling():
                other = self._other(robot)
                if other:
                    self.send(other, cmd)
                    self.waking = other
            return
        self._ensure_coverage()         # 전원 대기면 한 대 깨워 순찰 시작(부트스트랩)

    def _ensure_coverage(self):
        """항상 최소 1대 순찰하도록 보장 (부트스트랩/복구). 판단은 coverage_action."""
        r = coverage_action(self.robots, self.waking)
        if r:
            self.send(r, 'UNDOCK')
            self.waking = r
            self.get_logger().info(f'커버리지 확보 — {r} 순찰 시작')

    def _anyone_patrolling(self):
        return any(s == 'PATROLLING' for s, _ in self.robots.values())

    def event_cb(self, msg):
        name, x, y = fleet_msg.parse_event(msg.data)
        self.get_logger().info(f'이벤트 {name} at ({x:.2f}, {y:.2f})')
        if name == 'rat_detected':
            self._on_rat()
        elif name == 'rat_captured':
            self._end_rat(caught=True)
        elif name == 'rat_lost':
            self._end_rat(caught=False)
        # opening_confirmed/trap_installed/trap_ok/trap_bad 등은 로봇 로컬에서
        # 처리(db_node 저장, detector 순찰재개)하므로 central은 위 로그로만 관찰한다
        # — 설계상 central은 방역 세부를 조율하지 않고 PATROLLING으로만 안다.

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

    # 커버리지/부트스트랩
    both_docked = {'robot4': ('DOCKED', 100), 'robot6': ('DOCKED', 100)}
    assert coverage_action(both_docked, None) in ('robot4', 'robot6')  # 전원 도킹 → 한 대 깨움
    assert coverage_action(both_docked, 'robot4') is None              # 이미 깨우는 중 → 안 함
    # 누군가 순찰/복귀 중이면 개입 안 함 (커버리지 있음/교대 진행 중)
    assert coverage_action({'robot4': ('PATROLLING', 80), 'robot6': ('DOCKED', 100)}, None) is None
    assert coverage_action({'robot4': ('RETURNING', 20), 'robot6': ('DOCKED', 100)}, None) is None
    assert coverage_action({'robot4': ('IDLE', 50)}, None) == 'robot4'  # 대기뿐 → 후보
    assert coverage_action({}, None) is None                            # 로봇 없음 → None
    print('central_node self-check ok')


if __name__ == '__main__':
    import sys
    _self_check() if '--check' in sys.argv else main()
