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
# UNDOCK이 아니라 PATROL을 보낸다 — robot_agent가 도킹 중이면 언도킹부터,
# 이미 필드 위(IDLE)면 바로 순찰로 알아서 분기한다(robot_agent.command_cb).
# UNDOCK을 보내면 이미 언도킹된 로봇도 undock 액션을 또 타서 문제.
_TRANSITIONS = {
    'DOCKED': 'PATROL',     # A가 도킹 완료 -> B 순찰 투입 (교대 시작)
}

# UNDOCK 보낸 로봇이 이 시간 안에 순찰을 시작 못 하면 waking을 풀어 재시도 허용
# (언도킹 실패·waypoint 파일 없음·명령 유실 시 커버리지 복구가 영구 정지하는 것 방지).
WAKE_TIMEOUT = 30.0
# status가 이 시간 넘게 끊긴 로봇은 죽은 것으로 보고 목록에서 제거
# (보고 주기 10초의 3배 — 마지막 상태가 PATROLLING인 채 남는 것 방지).
STALE_SECS = 30.0


def stale_robots(last_seen, now, limit=STALE_SECS):
    """limit초 넘게 status 없는 로봇 이름 목록. 순수함수."""
    return [r for r, t in last_seen.items() if now - t > limit]


def next_patroller(current, robot, state):
    """status 1건으로 patroller(역할 A 후보) 갱신 -> 새 patroller.

    순찰 시작한 로봇이 A가 된다. 현재 A가 순찰을 멈추면(RETURNING/DOCKED 등)
    해제한다 — 안 그러면 배터리 복귀 중인 A에게 쥐 추적(TRACK)이 배정된다.
    """
    if state == 'PATROLLING':
        return robot
    if current == robot:
        return None
    return current


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
    busy = {'PATROLLING', 'TRACKING', 'HERDING', 'SWEEPING', 'RETURNING',
            'UNDOCKING'}
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
        self.last_seen = {}     # robot -> 마지막 status 수신 시각 (스테일 판정)
        self.patroller = None   # 현재 순찰 로봇 (역할 A)
        self.waking = None      # (UNDOCK 보낸 로봇, 보낸 시각). None=깨우는 중 아님
        self.rat_mode = False   # 쥐대응 모드 (중복 트리거 방지)
        self.rat_roles = None   # 쥐대응 중인 (A, B) — 종료 시 복귀 명령에 사용
        self.rat_caught = False  # 마지막 쥐대응 결과 표출 (포획 성공 여부)
        # UI(Sysmon) '시스템 시작' 버튼으로 켜지는 스위치. False면 모든 조율
        # (순찰 투입·교대·쥐대응)을 보류하고 status/event 기록만 한다.
        self.started = False

        self.cmd_pub = self.create_publisher(String, '/fleet/command', 10)
        self.create_subscription(String, '/fleet/status', self.status_cb, 10)
        self.create_subscription(String, '/fleet/event', self.event_cb, 10)
        # system:START/STOP 수신용 — 자기가 발행한 로봇 명령도 같이 들어오므로
        # fleet_cmd_cb가 'system' 대상만 골라 처리한다.
        self.create_subscription(String, '/fleet/command', self.fleet_cmd_cb, 10)
        # 죽은 로봇 청소 — status가 끊긴 로봇을 제거해 커버리지 오판 방지.
        self.create_timer(10.0, self._prune_stale)
        self.get_logger().info('중앙 노드 시작 — 대기 모드 (system:START 수신 시 순찰 개시)')

    def fleet_cmd_cb(self, msg):
        """UI(또는 수동 topic pub)의 시스템 제어 — system:START / system:STOP.

        START: 대기 해제 + 즉시 커버리지 확보(한 대 순찰 투입).
        STOP: 전 로봇 STOP 발행 + 대기 모드 복귀. started를 안 끄면 커버리지
        로직이 몇 초 뒤 IDLE 로봇을 도로 순찰 투입시키므로 반드시 같이 끈다.
        """
        robot, cmd = fleet_msg.parse_command(msg.data)
        if robot != 'system':
            return                      # 로봇 개별 명령 (자기 발행 echo 포함) — 무시
        if cmd == 'START' and not self.started:
            self.started = True
            self.get_logger().info('시스템 시작 — 조율 개시')
            self._ensure_coverage()
        elif cmd == 'STOP' and self.started:
            self.started = False
            self.rat_mode = False
            self.rat_roles = None
            self.waking = None
            for r in self.robots:
                self.send(r, 'STOP')
            self.get_logger().info('시스템 정지 — 전 로봇 STOP, 대기 모드 복귀')

    def status_cb(self, msg):
        robot, state, battery = fleet_msg.parse_status(msg.data)
        prev = self.robots.get(robot)
        self.robots[robot] = (state, battery)
        self.last_seen[robot] = self._now()
        self.patroller = next_patroller(self.patroller, robot, state)
        if state == 'PATROLLING':
            if self.waking and self.waking[0] == robot:
                self.waking = None      # 깨우라던 로봇이 순찰 시작 — 대기 해제
        if not self.started:
            return                      # 시스템 시작 전 — 기록만, 조율 보류
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
                    self.waking = (other, self._now())
            return
        self._ensure_coverage()         # 전원 대기면 한 대 깨워 순찰 시작(부트스트랩)

    def _ensure_coverage(self):
        """항상 최소 1대 순찰하도록 보장 (부트스트랩/복구). 판단은 coverage_action."""
        if not self.started:
            return                      # 시스템 시작 전 (_prune_stale 경유 호출 방어)
        if self.waking and self._now() - self.waking[1] > WAKE_TIMEOUT:
            self.get_logger().warn(
                f'{self.waking[0]} 깨웠지만 {WAKE_TIMEOUT:.0f}초째 순찰 시작 '
                '없음 — 재시도 허용')
            self.waking = None
        r = coverage_action(self.robots, self.waking)
        if r:
            # PATROL 하나로 충분 — 도킹 여부 분기는 robot_agent가 한다(_TRANSITIONS 주석).
            self.send(r, 'PATROL')
            self.waking = (r, self._now())
            self.get_logger().info(f'커버리지 확보 — {r} 순찰 시작')

    def _prune_stale(self):
        """status 끊긴 로봇 제거 — 죽은 로봇의 마지막 상태(PATROLLING 등)가
        남아 커버리지가 있다고 오판하는 것을 막는다. 빈자리는 즉시 복구 시도."""
        for r in stale_robots(self.last_seen, self._now()):
            self.get_logger().warn(f'{r} status {STALE_SECS:.0f}초 끊김 — 제거')
            self.robots.pop(r, None)
            self.last_seen.pop(r, None)
            if self.patroller == r:
                self.patroller = None
            if self.waking and self.waking[0] == r:
                self.waking = None
        if not self.rat_mode:
            self._ensure_coverage()

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _anyone_patrolling(self):
        return any(s == 'PATROLLING' for s, _ in self.robots.values())

    def event_cb(self, msg):
        name, x, y = fleet_msg.parse_event(msg.data)
        self.get_logger().info(f'이벤트 {name} at ({x:.2f}, {y:.2f})')
        if not self.started:
            return                      # 시스템 시작 전 — 쥐대응 등 조율 보류
        if name == 'rat_detected':
            self._on_rat()
        elif name == 'rat_captured':
            self._end_rat(caught=True)
        elif name == 'rat_lost':
            self._end_rat(caught=False)
        elif name == 'sweep_done':
            self._on_sweep_done()
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
        self.get_logger().info(f'쥐대응 진입 — A(추적)={robot_a}, B(점검)={robot_b}')
        self.send(robot_a, 'TRACK')
        # B는 먼저 DB 구멍을 순회 점검(SWEEP)하고, 다 끝내면(sweep_done) HERD로 전환.
        self.send(robot_b, 'SWEEP')

    def _on_sweep_done(self):
        """B가 DB 구멍 순회 점검을 마침 → herding 시작 (쥐대응 중일 때만).

        쥐대응이 이미 끝났으면(포획/놓침) 늦게 온 sweep_done은 무시한다.
        """
        if not self.rat_mode or self.rat_roles is None:
            return
        _, robot_b = self.rat_roles
        self.get_logger().info(f'{robot_b} 구멍 점검 완료 — HERD(몰이) 시작')
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
    # patroller 갱신/스테일 해제 (문제 2-5)
    assert next_patroller(None, 'robot4', 'PATROLLING') == 'robot4'   # 순찰 시작 → A
    assert next_patroller('robot4', 'robot4', 'RETURNING') is None    # A가 복귀 → 해제
    assert next_patroller('robot4', 'robot4', 'DOCKED') is None       # A가 도킹 → 해제
    assert next_patroller('robot4', 'robot6', 'DOCKED') == 'robot4'   # 남이 도킹 → 유지
    assert next_patroller('robot4', 'robot6', 'PATROLLING') == 'robot6'  # 교대

    assert next_command('DOCKED') == 'PATROL'   # 교대 핵심 전이 (도킹 분기는 로봇이)
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
    # 언도킹 중(수동 UNDOCK 등)도 busy — 다른 로봇을 또 깨우지 않는다
    assert coverage_action({'robot4': ('UNDOCKING', 90), 'robot6': ('DOCKED', 100)}, None) is None
    # 구멍 순회 점검 중(SWEEPING)도 busy — 다른 로봇을 또 깨우지 않는다
    assert coverage_action({'robot4': ('SWEEPING', 90), 'robot6': ('DOCKED', 100)}, None) is None
    assert coverage_action({'robot4': ('IDLE', 50)}, None) == 'robot4'  # 대기뿐 → 후보
    assert coverage_action({}, None) is None                            # 로봇 없음 → None
    # waking이 (robot, 시각) 튜플이어도 '깨우는 중' 판정은 그대로 동작
    assert coverage_action(both_docked, ('robot4', 100.0)) is None

    # 스테일 판정 — limit초 넘게 status 없는 로봇만 골라낸다
    assert stale_robots({'robot4': 90.0, 'robot6': 105.0}, 125.0, 30.0) == ['robot4']
    assert stale_robots({'robot4': 100.0}, 129.0, 30.0) == []   # 29초 — 아직
    assert stale_robots({}, 999.0, 30.0) == []                  # 빈 목록
    print('central_node self-check ok')


if __name__ == '__main__':
    import sys
    _self_check() if '--check' in sys.argv else main()
