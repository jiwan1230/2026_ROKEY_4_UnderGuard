"""로봇 1대 제어 — 순찰 주행(Nav2)·dock/undock·배터리 감시·상태보고.

namespace 파라미터로 로봇마다 실행 (robot4/robot6). /fleet/command에서 자기
명령만 필터해 실행하고, /fleet/status로 상태를 보고한다.

PATROL/UNDOCK을 받으면 waypoint YAML을 FollowWaypoints로 무한 순찰한다
(patrol_zigzag 로직 통합). 배터리<임계 또는 DOCK 명령이면 도크 앞 좌표로
이동해 정밀 도킹한다. dock/undock/navigate는 모두 논블로킹 폴링(patrol_tick)
으로 진행한다 — 순찰의 isTaskComplete 폴링과 같은 방식.
"""
import math

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

from turtle_project import fleet_msg


def load_waypoints(path):
    """순찰 waypoint YAML -> (frame_id, [wp]). generate_zigzag_waypoints.py 산출물."""
    with open(path) as f:
        data = yaml.safe_load(f)
    return data['frame_id'], data['waypoints']


def to_pose(nav, frame_id, wp):
    """waypoint dict -> map 프레임 PoseStamped. 좌표는 맵 절대좌표라 로봇
    시작 위치와 무관 (맵 원점이 어디든 각 로봇이 자기 위치만 알면 됨)."""
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = wp['x']
    p.pose.position.y = wp['y']
    yaw = math.radians(wp['yaw_deg'])
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


class RobotAgent(Node):

    def __init__(self):
        super().__init__('robot_agent')
        self.ns = self.declare_parameter('namespace', 'robot4').value
        self.threshold = self.declare_parameter('battery_threshold', 40).value
        period = self.declare_parameter('battery_check_period', 10.0).value
        self.wp_file = self.declare_parameter(
            'waypoints', '/home/rokey/patrol_waypoints.yaml').value
        # 도크 '앞' 접근점 (map 절대좌표) — 여기까지 Nav2로 간 뒤 dock() 정밀도킹.
        # 로봇/현장마다 실측 필요. yaw는 도크를 바라보는 방향(도)이어야 한다.
        self.dock_x = self.declare_parameter('dock_x', 0.0).value
        self.dock_y = self.declare_parameter('dock_y', 0.0).value
        self.dock_yaw = self.declare_parameter('dock_yaw_deg', 0.0).value

        self.state = 'IDLE'
        self.battery = 100

        # TurtleBot4Navigator는 BasicNavigator 상속 — 순찰(followWaypoints)에
        # dock/undock 액션이 딸려온다. 자체 노드라 main에서 같은 executor에 돌린다.
        self.nav = TurtleBot4Navigator(namespace=f'/{self.ns}')
        self.poses = None       # 로드된 순찰 waypoint (첫 PATROL 때 lazy 로드)
        self.patrolling = False  # followWaypoints 진행 중
        self.hold = False       # opening 처리 중 순찰 정지 (detector가 신호)
        # 도킹/언도킹 논블로킹 진행 단계. None=진행 중 아님.
        # 'NAV'=도크앞 이동 중, 'DOCK'=dock 액션 중, 'UNDOCK'=undock 액션 중.
        self.dock_phase = None

        self.status_pub = self.create_publisher(String, '/fleet/status', 10)
        self.create_subscription(String, '/fleet/command', self.command_cb, 10)
        # 로봇별 토픽은 상대경로 — detector·trap_check·camera와 같은 방식이라,
        # 실행 때 __ns:=/robot4 를 걸면 /robot4/... 로 맞춰진다. namespace
        # 파라미터는 fleet 식별(status/command)과 Nav2(TurtleBot4Navigator)용.
        self.create_subscription(BatteryState, 'battery_state',
                                 self.battery_cb, 10)
        self.create_subscription(PoseStamped, 'target_pose',
                                 self.target_cb, 10)
        self.create_subscription(Bool, 'patrol_hold',
                                 self.hold_cb, 10)
        self.create_timer(period, self.report)
        # 순찰 진행 감시 — 한 바퀴 끝나면 다음 바퀴 재시작 (무한 순찰).
        self.create_timer(1.0, self.patrol_tick)
        self.get_logger().info(f'{self.ns} agent 시작 — 임계 {self.threshold}%')

    def command_cb(self, msg):
        robot, cmd = fleet_msg.parse_command(msg.data)
        if robot != self.ns:
            return                          # 내 명령 아님
        self.get_logger().info(f'명령 수신: {cmd}')
        # DOCK은 여기서 상태를 안 바꾼다 — 도킹 완료(_dock_tick)에서만 DOCKED로.
        # 도킹 진행 중 DOCKED를 방송하면 central이 조기에 교대 트리거를 건다.
        self.state = {'UNDOCK': 'PATROLLING', 'PATROL': 'PATROLLING',
                      'TRACK': 'TRACKING', 'HERD': 'HERDING',
                      'STOP': 'IDLE'}.get(cmd, self.state)
        if cmd == 'UNDOCK':
            self.start_undocking()          # 도크에서 빠져나온 뒤 순찰 시작
        elif cmd == 'PATROL':
            self.start_patrol()             # 이미 필드 위 — 바로 순찰
        elif cmd == 'DOCK':
            self.state = 'RETURNING'        # 도킹 완료 시 _dock_tick이 DOCKED로
            self.start_docking()
        elif cmd in ('STOP', 'TRACK', 'HERD'):
            # 순찰 중단 — 추적/몰이는 target_pose로 별도 주행(아래 target_cb).
            self.stop_patrol()

    def start_patrol(self):
        """waypoint 순찰 시작. 이미 도는 중이면 무시. waypoint는 첫 호출 때 로드."""
        if self.patrolling:
            return
        if self.poses is None:
            try:
                frame_id, wps = load_waypoints(self.wp_file)
            except FileNotFoundError:
                self.get_logger().error(f'waypoint 파일 없음: {self.wp_file}')
                return
            if not wps:
                self.get_logger().error('waypoint가 비어있음')
                return
            self.poses = [to_pose(self.nav, frame_id, w) for w in wps]
        self._send_lap()

    def _send_lap(self):
        """순찰 한 바퀴 goal 발행. stamp를 현재로 갱신해서 보낸다."""
        now = self.nav.get_clock().now().to_msg()
        for p in self.poses:
            p.header.stamp = now
        self.nav.followWaypoints(self.poses)
        self.patrolling = True

    def stop_patrol(self):
        if self.patrolling:
            self.nav.cancelTask()
            self.patrolling = False

    def start_docking(self):
        """도크 앞 좌표로 Nav2 이동 시작 → 도착하면 _dock_tick이 dock() 액션.

        블로킹 dock() 대신 논블로킹으로, 순찰과 같은 폴링(patrol_tick)에서 진행.
        """
        self.stop_patrol()
        dock_pose = to_pose(self.nav, 'map', {
            'x': self.dock_x, 'y': self.dock_y, 'yaw_deg': self.dock_yaw})
        self.nav.goToPose(dock_pose)
        self.dock_phase = 'NAV'             # 도크앞 이동 중
        self.get_logger().info(
            f'도킹 시작 — 도크앞 ({self.dock_x:.2f}, {self.dock_y:.2f})로 이동')

    def start_undocking(self):
        """undock 액션으로 도크에서 빠져나온 뒤 순찰 시작 (논블로킹).

        도크에 붙은 채 followWaypoints를 보내면 안 먹으므로 먼저 빠져나온다.
        이미 도크 밖이면 undock 액션이 즉시 완료로 처리된다.
        """
        self.stop_patrol()
        self.nav.undock_send_goal()
        self.dock_phase = 'UNDOCK'
        self.get_logger().info('언도킹 시작 — 도크에서 빠져나오는 중')

    def patrol_tick(self):
        """1초 폴링 — 도킹/언도킹 진행 우선, 아니면 순찰 한 바퀴 이어돌기.

        dock/undock 액션이 블로킹이라 콜백에서 직접 못 부른다. 대신 여기서
        완료 여부만 폴링해 단계를 넘긴다 (순찰 isTaskComplete 폴링과 같은 방식).
        """
        if self.dock_phase is not None:
            self._dock_tick()
            return
        if self.hold or not self.patrolling or not self.nav.isTaskComplete():
            return
        result = self.nav.getResult()
        if result == TaskResult.CANCELED:
            self.patrolling = False         # 외부 취소 — 순찰 종료
            return
        # SUCCEEDED든 실패든 계속 순찰 (실패는 다음 바퀴에 재시도).
        self._send_lap()

    def _dock_tick(self):
        """도킹/언도킹 단계 진행. NAV→도착하면 dock 액션, 완료되면 DOCKED 방송."""
        if self.dock_phase == 'NAV':
            if not self.nav.isTaskComplete():
                return                      # 도크앞으로 아직 이동 중
            if self.nav.getResult() != TaskResult.SUCCEEDED:
                # 도크앞 도착 실패 — 그래도 dock 액션 시도 (근처면 붙을 수 있음).
                self.get_logger().warn('도크앞 이동 실패 — dock 액션 그대로 시도')
            self.nav.dock_send_goal()
            self.dock_phase = 'DOCK'
            self.get_logger().info('도크앞 도착 — dock 액션(정밀 도킹) 시작')
        elif self.dock_phase == 'DOCK':
            if not self.nav.isDockComplete():
                return                      # 도킹 액션 진행 중
            self.dock_phase = None
            self.state = 'DOCKED'           # ★ 여기서만 DOCKED — central 교대 트리거
            self.get_logger().info('도킹 완료 — DOCKED')
        elif self.dock_phase == 'UNDOCK':
            if not self.nav.isUndockComplete():
                return                      # 언도킹 액션 진행 중
            self.dock_phase = None
            self.get_logger().info('언도킹 완료 — 순찰 시작')
            self.start_patrol()             # 도크 밖으로 나왔으니 순찰 개시

    def hold_cb(self, msg):
        """detector가 opening 처리 시작(True)/끝(False)을 알린다.

        True면 진행 중 순찰을 멈추고, False면 다시 순찰을 시작한다.
        """
        self.hold = msg.data
        if self.hold:
            self.stop_patrol()              # 처리 시작 — 순찰 멈춤
            self.get_logger().info('순찰 정지 (opening 처리)')
        elif self.state == 'PATROLLING':
            self.get_logger().info('순찰 재개')
            self.start_patrol()             # 처리 끝 — 다시 순찰

    def battery_cb(self, msg):
        if math.isnan(msg.percentage):
            return                      # 배터리 값 불명(NaN) — 무시
        self.battery = int(msg.percentage * 100) if msg.percentage <= 1.0 \
            else int(msg.percentage)
        if self.battery < self.threshold and self.state == 'PATROLLING':
            self.state = 'RETURNING'
            self.get_logger().info(f'배터리 {self.battery}% < 임계 — 복귀')
            self.start_docking()            # 도크앞 이동→도킹, 완료 시 DOCKED

    def target_cb(self, msg):
        """TRACK/HERD·trap 설치가 준 목표로 Nav2 주행 (논블로킹, goToPose).

        target_pose가 갱신되면 새 goal이 이전 goal을 선점(preempt)해 방향을
        바꾼다 — 추적처럼 목표가 움직여도 계속 따라간다. goToPose는 도킹에서
        쓰는 것과 같은 논블로킹 호출이라 콜백에서 그대로 부른다.
        도킹/언도킹 중엔 도크 이동과 충돌하므로 무시한다.
        """
        if self.dock_phase is not None:
            return
        self.nav.goToPose(msg)
        self.get_logger().info(
            f'목표 주행: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})',
            throttle_duration_sec=1.0)

    def report(self):
        self.status_pub.publish(String(data=fleet_msg.status(
            self.ns, self.state, self.battery)))


def main():
    rclpy.init()
    node = RobotAgent()
    # nav(TurtleBot4Navigator)도 자체 노드라 같은 executor에 넣어 함께 돌린다.
    # 안 그러면 followWaypoints/dock 응답·피드백이 처리되지 않는다.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.nav)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.nav.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


class _FakeNav:
    """_dock_tick 폴링 전이 검증용 가짜 nav — 스크립트대로 완료 여부를 답한다."""
    def __init__(self, task_done, task_result, dock_done, undock_done):
        self._task_done, self._result = task_done, task_result
        self._dock_done, self._undock_done = dock_done, undock_done
        self.dock_sent = self.undock_sent = False
        self.goto_pose = None           # goToPose로 받은 마지막 목표 (target_cb 검증)
    def isTaskComplete(self): return self._task_done
    def getResult(self): return self._result
    def dock_send_goal(self): self.dock_sent = True
    def isDockComplete(self): return self._dock_done
    def undock_send_goal(self): self.undock_sent = True
    def isUndockComplete(self): return self._undock_done
    def goToPose(self, pose): self.goto_pose = pose


class _FakeAgent:
    """_dock_tick/target_cb만 떼어 돌리기 위한 최소 스텁 (Node 없이)."""
    _dock_tick = RobotAgent._dock_tick
    target_cb = RobotAgent.target_cb
    def __init__(self, nav, phase):
        self.nav, self.dock_phase, self.state = nav, phase, 'RETURNING'
        self.patrol_started = False
    def get_logger(self):
        import types
        return types.SimpleNamespace(info=lambda *a, **k: None,
                                     warn=lambda *a, **k: None)
    def start_patrol(self): self.patrol_started = True


def _self_check():
    # 도킹: NAV에서 이동 미완이면 그대로 대기
    a = _FakeAgent(_FakeNav(False, None, False, False), 'NAV')
    a._dock_tick()
    assert a.dock_phase == 'NAV' and not a.nav.dock_sent
    # NAV 이동 성공 → dock 액션 시작
    a = _FakeAgent(_FakeNav(True, TaskResult.SUCCEEDED, False, False), 'NAV')
    a._dock_tick()
    assert a.dock_phase == 'DOCK' and a.nav.dock_sent
    # dock 액션 완료 → DOCKED 방송, 폴링 종료
    a = _FakeAgent(_FakeNav(True, TaskResult.SUCCEEDED, True, False), 'DOCK')
    a._dock_tick()
    assert a.dock_phase is None and a.state == 'DOCKED'
    # dock 액션 진행 중이면 DOCKED 아직 아님 (조기 방송 방지)
    a = _FakeAgent(_FakeNav(True, TaskResult.SUCCEEDED, False, False), 'DOCK')
    a._dock_tick()
    assert a.dock_phase == 'DOCK' and a.state == 'RETURNING'
    # 언도킹 완료 → 순찰 시작
    a = _FakeAgent(_FakeNav(True, None, False, True), 'UNDOCK')
    a._dock_tick()
    assert a.dock_phase is None and a.patrol_started
    # 언도킹 진행 중이면 순찰 아직 안 시작
    a = _FakeAgent(_FakeNav(True, None, False, False), 'UNDOCK')
    a._dock_tick()
    assert a.dock_phase == 'UNDOCK' and not a.patrol_started

    # target_cb: 도킹 중 아니면 goToPose로 목표 주행
    import types
    msg = types.SimpleNamespace(pose=types.SimpleNamespace(
        position=types.SimpleNamespace(x=1.0, y=2.0)))
    a = _FakeAgent(_FakeNav(True, None, False, False), None)
    a.target_cb(msg)
    assert a.nav.goto_pose is msg
    # 도킹/언도킹 중이면 목표 주행 무시 (도크 이동과 충돌 방지)
    a = _FakeAgent(_FakeNav(True, None, False, False), 'NAV')
    a.target_cb(msg)
    assert a.nav.goto_pose is None
    print('robot_agent self-check ok')


if __name__ == '__main__':
    import sys
    _self_check() if '--check' in sys.argv else main()
