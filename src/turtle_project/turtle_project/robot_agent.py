"""로봇 1대 제어 — 순찰 주행(Nav2)·dock/undock·배터리 감시·상태보고.

namespace 파라미터로 로봇마다 실행 (robot4/robot6). /fleet/command에서 자기
명령만 필터해 실행하고, /fleet/status로 상태를 보고한다.

PATROL/UNDOCK을 받으면 waypoint YAML을 FollowWaypoints로 무한 순찰한다
(patrol_zigzag 로직 통합). 배터리<임계 또는 DOCK 명령이면 도크 앞 좌표로
이동해 정밀 도킹한다. dock/undock/navigate는 모두 논블로킹 폴링(patrol_tick)
으로 진행한다 — 순찰의 isTaskComplete 폴링과 같은 방식.
"""
import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
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
        self.threshold = self.declare_parameter('battery_threshold', 20).value
        period = self.declare_parameter('battery_check_period', 10.0).value
        self.wp_file = self.declare_parameter(
            'waypoints', os.path.join(
                get_package_share_directory('turtle_project'),
                'resource', 'patrol_waypoints.yaml')).value
        # 도크 '앞' 접근점 (map 절대좌표) — 여기까지 Nav2로 간 뒤 dock() 정밀도킹.
        # 로봇/현장마다 실측 필요. yaw는 도크를 바라보는 방향(도)이어야 한다.
        self.dock_x = self.declare_parameter('dock_x', 0.0).value
        self.dock_y = self.declare_parameter('dock_y', 0.0).value
        self.dock_yaw = self.declare_parameter('dock_yaw_deg', 0.0).value

        # detector 사망 대비 — hold heartbeat(1초 주기 True)가 이 시간 넘게
        # 끊기면 hold를 자동 해제하고 순찰을 재개한다.
        self.hold_lease = self.declare_parameter('hold_lease_sec', 3.0).value
        # herding_controller_dual은 10Hz로 target_pose를 쏜다 — 매번 goToPose를
        # 부르면 Nav2가 초당 10번 선점당해 경로를 다시 짜다 제자리에서 떤다.
        # 직전에 실제로 보낸 goal과 이만큼(m) 이상 차이날 때만 재전송한다.
        # 시간 기준 대신 거리 기준 — 빠르게 움직이는 국면에서 시간 기준은 반응이 늦다.
        self.goal_resend_dist = self.declare_parameter('goal_resend_dist', 0.18).value

        self.state = 'IDLE'
        self.battery = 100

        # TurtleBot4Navigator는 BasicNavigator 상속 — 순찰(followWaypoints)에
        # dock/undock 액션이 딸려온다. 자체 노드라 main에서 같은 executor에 돌린다.
        self.nav = TurtleBot4Navigator(namespace=f'/{self.ns}')
        self.poses = None       # 로드된 순찰 waypoint (첫 PATROL 때 lazy 로드)
        self.patrolling = False  # followWaypoints 진행 중
        self.driving = False    # target_pose로 받은 goal(접근/추적) 주행 중
        self.lap_from = 0       # 다음 순찰이 시작할 waypoint 인덱스 (중단 지점)
        self.lap_start = 0      # 지금 도는 바퀴가 시작한 인덱스 (피드백 오프셋)
        self.hold = False       # opening 처리 중 순찰 정지 (detector가 신호)
        self.hold_at = 0.0      # 마지막 hold=True 수신 시각 (lease 판정용)
        # 도킹/언도킹 논블로킹 진행 단계. None=진행 중 아님.
        # 'NAV'=도크앞 이동 중, 'DOCK'=dock 액션 중, 'UNDOCK'=undock 액션 중.
        self.dock_phase = None
        # 언도킹 완료 후 할 일: 'PATROL'(순찰) | 'DOCK'(바로 복귀) | None(추적/
        # 몰이 — target_pose 대기). 도킹된 로봇에 HERD/TRACK이 와도 먼저
        # 빠져나온 뒤 goal을 받게 한다 (도크에 붙은 채 HERDING 방지).
        self.after_undock = None
        # 언도킹 중 받은 target_pose — 버리면 detector가 60초 timeout까지 기다
        # 리므로(SWEEP 첫 구멍 goal 등) 보류했다가 언도킹 완료 후 주행한다.
        self.pending_target = None
        self._last_goal_xy = None  # 마지막으로 실제 전송한 goal (goal_resend_dist 판정용)

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
        # detector가 탐색 회전 직전 진행 중 goal을 끊어달라는 신호 —
        # 안 끊으면 Nav2 controller와 회전 cmd_vel이 동시에 발행돼 싸운다.
        self.create_subscription(Bool, 'cancel_drive',
                                 self.cancel_cb, 10)
        self.create_timer(period, self.report)
        # 순찰 진행 감시 — 한 바퀴 끝나면 다음 바퀴 재시작 (무한 순찰).
        self.create_timer(1.0, self.patrol_tick)
        self.get_logger().info(f'{self.ns} agent 시작 — 임계 {self.threshold}%')

    def command_cb(self, msg):
        robot, cmd = fleet_msg.parse_command(msg.data)
        if robot != self.ns:
            return                          # 내 명령 아님
        self.get_logger().info(f'명령 수신: {cmd}')
        # 도크에 붙어 있으면 Create3가 cmd_vel을 무시한다 — 주행 명령은 뭐든
        # 언도킹이 먼저다. state('DOCKED')는 이 agent가 직접 도킹시켰을 때만
        # 찍히므로 도크 센서를 본다 (도크 위에서 agent를 새로 띄운 경우도 잡힘).
        was_docked = bool(self.nav.is_docked)
        # DOCK/PATROL은 여기서 상태를 안 바꾼다 — DOCKED는 도킹 완료(_dock_tick),
        # PATROLLING은 순찰 goal 발행(_send_lap)에서만. 성공 전에 방송하면
        # central이 조기에 교대/커버리지 판단을 잘못 내린다.
        self.state = {'UNDOCK': 'UNDOCKING',
                      'TRACK': 'TRACKING', 'HERD': 'HERDING', 'SWEEP': 'SWEEPING',
                      'STOP': 'IDLE'}.get(cmd, self.state)
        if cmd == 'UNDOCK' or (cmd == 'PATROL' and was_docked):
            self.state = 'UNDOCKING'
            self.after_undock = 'PATROL'
            self.start_undocking()          # 도크에서 빠져나온 뒤 순찰 시작
        elif cmd == 'PATROL':
            self.start_patrol()             # 이미 필드 위 — 바로 순찰
        elif cmd == 'DOCK':
            self.state = 'RETURNING'        # 도킹 완료 시 _dock_tick이 DOCKED로
            if self.dock_phase == 'UNDOCK':
                self.after_undock = 'DOCK'  # 언도킹 중 복귀 명령 — 끝나면 도킹
            else:
                self.start_docking()
        elif cmd in ('TRACK', 'HERD', 'SWEEP'):
            # 순찰 중단 — 추적/점검/몰이 주행은 target_pose가 준다(아래 target_cb).
            # detector가 SWEEP면 구멍 접근 goal을, TRACK/HERD면 추적/몰이 goal을 쏜다.
            self.stop_patrol()
            if self.dock_phase == 'UNDOCK':
                self.after_undock = None    # 언도킹 중 — 완료 후 순찰 대신 goal 대기
            elif was_docked:
                # 도크에 붙어있으면 먼저 빠져나온다 (안 그러면 몸은 도크에
                # 붙은 채 상태만 HERDING이 된다).
                self.after_undock = None
                self.start_undocking()
        elif cmd == 'STOP':
            self.stop_patrol()
            self.pending_target = None  # 언도킹 뒤 보류 goal도 없던 일로
            if self.driving:
                # 접근/추적 goal도 끊는다 — 순찰만 끊으면 Nav2가 계속 주행한다.
                self.nav.cancelTask()
                self.driving = False
                self._last_goal_xy = None

    def start_patrol(self):
        """waypoint 순찰 시작. 이미 도는 중이면 무시. waypoint는 첫 호출 때 로드."""
        if self.patrolling:
            return
        if self.poses is None:
            try:
                frame_id, wps = load_waypoints(self.wp_file)
            except FileNotFoundError:
                self.get_logger().error(f'waypoint 파일 없음: {self.wp_file}')
                self.state = 'IDLE'     # 순찰 못 시작 — PATROLLING 보고 금지
                return
            if not wps:
                self.get_logger().error('waypoint가 비어있음')
                self.state = 'IDLE'
                return
            self.poses = [to_pose(self.nav, frame_id, w) for w in wps]
        self._send_lap()

    def _send_lap(self):
        """순찰 goal 발행. 중단됐던 지점(lap_from)부터 이어 돈다.

        opening 처리로 순찰이 끊길 때마다 0번부터 다시 돌면, 구멍이 앞쪽에
        몰려 있을 경우 맵 뒷부분에 영영 못 간다. 중단 지점부터 보내 남은
        구간을 마저 돌고, 그 바퀴가 끝나면 다음 바퀴는 전체를 다시 돈다.
        """
        now = self.nav.get_clock().now().to_msg()
        for p in self.poses:
            p.header.stamp = now
        self.lap_start = self.lap_from   # 피드백 인덱스는 이 오프셋 기준이다
        self.nav.followWaypoints(self.poses[self.lap_from:])
        self.lap_from = 0           # 이번 바퀴 소진 — 완주하면 다음은 처음부터
        self.driving = False        # 순찰이 goal 주인 — 접근 goal 감시 종료
        self.patrolling = True
        self.state = 'PATROLLING'       # goal 발행 성공 — 여기서만 PATROLLING

    def stop_patrol(self):
        """순찰 취소 — 어디까지 갔는지 기록해 재개 때 이어붙인다.

        피드백의 current_waypoint는 이번에 보낸 목록 기준이라 lap_start를
        더해 절대 인덱스로 되돌린다. feedback은 마지막 액션의 것이라
        followWaypoints가 아니면 current_waypoint가 없다 — 그땐 0으로 둔다.
        """
        if self.patrolling:
            self.lap_from = self.lap_start + getattr(
                self.nav.feedback, 'current_waypoint', 0)
            self.nav.cancelTask()
            # 취소 '요청'만 보내고 리턴하면 안 된다 — waypoint_follower는 취소
            # 처리 때 NavigateToPose를 전부 취소(cancel_all)하므로, 그 전에
            # 보낸 내 goal(접근/도크앞)까지 같이 죽는다. FollowWaypoints의
            # CANCELED 결과가 돌아온 뒤에야 새 goal을 보내도 안전하다.
            deadline = time.monotonic() + 3.0   # 실측 ~0.9s + WiFi 여유
            while not self.nav.isTaskComplete():
                if time.monotonic() > deadline:
                    self.get_logger().warn('순찰 취소 결과 대기 timeout — 진행')
                    break
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
        """undock 액션으로 도크에서 빠져나온다 (논블로킹).

        도크에 붙은 채 followWaypoints/goToPose를 보내면 안 먹으므로 먼저
        빠져나온다. 완료 후 할 일은 after_undock이 정한다 (_dock_tick).
        이미 도크 밖이면 undock 액션이 즉시 완료로 처리된다.
        """
        self.stop_patrol()
        self.pending_target = None      # 이전 언도킹의 잔여 goal 제거
        self.nav.undock_send_goal()
        self.dock_phase = 'UNDOCK'
        self.get_logger().info('언도킹 시작 — 도크에서 빠져나오는 중')

    def patrol_tick(self):
        """1초 폴링 — 도킹/언도킹 진행 우선, 아니면 순찰 한 바퀴 이어돌기.

        dock/undock 액션이 블로킹이라 콜백에서 직접 못 부른다. 대신 여기서
        완료 여부만 폴링해 단계를 넘긴다 (순찰 isTaskComplete 폴링과 같은 방식).
        """
        # hold lease — detector가 죽어 heartbeat가 끊기면 자동 해제 후 순찰 재개.
        if self.hold and self._now_s() - self.hold_at > self.hold_lease:
            self.hold = False
            self.get_logger().warn('patrol_hold lease 만료 — detector 무응답, 순찰 재개')
            if self.state == 'PATROLLING':
                self.start_patrol()
        if self.dock_phase is not None:
            self._dock_tick()
            return
        if self.driving:            # 접근/추적 goal 진행 중 — 성패만 찍고 빠진다.
            if not self.nav.isTaskComplete():
                return
            self.driving = False
            self._last_goal_xy = None
            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('목표 도착')
            else:
                # 조용히 묻히면 detector가 60초 timeout 날 때까지 원인을 못 본다.
                self.get_logger().warn(f'목표 주행 실패 ({result}) — Nav2가 goal을 '
                                       '거부/포기함 (경로 없음·costmap 막힘 등)')
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
            # isDockComplete는 실패한 goal도 '완료'로 주므로 실제 도킹 여부는
            # dock 센서(is_docked, DockStatus 구독)로 확인한다.
            if self.nav.is_docked:
                self.state = 'DOCKED'       # ★ 여기서만 DOCKED — central 교대 트리거
                self.get_logger().info('도킹 완료 — DOCKED')
            else:
                self.state = 'IDLE'         # 실패를 DOCKED로 축약 금지
                self.get_logger().warn('도킹 실패 — IDLE (재시도는 DOCK 명령)')
        elif self.dock_phase == 'UNDOCK':
            if not self.nav.isUndockComplete():
                return                      # 언도킹 액션 진행 중
            if self.nav.is_docked:
                # 액션은 성공을 반환했는데 도크 센서상 아직 도크 위 — 이대로
                # goal을 보내면 Create3가 도킹 중이라 cmd_vel을 무시해 로봇이
                # 안 움직인다. 진짜로 빠져나올 때까지 언도킹을 다시 보낸다.
                # ponytail: 무한 재시도 — 도크에 물리적으로 걸린 경우 수동 개입
                # 필요, 횟수 제한은 필요해지면.
                self.get_logger().warn('언도킹 액션은 완료됐지만 센서상 아직 도크 위 '
                                       '— 언도킹 재시도')
                self.nav.undock_send_goal()
                return
            self.dock_phase = None
            if self.after_undock == 'PATROL':
                self.get_logger().info('언도킹 완료 — 순찰 시작')
                self.start_patrol()
            elif self.after_undock == 'DOCK':
                self.get_logger().info('언도킹 완료 — 복귀 도킹')
                self.start_docking()
            elif self.pending_target is not None:
                self.get_logger().info('언도킹 완료 — 보류한 goal 주행')
                msg, self.pending_target = self.pending_target, None
                self.target_cb(msg)         # dock_phase가 None이라 바로 주행된다
            else:                           # TRACK/HERD — goal은 target_pose로 온다
                self.get_logger().info('언도킹 완료 — target_pose 대기 (추적/몰이)')

    def cancel_cb(self, msg):
        """접근/추적 goal만 취소 (순찰·도킹은 안 건드림). detector 회전 준비용."""
        if msg.data and self.driving:
            self.nav.cancelTask()
            self.driving = False
            self._last_goal_xy = None
            self.get_logger().info('주행 goal 취소 (cancel_drive)')

    def hold_cb(self, msg):
        """detector가 opening 처리 시작(True)/끝(False)을 알린다.

        True는 처리 중 1초마다 heartbeat로 재발행된다 — 수신 시각(hold_at)을
        갱신해 lease를 유지한다. 전이(False→True/True→False)에만 정지/재개.
        """
        self.hold_at = self._now_s()
        was, self.hold = self.hold, msg.data
        if self.hold and not was:
            self.stop_patrol()              # 처리 시작 — 순찰 멈춤
            self.get_logger().info('순찰 정지 (opening 처리)')
        elif was and not self.hold and self.state == 'PATROLLING':
            self.get_logger().info('순찰 재개')
            self.start_patrol()             # 처리 끝 — 다시 순찰

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

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
        도킹 중엔 도크 이동과 충돌하므로 무시하고, 언도킹 중엔 보류했다가
        완료 후 주행한다 (버리면 SWEEP 첫 구멍 goal이 사라져 60초 낭비).

        순찰 goal을 반드시 '먼저' 끊고 내 goal을 보낸다. cancelTask는 마지막에
        보낸 goal 하나를 끊으므로, 순서가 뒤집혀 hold_cb가 나중에 실행되면
        방금 보낸 이 goal이 취소된다 (rclpy는 구독 생성순으로 콜백을 돌리는데
        target_pose가 patrol_hold보다 먼저 만들어져 실제로 뒤집힌다).
        여기서 먼저 끊어두면 patrolling=False라 뒤늦은 stop_patrol이 no-op이 된다.
        """
        if self.dock_phase is not None:
            if self.dock_phase == 'UNDOCK':
                self.pending_target = msg   # 언도킹 끝나면 _dock_tick이 주행
            return
        x, y = msg.pose.position.x, msg.pose.position.y
        if self.driving and self._last_goal_xy is not None:
            d = math.hypot(x - self._last_goal_xy[0], y - self._last_goal_xy[1])
            if d < self.goal_resend_dist:
                return           # 직전 goal과 충분히 가까움 — 재전송 생략(재계획 폭주 방지)
        self.stop_patrol()
        self.nav.goToPose(msg)
        self.driving = True         # 결과 감시 대상 (patrol_tick이 성패를 찍는다)
        self._last_goal_xy = (x, y)
        self.get_logger().info(
            f'목표 주행: ({x:.2f}, {y:.2f})', throttle_duration_sec=1.0)

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
        # 진행 중 goal 회수 — 안 끊으면 agent가 죽어도 Nav2가 마지막 goal로
        # 계속 주행한다 (goal 주인은 bt_navigator라 클라이언트 사망과 무관).
        if getattr(node.nav, 'goal_handle', None) is not None:
            fut = node.nav.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node.nav, fut, timeout_sec=2.0)
        node.nav.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


class _FakeNav:
    """_dock_tick 폴링 전이 검증용 가짜 nav — 스크립트대로 완료 여부를 답한다."""
    def __init__(self, task_done, task_result, dock_done, undock_done,
                 docked=True):
        self._task_done, self._result = task_done, task_result
        self._dock_done, self._undock_done = dock_done, undock_done
        self.is_docked = docked         # dock 센서값 (DockStatus 구독 대역)
        self.dock_sent = self.undock_sent = False
        self.goto_pose = None           # goToPose로 받은 마지막 목표 (target_cb 검증)
        self.calls = []                 # 액션 호출 순서 — cancel이 goto를 죽였는지 검증
        self.feedback = None            # followWaypoints 피드백 (current_waypoint)
        self.sent = None                # followWaypoints로 보낸 waypoint 목록
    def followWaypoints(self, poses): self.sent = list(poses)
    def get_clock(self):
        import types
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: 0))
    def isTaskComplete(self): return self._task_done
    def getResult(self): return self._result
    def dock_send_goal(self): self.dock_sent = True
    def isDockComplete(self): return self._dock_done
    def undock_send_goal(self): self.undock_sent = True
    def isUndockComplete(self): return self._undock_done
    def cancelTask(self):
        self.calls.append('cancel')
        self._task_done = True      # 취소하면 곧 CANCELED 결과가 온다 — 대기 종료
    def goToPose(self, pose):
        self.goto_pose = pose
        self.calls.append('goto')


class _FakeAgent:
    """_dock_tick/target_cb/command_cb/patrol_tick만 떼어 돌리는 최소 스텁."""
    _dock_tick = RobotAgent._dock_tick
    target_cb = RobotAgent.target_cb
    command_cb = RobotAgent.command_cb
    patrol_tick = RobotAgent.patrol_tick
    hold_cb = RobotAgent.hold_cb
    cancel_cb = RobotAgent.cancel_cb
    stop_patrol = RobotAgent.stop_patrol      # 진짜 취소 로직으로 순서 검증
    _send_lap = RobotAgent._send_lap
    def __init__(self, nav, phase, state='RETURNING'):
        self.nav, self.dock_phase, self.state = nav, phase, state
        self.ns = 'robot4'
        self.after_undock = None
        self.pending_target = None
        self._last_goal_xy = None
        self.goal_resend_dist = 0.18
        self.hold, self.hold_at, self.hold_lease = False, 0.0, 3.0
        self.patrolling = self.driving = False
        self.lap_from = self.lap_start = 0
        self.poses = None
        self.patrol_started = self.docking_started = self.undocking_started = False
    def _now_s(self): return 10.0       # 고정 현재시각 (lease 판정 검증용)
    def get_logger(self):
        import types
        return types.SimpleNamespace(info=lambda *a, **k: None,
                                     warn=lambda *a, **k: None)
    def start_patrol(self): self.patrol_started = True
    def start_docking(self): self.docking_started = True
    def start_undocking(self):
        self.undocking_started = True
        self.dock_phase = 'UNDOCK'


def _self_check():
    # 도킹: NAV에서 이동 미완이면 그대로 대기
    a = _FakeAgent(_FakeNav(False, None, False, False), 'NAV')
    a._dock_tick()
    assert a.dock_phase == 'NAV' and not a.nav.dock_sent
    # NAV 이동 성공 → dock 액션 시작
    a = _FakeAgent(_FakeNav(True, TaskResult.SUCCEEDED, False, False), 'NAV')
    a._dock_tick()
    assert a.dock_phase == 'DOCK' and a.nav.dock_sent
    # dock 액션 완료 + 실제 도킹됨 → DOCKED 방송, 폴링 종료
    a = _FakeAgent(_FakeNav(True, TaskResult.SUCCEEDED, True, False), 'DOCK')
    a._dock_tick()
    assert a.dock_phase is None and a.state == 'DOCKED'
    # dock 액션 '완료'지만 실제로는 도킹 안 됨(실패) → DOCKED 금지, IDLE (문제 3)
    a = _FakeAgent(_FakeNav(True, TaskResult.SUCCEEDED, True, False,
                            docked=False), 'DOCK')
    a._dock_tick()
    assert a.dock_phase is None and a.state == 'IDLE'
    # dock 액션 진행 중이면 DOCKED 아직 아님 (조기 방송 방지)
    a = _FakeAgent(_FakeNav(True, TaskResult.SUCCEEDED, False, False), 'DOCK')
    a._dock_tick()
    assert a.dock_phase == 'DOCK' and a.state == 'RETURNING'
    # 언도킹 완료(after=PATROL, UNDOCK 명령) → 순찰 시작
    a = _FakeAgent(_FakeNav(True, None, False, True, docked=False), 'UNDOCK')
    a.after_undock = 'PATROL'
    a._dock_tick()
    assert a.dock_phase is None and a.patrol_started
    # 언도킹 진행 중이면 순찰 아직 안 시작
    a = _FakeAgent(_FakeNav(True, None, False, False), 'UNDOCK')
    a.after_undock = 'PATROL'
    a._dock_tick()
    assert a.dock_phase == 'UNDOCK' and not a.patrol_started
    # 언도킹 액션은 '성공'인데 도크 센서상 아직 도크 위 → goal/순찰 대신 재시도
    # (도킹 상태에선 Create3가 cmd_vel을 무시하므로 goal을 보내봤자 안 움직인다)
    a = _FakeAgent(_FakeNav(True, None, False, True), 'UNDOCK')  # docked=True
    a.after_undock = 'PATROL'
    a._dock_tick()
    assert a.dock_phase == 'UNDOCK' and a.nav.undock_sent \
        and not a.patrol_started
    # 언도킹 완료(after=None, TRACK/HERD) → 순찰 없이 target_pose 대기
    a = _FakeAgent(_FakeNav(True, None, False, True, docked=False), 'UNDOCK')
    a._dock_tick()
    assert a.dock_phase is None and not a.patrol_started
    # 언도킹 완료(after=DOCK) → 바로 복귀 도킹
    a = _FakeAgent(_FakeNav(True, None, False, True, docked=False), 'UNDOCK')
    a.after_undock = 'DOCK'
    a._dock_tick()
    assert a.dock_phase is None and a.docking_started

    import types
    # UNDOCK 수신 → 즉시 PATROLLING이 아니라 UNDOCKING 보고 (문제 3)
    a = _FakeAgent(_FakeNav(True, None, False, False), None, state='DOCKED')
    a.command_cb(types.SimpleNamespace(data='robot4:UNDOCK'))
    assert a.state == 'UNDOCKING' and a.after_undock == 'PATROL' \
        and a.undocking_started
    # 도킹된 로봇에 HERD → 먼저 언도킹, 완료 후 goal 대기 모드 (문제 1 핵심)
    a = _FakeAgent(_FakeNav(True, None, False, False), None, state='DOCKED')
    a.command_cb(types.SimpleNamespace(data='robot4:HERD'))
    assert a.undocking_started and a.after_undock is None and a.state == 'HERDING'
    # 도킹된 로봇에 SWEEP(B 구멍 순회) → 언도킹 먼저, 상태 SWEEPING
    a = _FakeAgent(_FakeNav(True, None, False, False), None, state='DOCKED')
    a.command_cb(types.SimpleNamespace(data='robot4:SWEEP'))
    assert a.undocking_started and a.after_undock is None and a.state == 'SWEEPING'
    # 필드 위(도크 센서 off) 로봇에 TRACK → 언도킹 없이 역할 전환만
    a = _FakeAgent(_FakeNav(True, None, False, False, docked=False), None,
                   state='PATROLLING')
    a.command_cb(types.SimpleNamespace(data='robot4:TRACK'))
    assert not a.undocking_started and a.state == 'TRACKING'
    # 도크 위인데 PATROL → 도크에선 못 움직이므로 언도킹 먼저, 끝나면 순찰
    a = _FakeAgent(_FakeNav(True, None, False, False), None, state='IDLE')
    a.command_cb(types.SimpleNamespace(data='robot4:PATROL'))
    assert a.undocking_started and a.after_undock == 'PATROL' \
        and a.state == 'UNDOCKING' and not a.patrol_started
    # 도크 밖에서 PATROL → 바로 순찰 (언도킹 없음)
    a = _FakeAgent(_FakeNav(True, None, False, False, docked=False), None,
                   state='IDLE')
    a.command_cb(types.SimpleNamespace(data='robot4:PATROL'))
    assert not a.undocking_started and a.patrol_started
    # 언도킹 중 DOCK 명령 → 도킹은 언도킹 완료 후로 예약
    a = _FakeAgent(_FakeNav(False, None, False, False), 'UNDOCK')
    a.command_cb(types.SimpleNamespace(data='robot4:DOCK'))
    assert a.after_undock == 'DOCK' and not a.docking_started
    # 남의 명령은 무시
    a = _FakeAgent(_FakeNav(True, None, False, False), None, state='DOCKED')
    a.command_cb(types.SimpleNamespace(data='robot6:HERD'))
    assert not a.undocking_started and a.state == 'DOCKED'
    # STOP은 순찰뿐 아니라 접근/추적 goal(driving)도 끊는다
    a = _FakeAgent(_FakeNav(False, None, False, False, docked=False), None,
                   state='TRACKING')
    a.driving = True
    a.command_cb(types.SimpleNamespace(data='robot4:STOP'))
    assert a.nav.calls == ['cancel'] and not a.driving and a.state == 'IDLE'
    # STOP은 언도킹 중 보류해둔 goal도 폐기 (완료 후 돌진 방지)
    a = _FakeAgent(_FakeNav(False, None, False, False), 'UNDOCK')
    a.pending_target = object()
    a.command_cb(types.SimpleNamespace(data='robot4:STOP'))
    assert a.pending_target is None

    # cancel_drive: 추적 goal 주행 중이면 취소, 아니면 no-op (순찰은 안 건드림)
    a = _FakeAgent(_FakeNav(False, None, False, False), None, state='TRACKING')
    a.driving = True
    a.cancel_cb(types.SimpleNamespace(data=True))
    assert a.nav.calls == ['cancel'] and not a.driving
    a = _FakeAgent(_FakeNav(False, None, False, False), None, state='PATROLLING')
    a.patrolling = True
    a.cancel_cb(types.SimpleNamespace(data=True))
    assert a.nav.calls == [] and a.patrolling      # 주행 중 아님 — 아무것도 안 함

    # hold lease 만료 → detector 무응답 시 자동 해제 + 순찰 재개 (문제 4)
    a = _FakeAgent(_FakeNav(False, None, False, False), None, state='PATROLLING')
    a.hold, a.hold_at = True, 0.0       # 지금(10.0)보다 10초 전 — lease(3s) 초과
    a.patrol_tick()
    assert not a.hold and a.patrol_started
    # lease 이내면 hold 유지 (정상 heartbeat 수신 중)
    a = _FakeAgent(_FakeNav(False, None, False, False), None, state='PATROLLING')
    a.hold, a.hold_at = True, 8.5       # 1.5초 전 — lease 이내
    a.patrol_tick()
    assert a.hold and not a.patrol_started

    # target_cb: 도킹 중 아니면 goToPose로 목표 주행
    msg = types.SimpleNamespace(pose=types.SimpleNamespace(
        position=types.SimpleNamespace(x=1.0, y=2.0)))
    a = _FakeAgent(_FakeNav(True, None, False, False), None)
    a.target_cb(msg)
    assert a.nav.goto_pose is msg
    # goal_resend_dist: 직전 goal과 가까운 재발행(10Hz 몰이 등)은 재전송 생략
    a.driving = True
    a.nav.goto_pose, a.nav.calls = None, []
    near = types.SimpleNamespace(pose=types.SimpleNamespace(
        position=types.SimpleNamespace(x=1.05, y=2.05)))   # ~0.07m — 문턱(0.18) 안
    a.target_cb(near)
    assert a.nav.goto_pose is None and a.nav.calls == []
    # 문턱 넘게 움직이면 재전송
    far = types.SimpleNamespace(pose=types.SimpleNamespace(
        position=types.SimpleNamespace(x=1.5, y=2.0)))      # 0.5m — 문턱 밖
    a.target_cb(far)
    assert a.nav.goto_pose is far
    # 도킹 중이면 목표 주행 무시 (도크 이동과 충돌 방지) — 보류도 안 함
    a = _FakeAgent(_FakeNav(True, None, False, False), 'NAV')
    a.target_cb(msg)
    assert a.nav.goto_pose is None and a.pending_target is None
    # 언도킹 중 받은 goal은 버리지 않고 보류 → 언도킹 완료 시 주행 (문제 1)
    a = _FakeAgent(_FakeNav(True, None, False, True, docked=False), 'UNDOCK')
    a.target_cb(msg)
    assert a.nav.goto_pose is None and a.pending_target is msg
    a._dock_tick()                          # 언도킹 완료 폴링
    assert a.nav.goto_pose is msg and a.pending_target is None and a.driving
    # 보류 goal 없이 언도킹 완료(after=None) → 기존대로 target_pose 대기
    a = _FakeAgent(_FakeNav(True, None, False, True, docked=False), 'UNDOCK')
    a._dock_tick()
    assert a.nav.goto_pose is None and not a.patrol_started

    # 접근 goal이 순찰 취소에 안 죽는지 — cancelTask는 마지막 goal 하나만 끊으므로
    # 'goto' 뒤에 'cancel'이 오면 접근이 취소된 것이다. 두 콜백 순서 모두 검증.
    for first_target in (True, False):      # rclpy 구독 생성순상 target이 먼저다
        a = _FakeAgent(_FakeNav(False, None, False, False), None,
                       state='PATROLLING')
        a.patrolling = True                 # 순찰 중 opening 발견
        hold = types.SimpleNamespace(data=True)
        (a.target_cb(msg), a.hold_cb(hold)) if first_target \
            else (a.hold_cb(hold), a.target_cb(msg))
        assert a.nav.calls == ['cancel', 'goto'], a.nav.calls
        assert not a.patrolling and a.hold and a.driving

    # 접근 goal 실패는 조용히 묻히면 안 된다 (detector 60초 timeout까지 깜깜)
    a = _FakeAgent(_FakeNav(True, TaskResult.FAILED, False, False), None,
                   state='PATROLLING')
    a.driving, a.hold, a.hold_at = True, True, 9.5   # lease 이내 heartbeat 수신 중
    a.patrol_tick()
    assert not a.driving and not a.patrol_started    # 감시 종료, 순찰 재시작 안 함
    # 아직 주행 중이면 결과를 안 본다
    a = _FakeAgent(_FakeNav(False, None, False, False), None, state='PATROLLING')
    a.driving, a.hold, a.hold_at = True, True, 9.5
    a.patrol_tick()
    assert a.driving

    # 순찰 재개는 '중단 지점부터' — 0번부터 다시 돌면 맵 뒷부분에 영영 못 간다
    def _wp(n):     # stamp만 갱신되는 최소 pose 스텁
        return [types.SimpleNamespace(header=types.SimpleNamespace(stamp=None))
                for _ in range(n)]
    a = _FakeAgent(_FakeNav(False, None, False, False), None, state='PATROLLING')
    a.poses = _wp(10)
    a._send_lap()                                   # 1바퀴: 전체 10개
    assert len(a.nav.sent) == 10 and a.lap_start == 0
    a.nav.feedback = types.SimpleNamespace(current_waypoint=4)
    a.stop_patrol()                                 # 4번에서 opening 처리로 중단
    assert a.lap_from == 4
    a._send_lap()                                   # 재개: 4~9번 6개만
    assert len(a.nav.sent) == 6 and a.lap_start == 4
    # 이어 돌던 중 또 중단 — 피드백(2)은 슬라이스 기준이라 오프셋을 더해야 6
    a.nav.feedback = types.SimpleNamespace(current_waypoint=2)
    a.stop_patrol()
    assert a.lap_from == 6
    # 완주하면 다음 바퀴는 처음부터
    a.lap_from = 0
    a._send_lap()
    assert len(a.nav.sent) == 10
    # followWaypoints가 아닌 액션의 피드백엔 current_waypoint가 없다 → 0
    a.nav.feedback = types.SimpleNamespace(distance_remaining=1.0)
    a.stop_patrol()
    assert a.lap_from == 0
    print('robot_agent self-check ok')


if __name__ == '__main__':
    import sys
    _self_check() if '--check' in sys.argv else main()
