"""로봇 1대 제어 — 순찰 주행(Nav2)·dock/undock·배터리 감시·상태보고.

namespace 파라미터로 로봇마다 실행 (robot4/robot6). /fleet/command에서 자기
명령만 필터해 실행하고, /fleet/status로 상태를 보고한다.

PATROL 명령을 받으면 waypoint YAML을 FollowWaypoints로 무한 순찰한다
(patrol_zigzag 로직 통합). dock/undock 실행은 TODO(팀원).
"""
import math

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

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
            'waypoints', '/home/rokey/patrol_waypoints.yaml').value

        self.state = 'IDLE'
        self.battery = 100

        # BasicNavigator는 자체 노드다. 같은 namespace로 만들어 이 로봇의 Nav2에
        # 붙는다. spin은 아래 main에서 이 두 노드를 한 executor에 함께 돌린다.
        self.nav = BasicNavigator(namespace=f'/{self.ns}')
        self.poses = None       # 로드된 순찰 waypoint (첫 PATROL 때 lazy 로드)
        self.patrolling = False  # followWaypoints 진행 중

        self.status_pub = self.create_publisher(String, '/fleet/status', 10)
        self.create_subscription(String, '/fleet/command', self.command_cb, 10)
        self.create_subscription(BatteryState, f'{self.ns}/battery_state',
                                 self.battery_cb, 10)
        self.create_subscription(PoseStamped, f'{self.ns}/target_pose',
                                 self.target_cb, 10)
        self.create_timer(period, self.report)
        # 순찰 진행 감시 — 한 바퀴 끝나면 다음 바퀴 재시작 (무한 순찰).
        self.create_timer(1.0, self.patrol_tick)
        self.get_logger().info(f'{self.ns} agent 시작 — 임계 {self.threshold}%')

    def command_cb(self, msg):
        robot, cmd = fleet_msg.parse_command(msg.data)
        if robot != self.ns:
            return                          # 내 명령 아님
        self.get_logger().info(f'명령 수신: {cmd}')
        self.state = {'UNDOCK': 'PATROLLING', 'DOCK': 'DOCKED',
                      'PATROL': 'PATROLLING', 'TRACK': 'TRACKING',
                      'HERD': 'HERDING', 'STOP': 'IDLE'}.get(cmd, self.state)
        if cmd in ('PATROL', 'UNDOCK'):
            self.start_patrol()
        elif cmd in ('STOP', 'TRACK', 'HERD'):
            # 순찰 중단 — 추적/몰이는 target_pose로 별도 주행(아래 target_cb).
            self.stop_patrol()
        # TODO(팀원): DOCK 시 dock 스테이션 이동 + Dock 액션.

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

    def patrol_tick(self):
        """순찰 중 한 바퀴가 끝났으면 다음 바퀴를 이어 돈다 (무한 순찰)."""
        if not self.patrolling or not self.nav.isTaskComplete():
            return
        result = self.nav.getResult()
        if result == TaskResult.CANCELED:
            self.patrolling = False         # 외부 취소 — 순찰 종료
            return
        # SUCCEEDED든 실패든 계속 순찰 (실패는 다음 바퀴에 재시도).
        self._send_lap()

    def battery_cb(self, msg):
        if math.isnan(msg.percentage):
            return                      # 배터리 값 불명(NaN) — 무시
        self.battery = int(msg.percentage * 100) if msg.percentage <= 1.0 \
            else int(msg.percentage)
        if self.battery < self.threshold and self.state == 'PATROLLING':
            self.state = 'RETURNING'
            self.stop_patrol()              # 순찰 멈추고 복귀
            self.get_logger().info(f'배터리 {self.battery}% < 임계 — 복귀')
            # TODO(팀원): dock 스테이션 이동 + Dock 액션. 완료되면 state=DOCKED.

    def target_cb(self, msg):
        # TODO(팀원): 받은 목표 좌표로 Nav2 navigate_to_pose 전송 (TRACK/HERD 주행).
        self.get_logger().info(
            f'목표 좌표 수신: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})',
            throttle_duration_sec=1.0)

    def report(self):
        self.status_pub.publish(String(data=fleet_msg.status(
            self.ns, self.state, self.battery)))


def main():
    rclpy.init()
    node = RobotAgent()
    # BasicNavigator(node.nav)도 자체 노드라 같은 executor에 넣어 함께 돌린다.
    # 안 그러면 followWaypoints 응답/피드백이 처리되지 않는다.
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


if __name__ == '__main__':
    main()
