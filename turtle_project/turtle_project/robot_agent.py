"""로봇 1대 제어 — 주행(Nav2)·dock/undock·배터리 감시·상태보고.

namespace 파라미터로 로봇마다 실행 (robot4/robot6). /fleet/command에서 자기
명령만 필터해 실행하고, /fleet/status로 상태를 보고한다.

배터리 임계 감지만 실제 동작, 주행·도킹 실행은 TODO(팀원).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from turtle_project import fleet_msg


class RobotAgent(Node):

    def __init__(self):
        super().__init__('robot_agent')
        self.ns = self.declare_parameter('namespace', 'robot4').value
        self.threshold = self.declare_parameter('battery_threshold', 20).value
        period = self.declare_parameter('battery_check_period', 10.0).value

        self.state = 'IDLE'
        self.battery = 100

        self.status_pub = self.create_publisher(String, '/fleet/status', 10)
        self.create_subscription(String, '/fleet/command', self.command_cb, 10)
        self.create_subscription(BatteryState, f'{self.ns}/battery_state',
                                 self.battery_cb, 10)
        self.create_timer(period, self.report)
        self.get_logger().info(f'{self.ns} agent 시작 — 임계 {self.threshold}%')

    def command_cb(self, msg):
        robot, cmd = fleet_msg.parse_command(msg.data)
        if robot != self.ns:
            return                          # 내 명령 아님
        self.get_logger().info(f'명령 수신: {cmd}')
        # TODO(팀원): UNDOCK/DOCK/PATROL/TRACK/HERD 실제 실행 (액션/Nav2).
        # 뼈대는 명령을 상태로만 반영.
        self.state = {'UNDOCK': 'PATROLLING', 'DOCK': 'DOCKED',
                      'PATROL': 'PATROLLING', 'TRACK': 'TRACKING',
                      'HERD': 'HERDING', 'STOP': 'IDLE'}.get(cmd, self.state)

    def battery_cb(self, msg):
        self.battery = int(msg.percentage * 100) if msg.percentage <= 1.0 \
            else int(msg.percentage)
        if self.battery < self.threshold and self.state == 'PATROLLING':
            self.state = 'RETURNING'
            self.get_logger().info(f'배터리 {self.battery}% < 임계 — 복귀')
            # TODO(팀원): dock 스테이션 이동 + Dock 액션. 완료되면 state=DOCKED.

    def report(self):
        self.status_pub.publish(String(data=fleet_msg.status(
            self.ns, self.state, self.battery)))


def main():
    rclpy.init()
    node = RobotAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
