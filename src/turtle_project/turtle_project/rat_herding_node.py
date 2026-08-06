"""쥐몰이 — 로봇B 목표좌표를 몰이 알고리즘으로 발행.

/fleet/event의 rat 위치를 받아 로봇B가 갈 goal을 계속 갱신한다. 알고리즘은
팀원이 만든다. 뼈대는 구독→발행 배관만.

로봇B(몰이 담당)는 고정이 아니라 central이 그때그때 배정하므로, /fleet/command
에서 HERD 명령을 받은 로봇을 B로 인식해 그 로봇의 target_pose로 goal을 쏜다.
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from turtle_project import fleet_msg


class RatHerdingNode(Node):

    def __init__(self):
        super().__init__('rat_herding_node')
        self.robot_b = None     # HERD 명령 받은 로봇 (command 구독으로 동적 결정)
        self.pose_pub = None    # robot_b 정해지면 그 로봇의 target_pose로 생성
        self.create_subscription(String, '/fleet/command', self.command_cb, 10)
        self.create_subscription(String, '/fleet/event', self.event_cb, 10)
        self.get_logger().info('쥐몰이 노드 시작 — HERD 대상 대기')

    def command_cb(self, msg):
        """HERD 명령 대상 = 로봇B. central이 배정한 B로 발행 토픽을 맞춘다."""
        robot, cmd = fleet_msg.parse_command(msg.data)
        if cmd == 'HERD' and robot != self.robot_b:
            self.robot_b = robot
            self.pose_pub = self.create_publisher(
                PoseStamped, f'{robot}/target_pose', 10)
            self.get_logger().info(f'robot B = {robot} — 몰이 goal 발행 대상')

    def event_cb(self, msg):
        name, x, y = fleet_msg.parse_event(msg.data)
        if name != 'rat_detected' or self.pose_pub is None:
            return              # B 미배정 시 발행 대상 없음 (HERD 전이거나 순찰중)
        # TODO(팀원): 몰이 알고리즘 — 쥐 위치(x,y)로 로봇B 유도 goal 계산 후
        # self.pose_pub.publish(...)로 self.robot_b에게 발행.
        self.get_logger().info(
            f'쥐 {x:.2f},{y:.2f} — TODO: 몰이 goal 계산·발행 (B={self.robot_b})',
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = RatHerdingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
