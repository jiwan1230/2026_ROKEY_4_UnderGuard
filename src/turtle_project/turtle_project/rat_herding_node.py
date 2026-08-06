"""쥐몰이 — 로봇B 목표좌표를 몰이 알고리즘으로 발행.

/fleet/event의 rat 위치를 받아 로봇B가 갈 goal을 계속 갱신한다. 알고리즘은
팀원이 만든다. 뼈대는 구독→발행 배관만.
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from turtle_project import fleet_msg


class RatHerdingNode(Node):

    def __init__(self):
        super().__init__('rat_herding_node')
        self.robot_b = self.declare_parameter('robot_b', 'robot6').value
        self.pose_pub = self.create_publisher(
            PoseStamped, f'{self.robot_b}/target_pose', 10)
        self.create_subscription(String, '/fleet/event', self.event_cb, 10)
        self.get_logger().info(f'쥐몰이 노드 시작 — {self.robot_b} goal 발행')

    def event_cb(self, msg):
        name, x, y = fleet_msg.parse_event(msg.data)
        if name != 'rat_detected':
            return
        # TODO(팀원): 몰이 알고리즘 — 쥐 위치(x,y)로 로봇B 유도 goal 계산.
        self.get_logger().info(
            f'쥐 {x:.2f},{y:.2f} — TODO: 몰이 goal 계산·발행',
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
