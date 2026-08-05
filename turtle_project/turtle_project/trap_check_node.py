"""trap 설치 점검 — sync 이미지로 trap이 제대로 설치됐는지 판정.

판정 기준은 미정(나중). synced/rgb·synced/depth를 구독하고, 결과를
/fleet/event(trap_ok:x:y)로 발행한다. 판정 로직은 TODO(팀원).
"""
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class TrapCheckNode(Node):

    def __init__(self):
        super().__init__('trap_check_node')
        self.event_pub = self.create_publisher(String, '/fleet/event', 10)
        self.rgb_sub = Subscriber(self, CompressedImage, 'synced/rgb',
                                  qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, CompressedImage, 'synced/depth',
                                    qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.cb)
        self.get_logger().info('trap 점검 노드 시작 — synced 구독')

    def cb(self, rgb_msg, depth_msg):
        # TODO(팀원): trap 설치 상태 판정 (기준 미정) → trap_ok event 발행.
        pass


def main():
    rclpy.init()
    node = TrapCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
