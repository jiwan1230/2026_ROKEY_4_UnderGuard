"""카메라 이미지 파이프 — rgb+depth를 stamp로 sync해서 재발행만 한다.

detector_node / trap_check_node가 이 정렬된 토픽을 구독한다. 원본을 각자
구독해 따로 sync하면 노드마다 같은 짝맞추기를 반복하므로, 여기서 한 번만 한다.
namespace 파라미터로 로봇마다 실행.
"""
import rclpy
from message_filters import (ApproximateTimeSynchronizer, Subscriber)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage

RGB_IN = 'oakd/rgb/image_raw/compressed'
DEPTH_IN = 'oakd/stereo/image_raw/compressedDepth'
INFO_IN = 'oakd/stereo/camera_info'


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        # 원본은 절대 토픽(네임스페이스는 launch/remap에서). 재발행은 상대 토픽.
        self.rgb_pub = self.create_publisher(CompressedImage, 'synced/rgb', 10)
        self.depth_pub = self.create_publisher(CompressedImage, 'synced/depth', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'synced/camera_info', 10)

        self.rgb_sub = Subscriber(self, CompressedImage, RGB_IN,
                                  qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, CompressedImage, DEPTH_IN,
                                    qos_profile=qos_profile_sensor_data)
        self.info_sub = Subscriber(self, CameraInfo, INFO_IN,
                                   qos_profile=qos_profile_sensor_data)
        # camera_info도 함께 sync — detector가 K를 같은 프레임으로 받게.
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub],
            queue_size=10, slop=0.1)
        self.sync.registerCallback(self.cb)
        self.get_logger().info('sync 파이프 시작 — synced/rgb, synced/depth 재발행')

    def cb(self, rgb, depth, info):
        # 세 메시지가 같은 stamp로 짝맞은 것만 그대로 재발행 (변환 없음).
        self.rgb_pub.publish(rgb)
        self.depth_pub.publish(depth)
        self.info_pub.publish(info)


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
