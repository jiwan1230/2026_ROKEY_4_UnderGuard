"""depth_spread 판정 단독 테스트 — RGB 창에서 박스 클릭, depth로 판정.

RGB(image_raw/compressed)를 OpenCV 창에 띄우고, 좌클릭 4점으로 박스를 그리면
그 영역의 depth 편차(depth_spread)로 구멍인지 로그·화면에 표시한다. 이동 없음.

    좌클릭 4번 -> ROI 박스 지정 (4점을 감싸는 사각형)
    우클릭     -> ROI 초기화 (다시 중앙 기본 ROI)

    ros2 run turtle_project opening_test_node
    ros2 run turtle_project opening_test_node --ros-args -p depth_gap:=0.08
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage

from turtle_project.depth_math import (decode_depth, depth_at, depth_spread,
                                       side_px, to_depth_px)

RGB_TOPIC = '/robot4/oakd/rgb/image_raw/compressed'
DEPTH_TOPIC = '/robot4/oakd/stereo/image_raw/compressedDepth'
DEPTH_INFO_TOPIC = '/robot4/oakd/stereo/camera_info'
WINDOW = 'opening test (rgb)'


class OpeningTestNode(Node):

    def __init__(self):
        super().__init__('opening_test_node')
        self.depth_gap = self.declare_parameter('depth_gap', 0.05).value
        self.side_margin = self.declare_parameter('side_margin', 0.05).value
        self.roi_ratio = self.declare_parameter('roi_ratio', 0.4).value
        self.show = self.declare_parameter('show', True).value
        rgb_topic = self.declare_parameter('rgb_topic', RGB_TOPIC).value
        depth_topic = self.declare_parameter('depth_topic', DEPTH_TOPIC).value
        info_topic = self.declare_parameter('camera_info_topic',
                                            DEPTH_INFO_TOPIC).value

        self.bridge = CvBridge()
        self.fx = None      # stereo camera_info의 fx — side 마진 픽셀 변환용
        self.pts = []       # 좌클릭으로 찍은 점 (RGB 창 좌표, 최대 4개)
        self.roi = None     # (x1, y1, x2, y2) RGB 좌표 — 4점 다 찍히면 설정
        if self.show:
            cv2.namedWindow(WINDOW)
            cv2.setMouseCallback(WINDOW, self.on_mouse)

        # 센서 QoS로 구독 — 카메라가 best-effort로 발행하면 기본(RELIABLE)은
        # fx를 영영 못 받아 옆 벽 확장이 항상 0이 된다.
        self.create_subscription(CameraInfo, info_topic, self.info_cb,
                                 qos_profile_sensor_data)
        # RGB와 depth를 header.stamp로 짝지어 함께 받는다 (같은 순간 프레임).
        self.rgb_sub = Subscriber(self, CompressedImage, rgb_topic,
                                  qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, CompressedImage, depth_topic,
                                    qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.cb)
        self.get_logger().info(
            f'준비 — 좌클릭 4점으로 박스 지정, 우클릭 초기화. '
            f'gap 임계 {self.depth_gap}m')

    def info_cb(self, msg):
        self.fx = msg.k[0]

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.pts) >= 4:
                self.pts = []           # 4점 넘겨 다시 찍으면 새로 시작
            self.pts.append((x, y))
            if len(self.pts) == 4:
                xs = [p[0] for p in self.pts]
                ys = [p[1] for p in self.pts]
                self.roi = (min(xs), min(ys), max(xs), max(ys))
                self.get_logger().info(f'ROI 지정(RGB): {self.roi}')
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.pts = []
            self.roi = None
            self.get_logger().info('ROI 초기화 — 중앙 기본 ROI')

    def cb(self, rgb_msg, depth_msg):
        img = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = decode_depth(depth_msg.data, depth_msg.format)
        h, w = img.shape[:2]
        if self.roi is not None:
            x1, y1, x2, y2 = self.roi
        else:
            rw, rh = int(w * self.roi_ratio / 2), int(h * self.roi_ratio / 2)
            x1, y1, x2, y2 = w // 2 - rw, h // 2 - rh, w // 2 + rw, h // 2 + rh

        # 박스는 RGB 좌표 -> depth 좌표로 변환해서 편차 계산 (해상도 다를 수 있음)
        du1, dv1 = to_depth_px(x1, y1, img.shape, depth.shape)
        du2, dv2 = to_depth_px(x2, y2, img.shape, depth.shape)
        # bbox가 구멍에 딱 맞으면 안이 다 '먼' depth라 편차가 안 생긴다. 박스 중심
        # depth로 side_margin(m)을 픽셀로 바꿔 좌우로 넓혀 옆 벽을 포함한다.
        z = depth_at(depth, (du1 + du2) // 2, (dv1 + dv2) // 2)
        side = side_px(self.side_margin, z, self.fx)
        gap = depth_spread(depth, du1, dv1, du2, dv2, side=side)

        if gap is None:
            label = 'depth 유효 픽셀 부족'
        elif gap >= self.depth_gap:
            label = f'OPENING (gap={gap:.3f}m)'
        else:
            label = f'flat (gap={gap:.3f}m)'
        self.get_logger().info(label, throttle_duration_sec=1.0)

        if self.show:
            box = (0, 255, 0) if self.roi is not None else (255, 255, 255)
            cv2.rectangle(img, (x1, y1), (x2, y2), box, 2)
            # 실제 측정 영역(좌우 side 확장)을 RGB 좌표로 되돌려 점선처럼 표시
            sx = int(side * img.shape[1] / depth.shape[1])
            cv2.rectangle(img, (x1 - sx, y1), (x2 + sx, y2), (0, 140, 255), 1)
            for p in self.pts:
                cv2.circle(img, p, 4, (0, 255, 255), -1)
            cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 255), 2)
            cv2.imshow(WINDOW, img)
            cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = OpeningTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
