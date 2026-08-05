"""opening 감지 시 추가 조치.

순찰 중 YOLO로 opening(침입구 후보)을 보면 Nav2로 앞 approach_dist까지 접근한
뒤, bbox depth 편차(depth_spread)로 진짜 구멍인지 판정해 로그로 낸다.

상태: SEARCHING -> (opening+map좌표) -> APPROACHING -> (Nav2 SUCCEEDED) ->
      VERIFYING -> (판정 로그) -> SEARCHING
"""
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  PointStamped TF 변환 등록
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf2_ros import Buffer, TransformException, TransformListener

from turtle_project.depth_math import (decode_depth, depth_at, depth_spread,
                                       deproject, side_px, to_depth_px)
from turtle_project.nav_controller import (Navigator, approach_point,
                                           make_pose)

RGB_TOPIC = 'oakd/rgb/image_raw/compressed'
DEPTH_TOPIC = 'oakd/stereo/image_raw/compressedDepth'
DEPTH_INFO_TOPIC = 'oakd/stereo/camera_info'
FRAME = 'map'


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.model_path = self.declare_parameter('model_path', '').value
        self.target_class = self.declare_parameter('target_class', 'opening').value
        self.conf = self.declare_parameter('conf', 0.6).value
        self.approach_dist = self.declare_parameter('approach_dist', 0.5).value
        self.depth_gap = self.declare_parameter('depth_gap', 0.05).value
        self.side_margin = self.declare_parameter('side_margin', 0.05).value
        self.verify_timeout = self.declare_parameter('verify_timeout', 30).value

        self.model = self._load_model()
        self.bridge = CvBridge()
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.K = None

        self.state = 'SEARCHING'
        self.target = None          # (x, y) map 좌표 — 접근 목표
        self.verify_count = 0       # VERIFYING 재검출 시도 프레임 수

        self.nav = Navigator(self, on_arrived=self._arrived)
        self.create_subscription(CameraInfo, DEPTH_INFO_TOPIC, self.info_cb, 10)
        self.rgb_sub = Subscriber(self, CompressedImage, RGB_TOPIC,
                                  qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, CompressedImage, DEPTH_TOPIC,
                                    qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.synced_cb)
        self.get_logger().info(
            f"대기 — target='{self.target_class}', 접근 {self.approach_dist}m, "
            f'gap 임계 {self.depth_gap}m')

    def _load_model(self):
        if not self.model_path:
            self.get_logger().warn('model_path 없음 — 탐지 스킵 (경로 주면 동작)')
            return None
        try:
            from ultralytics import YOLO
            # .engine(TensorRT)는 task 메타데이터가 없을 수 있어 명시. .pt에도 무해.
            m = YOLO(self.model_path, task='detect')
            self.get_logger().info(f'모델 로드: {self.model_path}')
            return m
        except Exception as e:
            self.get_logger().warn(f'모델 로드 실패 ({e}) — 탐지 스킵')
            return None

    def info_cb(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def synced_cb(self, rgb_msg, depth_msg):
        if self.model is None or self.K is None or self.state == 'APPROACHING':
            return                  # 모델/보정 없거나 접근 중이면 프레임 무시
        img = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = decode_depth(depth_msg.data, depth_msg.format)
        depth_frame = depth_msg.header.frame_id
        result = self.model(img, conf=self.conf, verbose=False)[0]
        box = self._pick_target(result, img.shape, depth)

        if self.state == 'SEARCHING':
            if box is not None:
                self._start_approach(box, img.shape, depth, depth_frame)
        elif self.state == 'VERIFYING':
            self._verify(box, img.shape, depth)

    def _pick_target(self, result, img_shape, depth):
        """target_class 박스 중 하나 -> box(xyxy) or None.

        여러 개면 화면 중앙에 가까운 것. VERIFYING 재검출도 이걸 쓴다.
        """
        cx = img_shape[1] / 2
        best = None
        best_d = None
        for b in result.boxes:
            if self.model.names[int(b.cls)] != self.target_class:
                continue
            u = float(b.xywh[0][0])
            d = abs(u - cx)
            if best_d is None or d < best_d:
                best_d, best = d, b.xyxy[0].tolist()
        return best

    def _start_approach(self, box, img_shape, depth, depth_frame):
        u = (box[0] + box[2]) / 2
        v = (box[1] + box[3]) / 2
        du, dv = to_depth_px(u, v, img_shape, depth.shape)
        z = depth_at(depth, du, dv)
        if not z:
            self.get_logger().info('opening 봤지만 depth 무효 — 대기')
            return
        pt = PointStamped()
        pt.header.frame_id = depth_frame
        pt.point.x, pt.point.y, pt.point.z = deproject(du, dv, z, self.K)
        try:
            p = self.tf.transform(pt, FRAME).point
        except TransformException as e:
            self.get_logger().warn(f'TF 실패 ({depth_frame}->{FRAME}): {e}',
                                   throttle_duration_sec=5.0)
            return
        robot = self.robot_xy()
        if robot is None:
            return
        rx, ry = robot
        g = approach_point(p.x - rx, p.y - ry, self.approach_dist)
        if g is None:
            self.get_logger().info('이미 접근 거리 안 — 바로 검증')
            self.target = (p.x, p.y)
            self.state = 'VERIFYING'
            self.verify_count = 0
            return
        gx, gy, yaw = rx + g[0], ry + g[1], g[2]
        if self.nav.go(make_pose(FRAME, gx, gy, yaw)):
            self.target = (p.x, p.y)
            self.state = 'APPROACHING'
            self.get_logger().info(
                f'opening ({p.x:.2f}, {p.y:.2f}) 접근 시작')

    def _arrived(self):
        self.get_logger().info('도착 — 검증 시작')
        self.state = 'VERIFYING'
        self.verify_count = 0

    def _verify(self, box, img_shape, depth):
        if box is None:
            self.verify_count += 1
            if self.verify_count >= self.verify_timeout:
                self.get_logger().info('opening 재검출 실패 — 포기, 복귀')
                self._reset()
            return
        du1, dv1 = to_depth_px(box[0], box[1], img_shape, depth.shape)
        du2, dv2 = to_depth_px(box[2], box[3], img_shape, depth.shape)
        # bbox가 구멍에 딱 맞으면 안이 다 '먼' depth라 편차가 안 생긴다.
        # 중심 depth로 side_margin(m)을 픽셀로 바꿔 좌우로 넓혀 옆 벽을 포함한다.
        z = depth_at(depth, (du1 + du2) // 2, (dv1 + dv2) // 2)
        side = side_px(self.side_margin, z, self.K[0, 0])
        gap = depth_spread(depth, du1, dv1, du2, dv2, side=side)
        if gap is None:
            self.verify_count += 1
            if self.verify_count >= self.verify_timeout:
                self.get_logger().info('depth 유효 픽셀 부족 — 포기, 복귀')
                self._reset()
            return
        if gap >= self.depth_gap:
            self.get_logger().info(f'진짜 opening 확인 (gap={gap:.3f}m)')
        else:
            self.get_logger().info(
                f'opening 아님 (gap={gap:.3f}m < {self.depth_gap})')
        self._reset()

    def _reset(self):
        self.state = 'SEARCHING'
        self.target = None
        self.verify_count = 0

    def robot_xy(self):
        try:
            t = self.tf.lookup_transform(FRAME, 'base_link',
                                         rclpy.time.Time()).transform.translation
            return t.x, t.y
        except TransformException as e:
            self.get_logger().warn(f'TF 실패 ({FRAME}->base_link): {e}',
                                   throttle_duration_sec=5.0)
            return None


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
