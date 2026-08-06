"""로봇 카메라 감지 — YOLO로 opening·rat 감지.

opening: DB에 좌표 조회 → 없으면 접근·depth_spread 검증(기존 camera_node 로직
이동) → 진짜 구멍이면 trap 설치단계(로그)+DB기록. 있으면 trap 점검으로.
rat: target_pose로 추적 goal + /fleet/event 발행.

synced/rgb·synced/depth(camera_node 발행)를 구독한다. namespace로 실행.
opening 검증만 실제 동작, rat·DB 분기는 TODO(팀원).
"""
import math

import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  PointStamped TF 변환 등록
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from turtle_interfaces.srv import QueryHole
from turtle_project import fleet_msg
from turtle_project.depth_math import (decode_depth, depth_at, depth_spread,
                                       deproject, side_px, to_depth_px)
from turtle_project.nav_controller import approach_point, make_pose

FRAME = 'map'


class DetectorNode(Node):

    def __init__(self):
        super().__init__('detector_node')
        self.model_path = self.declare_parameter('model_path', '').value
        self.conf = self.declare_parameter('conf', 0.6).value
        self.approach_dist = self.declare_parameter('approach_dist', 0.5).value
        self.arrive_tol = self.declare_parameter('arrive_tol', 0.3).value
        self.depth_gap = self.declare_parameter('depth_gap', 0.05).value
        self.side_margin = self.declare_parameter('side_margin', 0.05).value
        self.verify_timeout = self.declare_parameter('verify_timeout', 30).value

        self.model = self._load_model()
        self.bridge = CvBridge()
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.K = None

        self.state = 'SEARCHING'
        self.target = None      # 검증 대상 opening의 map 좌표
        self.goal = None        # 발행한 접근 goal (TF 도착 판정용)
        self.verify_count = 0

        self.event_pub = self.create_publisher(String, '/fleet/event', 10)
        # detector는 goal을 직접 안 쏜다. 접근·추적 목표는 target_pose로 발행만
        # 하고, 실제 Nav2 주행은 robot_agent가 한다 (로봇당 Nav2 주인 1개).
        self.pose_pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.db = self.create_client(QueryHole, '/db/query_hole')

        self.rgb_sub = Subscriber(self, CompressedImage, 'synced/rgb',
                                  qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, CompressedImage, 'synced/depth',
                                    qos_profile=qos_profile_sensor_data)
        self.info_sub = Subscriber(self, CameraInfo, 'synced/camera_info',
                                   qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub],
            queue_size=10, slop=0.1)
        self.sync.registerCallback(self.synced_cb)
        self.get_logger().info(
            f'감지 시작 — gap 임계 {self.depth_gap}m, 접근 {self.approach_dist}m')

    def _load_model(self):
        if not self.model_path:
            self.get_logger().warn('model_path 없음 — 탐지 스킵 (경로 주면 동작)')
            return None
        try:
            from ultralytics import YOLO
            m = YOLO(self.model_path, task='detect')
            self.get_logger().info(f'모델 로드: {self.model_path}')
            return m
        except Exception as e:
            self.get_logger().warn(f'모델 로드 실패 ({e}) — 탐지 스킵')
            return None

    def synced_cb(self, rgb_msg, depth_msg, info_msg):
        if self.model is None:
            return
        if self.state == 'APPROACHING':     # 주행은 robot_agent — 도착만 감시
            self._check_arrival()
            return
        self.K = np.array(info_msg.k).reshape(3, 3)
        img = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = decode_depth(depth_msg.data, depth_msg.format)
        depth_frame = depth_msg.header.frame_id
        result = self.model(img, conf=self.conf, verbose=False)[0]

        # TODO(팀원): rat 감지 — 아래 opening과 같은 방식으로 rat 클래스 박스를
        # 잡아 map 좌표 계산 후 self._emit_rat(x, y) + target_pose 추적 goal.
        self._detect_rat(result, img.shape, depth, depth_frame)

        box = self._pick(result, 'opening', img.shape)
        if self.state == 'SEARCHING':
            if box is not None:
                self._on_opening(box, img.shape, depth, depth_frame)
        elif self.state == 'VERIFYING':
            self._verify(box, img.shape, depth)

    def _detect_rat(self, result, img_shape, depth, depth_frame):
        """TODO(팀원): rat 박스 → map좌표 → event+target_pose. 뼈대는 감지만 로그."""
        box = self._pick(result, 'rat', img_shape)
        if box is not None:
            self.get_logger().info('rat 감지 — TODO: 추적 goal/event 발행',
                                   throttle_duration_sec=1.0)

    def _pick(self, result, cls_name, img_shape):
        """cls_name 박스 중 화면 중앙에 가장 가까운 것 -> xyxy or None."""
        if self.model is None:
            return None
        cx = img_shape[1] / 2
        best, best_d = None, None
        for b in result.boxes:
            if self.model.names[int(b.cls)] != cls_name:
                continue
            u = float(b.xywh[0][0])
            d = abs(u - cx)
            if best_d is None or d < best_d:
                best_d, best = d, b.xyxy[0].tolist()
        return best

    def _on_opening(self, box, img_shape, depth, depth_frame):
        """opening 감지 → map좌표 → DB조회 분기. 좌표계산은 기존 로직."""
        xy = self._box_to_map(box, img_shape, depth, depth_frame)
        if xy is None:
            return
        # TODO(팀원): DB 조회. 뼈대는 조회 요청만 보내고 바로 접근 검증으로 간다.
        # if exists: trap 점검 단계로. else: 아래 접근·검증.
        self._request_db(*xy)
        self._start_approach(xy, depth_frame)

    def _request_db(self, x, y):
        """/db/query_hole 비동기 호출. 뼈대는 결과 로그만 (분기는 TODO)."""
        if not self.db.service_is_ready():
            self.get_logger().warn('db 서비스 아직 없음', throttle_duration_sec=5.0)
            return
        req = QueryHole.Request()
        req.x, req.y = float(x), float(y)
        self.db.call_async(req).add_done_callback(
            lambda f: self.get_logger().info(
                f'DB 응답: exists={f.result().exists} '
                f'trap={f.result().trap_installed}'))

    def _box_to_map(self, box, img_shape, depth, depth_frame):
        """bbox 중심 → depth → deproject → TF map좌표 (x, y). 무효면 None."""
        u, v = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
        du, dv = to_depth_px(u, v, img_shape, depth.shape)
        z = depth_at(depth, du, dv)
        if not z:
            return None
        pt = PointStamped()
        pt.header.frame_id = depth_frame
        pt.point.x, pt.point.y, pt.point.z = deproject(du, dv, z, self.K)
        try:
            p = self.tf.transform(pt, FRAME).point
        except TransformException as e:
            self.get_logger().warn(f'TF 실패: {e}', throttle_duration_sec=5.0)
            return None
        return p.x, p.y

    def _start_approach(self, xy, depth_frame):
        robot = self.robot_xy()
        if robot is None:
            return
        rx, ry = robot
        g = approach_point(xy[0] - rx, xy[1] - ry, self.approach_dist)
        if g is None:                       # 이미 approach_dist 안 — 바로 검증
            self.target = xy
            self.state = 'VERIFYING'
            self.verify_count = 0
            return
        # 접근점을 map 절대좌표로 되돌려 target_pose로 발행 (robot_agent가 주행).
        gx, gy, yaw = rx + g[0], ry + g[1], g[2]
        self.pose_pub.publish(make_pose(FRAME, gx, gy, yaw))
        self.target = xy
        self.goal = (gx, gy)
        self.state = 'APPROACHING'
        self.get_logger().info(f'opening ({xy[0]:.2f}, {xy[1]:.2f}) 접근 goal 발행')

    def _check_arrival(self):
        """APPROACHING 중 로봇이 goal 근처(arrive_tol)에 오면 검증으로 전환.

        Nav2 완료 콜백 대신 TF 거리로 판정 (detector는 주행을 안 하므로 도착
        신호가 없다). robot_agent가 실제로 그 goal로 몰고 있다고 전제한다.
        """
        robot = self.robot_xy()
        if robot is None or self.goal is None:
            return
        d = math.hypot(self.goal[0] - robot[0], self.goal[1] - robot[1])
        if d <= self.arrive_tol:
            self.get_logger().info('도착 — 검증 시작')
            self.state = 'VERIFYING'
            self.verify_count = 0

    def _verify(self, box, img_shape, depth):
        if box is None:
            self._verify_miss('opening 재검출 실패')
            return
        du1, dv1 = to_depth_px(box[0], box[1], img_shape, depth.shape)
        du2, dv2 = to_depth_px(box[2], box[3], img_shape, depth.shape)
        z = depth_at(depth, (du1 + du2) // 2, (dv1 + dv2) // 2)
        side = side_px(self.side_margin, z, self.K[0, 0])
        gap = depth_spread(depth, du1, dv1, du2, dv2, side=side)
        if gap is None:
            self._verify_miss('depth 유효 픽셀 부족')
            return
        if gap >= self.depth_gap:
            self.get_logger().info(f'진짜 opening 확인 (gap={gap:.3f}m)')
            if self.target:
                self.event_pub.publish(String(data=fleet_msg.event(
                    'opening_confirmed', *self.target)))
            # TODO(팀원): trap 설치 단계 + DB 기록
        else:
            self.get_logger().info(
                f'opening 아님 (gap={gap:.3f}m < {self.depth_gap})')
        self._reset()

    def _verify_miss(self, why):
        self.verify_count += 1
        if self.verify_count >= self.verify_timeout:
            self.get_logger().info(f'{why} — 포기, 복귀')
            self._reset()

    def _reset(self):
        self.state = 'SEARCHING'
        self.target = None
        self.goal = None
        self.verify_count = 0

    def robot_xy(self):
        try:
            t = self.tf.lookup_transform(FRAME, 'base_link',
                                         rclpy.time.Time()).transform.translation
            return t.x, t.y
        except TransformException as e:
            self.get_logger().warn(f'TF 실패: {e}', throttle_duration_sec=5.0)
            return None


def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
