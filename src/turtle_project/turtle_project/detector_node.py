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
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from turtle_interfaces.msg import TrapJob
from turtle_interfaces.srv import QueryHole
from turtle_project import fleet_msg
from turtle_project.depth_math import (decode_depth, depth_at, depth_spread,
                                       deproject, side_px, to_depth_px)
from turtle_project.nav_controller import approach_point, make_pose

FRAME = 'map'

# 상태별 wall timeout(초) — 어떤 상태에서도 영구 대기하지 않도록 한다.
# VERIFYING/INSPECTING은 프레임 카운트 timeout이 따로 있지만, 카메라가 죽어
# 프레임 자체가 끊기면 그것도 안 돌므로 wall 백스톱을 함께 둔다.
DEADLINES = {'APPROACHING': 60.0, 'QUERYING': 5.0, 'VERIFYING': 30.0,
             'INSPECTING': 30.0, 'AWAIT_TRAP': 20.0}


def watchdog_action(state, elapsed):
    """상태·경과시간 -> timeout 처리 ('verify'|'finish'|None). 순수함수.

    QUERYING timeout은 db가 응답 없는 것 — 처음 본 셈 치고 검증으로 진행.
    나머지는 opening 작업을 포기하고 순찰 재개(finish).
    """
    limit = DEADLINES.get(state)
    if limit is None or elapsed <= limit:
        return None
    return 'verify' if state == 'QUERYING' else 'finish'


class RatTracker:
    """쥐 추적 판정 — 좌표 스트림으로 포획/놓침을 가린다. ROS 무관(시각 주입).

    포획: 쥐가 capture_radius 안에서 capture_secs 이상 머무름 (안 움직임).
    놓침: lost_secs 이상 쥐가 안 보임 (감지 끊김).
    좌표 추정값은 떨리므로 '정확히 같음'이 아니라 반경으로 판정한다.
    """

    def __init__(self, capture_radius=0.3, capture_secs=10.0, lost_secs=5.0):
        self.capture_radius = capture_radius
        self.capture_secs = capture_secs
        self.lost_secs = lost_secs
        self.anchor = None      # 머무는 중심 (x, y)
        self.anchor_time = None  # anchor에 자리잡은 시각
        self.last_seen = None   # 마지막 감지 시각
        self.started = None     # 추적 시작 시각 (관측 0회여도 lost 판정 기준)

    def start(self, now):
        """TRACK 진입 — 시작 시각 기록. 이후 쥐를 한 번도 못 봐도
        lost_secs 뒤에는 is_lost가 True가 된다 (영구 고착 방지)."""
        self.reset()
        self.started = now

    def seen(self, x, y, now):
        """쥐 감지 갱신. 반경 밖으로 움직이면 anchor 재설정. 포획이면 True."""
        if self.anchor is None or \
                math.hypot(x - self.anchor[0], y - self.anchor[1]) > self.capture_radius:
            self.anchor = (x, y)        # 새로 자리잡음 (또는 반경 벗어나 이동)
            self.anchor_time = now
        self.last_seen = now
        return now - self.anchor_time >= self.capture_secs

    def is_lost(self, now):
        """마지막 감지(없으면 추적 시작) 후 lost_secs 넘게 안 보였으면 True."""
        ref = self.last_seen if self.last_seen is not None else self.started
        return ref is not None and now - ref >= self.lost_secs

    def reset(self):
        self.anchor = self.anchor_time = self.last_seen = self.started = None


class DetectorNode(Node):

    def __init__(self):
        super().__init__('detector_node')
        self.model_path = self.declare_parameter(
            'model_path',
            '/home/rokey/turtlebot4_ws/src/turtle_project/resource/best.pt').value
        self.conf = self.declare_parameter('conf', 0.6).value
        self.approach_dist = self.declare_parameter('approach_dist', 0.5).value
        self.arrive_tol = self.declare_parameter('arrive_tol', 0.3).value
        self.depth_gap = self.declare_parameter('depth_gap', 0.05).value
        self.side_margin = self.declare_parameter('side_margin', 0.05).value
        self.verify_timeout = self.declare_parameter('verify_timeout', 30).value
        cap_r = self.declare_parameter('capture_radius', 0.3).value
        cap_s = self.declare_parameter('capture_secs', 10.0).value
        lost_s = self.declare_parameter('lost_secs', 5.0).value
        self.reinstall_max = self.declare_parameter('reinstall_max', 3).value
        # 추적 goal 재발행 주기(초) — 매 프레임 쏘면 Nav2 goal이 폭주한다.
        self.rat_goal_period = self.declare_parameter('rat_goal_period', 1.0).value

        # fleet 명령의 대상 필터용 자기 ID — __ns:=/robot4 로 실행하면 'robot4'.
        # 네임스페이스 없이 띄우면 빈 문자열이라 모든 명령을 무시하게 된다
        # (시작 로그로 확인 가능).
        self.robot_id = self.get_namespace().rstrip('/').split('/')[-1]

        self.model = self._load_model()
        self.bridge = CvBridge()
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.K = None

        self.state = 'SEARCHING'
        self.target = None      # 검증 대상 opening의 map 좌표
        self.goal = None        # 발행한 접근 goal (TF 도착 판정용)
        self.verify_count = 0
        self.hole = None        # 점검 대상 구멍의 map 좌표 (INSPECTING/재설치용)
        self.reinstall_count = 0  # trap_bad/미검출로 인한 재설치 횟수
        # 쥐 추적 판정기 (포획/놓침). tracking True일 때만 굴린다.
        self.rat = RatTracker(cap_r, cap_s, lost_s)
        self.tracking = False
        self._last_rat_goal = 0.0   # 마지막 추적 goal 발행 시각 (rat_goal_period 조절용)

        self.event_pub = self.create_publisher(String, '/fleet/event', 10)
        # detector는 goal을 직접 안 쏜다. 접근·추적 목표는 target_pose로 발행만
        # 하고, 실제 Nav2 주행은 robot_agent가 한다 (로봇당 Nav2 주인 1개).
        self.pose_pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        # opening 처리 동안 순찰을 멈춰둔다 (명시 신호). robot_agent가 구독.
        self.hold_pub = self.create_publisher(Bool, 'patrol_hold', 10)
        # trap 설치/점검 지시 — trap_check가 받아 주행/판정만 한다.
        self.job_pub = self.create_publisher(TrapJob, 'trap_job', 10)
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
        # 쥐 추적은 central의 TRACK 명령으로 켠다 (내 로봇이 A일 때만).
        self.create_subscription(String, '/fleet/command', self.command_cb, 10)
        # trap_check가 발행하는 설치완료/점검결과를 받아 상태를 전이한다.
        self.create_subscription(String, '/fleet/event', self.trap_event_cb, 10)
        # 놓침은 감지가 끊긴 걸로 판단 — 프레임 콜백이 안 오므로 타이머로 감시.
        self.create_timer(1.0, self.lost_tick)
        # opening 상태기계 wall timeout + hold heartbeat (영구 대기 방지).
        self._wd_state = 'SEARCHING'
        self._wd_since = 0.0
        self.create_timer(1.0, self.watchdog_tick)
        self.get_logger().info(
            f'감지 시작 [{self.robot_id or "네임스페이스 없음!"}] — '
            f'gap 임계 {self.depth_gap}m, 접근 {self.approach_dist}m')

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
        if self.state in ('QUERYING', 'AWAIT_TRAP'):
            return                          # db 응답/trap_check 응답 대기 — 지각 불필요
        self.K = np.array(info_msg.k).reshape(3, 3)
        img = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = decode_depth(depth_msg.data, depth_msg.format)
        depth_frame = depth_msg.header.frame_id
        result = self.model(img, conf=self.conf, verbose=False)[0]

        # rat 감지 → 추적 goal/포획 판정 (SEARCHING·TRACK 무관하게 매 프레임 확인).
        self._detect_rat(result, img.shape, depth, depth_frame)

        if self.state == 'SEARCHING':
            box, conf = self._pick(result, 'opening', img.shape)
            if box is not None:
                self._on_opening(box, conf, img.shape, depth, depth_frame)
        elif self.state == 'VERIFYING':
            box, _ = self._pick(result, 'opening', img.shape)
            self._verify(box, img.shape, depth)
        elif self.state == 'INSPECTING':
            self._inspect(result, img.shape, depth, depth_frame)

    def _detect_rat(self, result, img_shape, depth, depth_frame):
        """쥐 감지 → map좌표 → RatTracker 판정. 포획이면 rat_captured 발행.

        TRACK 모드(self.tracking)일 때만 포획/놓침을 판정한다. 순찰 중 우연히
        쥐가 보이면 rat_detected만 쏴 central이 쥐대응을 시작하게 한다.
        """
        box, conf = self._pick(result, 'rat', img_shape)
        if box is None:
            return
        xy = self._box_to_map(box, img_shape, depth, depth_frame)
        if xy is None:
            return
        self.get_logger().info(
            f'rat 감지 bbox=({box[0]:.0f},{box[1]:.0f},{box[2]:.0f},{box[3]:.0f}) '
            f'conf={conf:.2f} -> map=({xy[0]:.2f}, {xy[1]:.2f})',
            throttle_duration_sec=1.0)
        if not self.tracking:
            # 아직 쥐대응 전 — 최초 감지만 알림 (central이 역할배정 시작).
            self.event_pub.publish(String(data=fleet_msg.event(
                'rat_detected', *xy)))
            return
        # 추적 중 — 쥐 위치로 추적 goal 발행 + 포획 판정. robot_agent가 target_pose를
        # 받아 실제로 몬다. 매 프레임 쏘면 goal 폭주라 rat_goal_period마다 한 번만.
        now = self._now()
        if now - self._last_rat_goal >= self.rat_goal_period:
            self.pose_pub.publish(make_pose(FRAME, xy[0], xy[1], 0.0))
            self._last_rat_goal = now
            self.get_logger().info(
                f'쥐 ({xy[0]:.2f}, {xy[1]:.2f}) 추적 goal 발행',
                throttle_duration_sec=1.0)
        if self.rat.seen(xy[0], xy[1], now):
            self.get_logger().info(f'쥐 포획 판정 ({xy[0]:.2f}, {xy[1]:.2f})')
            self.event_pub.publish(String(data=fleet_msg.event(
                'rat_captured', *xy)))
            self._stop_tracking()

    def _pick(self, result, cls_name, img_shape):
        """cls_name 박스 중 화면 중앙에 가장 가까운 것 -> (xyxy, conf) or (None, None)."""
        if self.model is None:
            return None, None
        cx = img_shape[1] / 2
        best, best_conf, best_d = None, None, None
        for b in result.boxes:
            if self.model.names[int(b.cls)] != cls_name:
                continue
            u = float(b.xywh[0][0])
            d = abs(u - cx)
            if best_d is None or d < best_d:
                best_d, best, best_conf = d, b.xyxy[0].tolist(), float(b.conf)
        return best, best_conf

    def _on_opening(self, box, conf, img_shape, depth, depth_frame):
        """opening 감지 → 접근 goal 계산 성공 후에만 순찰 정지(hold).

        hold를 먼저 걸면 TF 실패 시 순찰만 멈추고 접근은 안 하는 채로 남는다
        — hold는 _start_approach 안에서 goal이 확정된 뒤 건다.
        """
        xy = self._box_to_map(box, img_shape, depth, depth_frame)
        if xy is None:
            return
        self.get_logger().info(
            f'opening 감지 bbox=({box[0]:.0f},{box[1]:.0f},{box[2]:.0f},{box[3]:.0f}) '
            f'conf={conf:.2f} -> map=({xy[0]:.2f}, {xy[1]:.2f})')
        self._start_approach(xy, depth_frame)

    def _request_db(self, x, y):
        """/db/query_hole 비동기 호출 → 응답으로 신구 분기 (QUERYING 진입)."""
        if not self.db.service_is_ready():
            self.get_logger().warn('db 서비스 아직 없음 — 검증으로 진행',
                                   throttle_duration_sec=5.0)
            self._enter_verify()            # db 없으면 처음 본 셈 치고 검증
            return
        self.state = 'QUERYING'
        req = QueryHole.Request()
        req.x, req.y = float(x), float(y)
        self.db.call_async(req).add_done_callback(self._db_done)

    def _db_done(self, fut):
        """db 응답 처리 — 처음이면 검증, 기존이면 점검(저장좌표 보관)."""
        resp = fut.result()
        if resp.exists:
            self.hole = (resp.hole_x, resp.hole_y)   # 15cm 비교 기준
            self.state = 'INSPECTING'
            self.get_logger().info('기존 구멍 — trap 점검 시작')
        else:
            self._enter_verify()

    def _enter_verify(self):
        self.state = 'VERIFYING'
        self.verify_count = 0

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
            return          # TF 실패 — hold 안 걸고 SEARCHING 유지 (다음 프레임 재시도)
        rx, ry = robot
        g = approach_point(xy[0] - rx, xy[1] - ry, self.approach_dist)
        if g is None:                       # 이미 approach_dist 안 — 바로 검증
            self._hold_patrol(True)         # goal 확정 — 이제 순찰 멈춤
            self.target = xy
            self.state = 'VERIFYING'
            self.verify_count = 0
            return
        # 접근점을 map 절대좌표로 되돌려 target_pose로 발행 (robot_agent가 주행).
        gx, gy, yaw = rx + g[0], ry + g[1], g[2]
        self._hold_patrol(True)             # goal 확정 — 이제 순찰 멈춤
        self.pose_pub.publish(make_pose(FRAME, gx, gy, yaw))
        self.target = xy
        self.goal = (gx, gy)
        self.state = 'APPROACHING'
        self.get_logger().info(f'opening ({xy[0]:.2f}, {xy[1]:.2f}) 접근 goal 발행')

    def _check_arrival(self):
        """APPROACHING 중 로봇이 goal 근처(arrive_tol)에 오면 db 조회로 전환.

        Nav2 완료 콜백 대신 TF 거리로 판정 (detector는 주행을 안 하므로 도착
        신호가 없다). robot_agent가 실제로 그 goal로 몰고 있다고 전제한다.
        도착 후 감지 좌표(self.target)로 db에 신구를 물어 분기한다.
        """
        robot = self.robot_xy()
        if robot is None or self.goal is None:
            return
        d = math.hypot(self.goal[0] - robot[0], self.goal[1] - robot[1])
        if d <= self.arrive_tol:
            self.get_logger().info('도착 — db 조회')
            self._request_db(*self.target)

    def command_cb(self, msg):
        """내 로봇이 A(TRACK)로 배정되면 추적 판정 시작, 끝나면 끈다."""
        robot, cmd = fleet_msg.parse_command(msg.data)
        if robot != self.robot_id:
            return                  # 내 로봇 명령 아님 (robot6용 TRACK 등)
        if cmd == 'TRACK' and not self.tracking:
            self.tracking = True
            self.rat.start(self._now())  # 관측 0회여도 lost_secs 뒤 놓침 판정
            self.get_logger().info('TRACK 시작 — 쥐 포획/놓침 판정 on')
        elif cmd in ('PATROL', 'STOP', 'DOCK') and self.tracking:
            self._stop_tracking()   # 쥐대응 종료 명령 — 추적 판정 off

    def lost_tick(self):
        """추적 중 lost_secs 넘게 쥐가 안 보이면 rat_lost 발행 (감지 끊김)."""
        if not self.tracking or not self.rat.is_lost(self._now()):
            return
        self.get_logger().info('쥐 놓침 판정 — rat_lost 발행')
        # 놓친 위치는 마지막 anchor (없으면 0,0 — central은 이름만 보고 판단).
        x, y = self.rat.anchor or (0.0, 0.0)
        self.event_pub.publish(String(data=fleet_msg.event('rat_lost', x, y)))
        self._stop_tracking()

    def _stop_tracking(self):
        self.tracking = False
        self.rat.reset()

    def watchdog_tick(self):
        """opening 상태기계 wall timeout + hold heartbeat (1초 주기).

        어떤 비-SEARCHING 상태도 DEADLINES를 넘기면 빠져나온다 — Nav2 실패,
        db 무응답, trap_check 미실행 등으로 영구 대기하지 않는다. 처리 중엔
        hold=True를 재발행(heartbeat)해서, detector가 죽으면 robot_agent가
        lease(hold_lease_sec) 만료로 자동 순찰 재개하게 한다.
        """
        now = self._now()
        if self.state != self._wd_state:    # 상태 전이 감지 — 시계 리셋
            self._wd_state, self._wd_since = self.state, now
        if self.state == 'SEARCHING':
            return
        self._hold_patrol(True)             # heartbeat — agent의 lease 갱신
        act = watchdog_action(self.state, now - self._wd_since)
        if act == 'verify':
            self.get_logger().warn('db 응답 timeout — 처음 본 셈 치고 검증')
            self._enter_verify()
        elif act == 'finish':
            self.get_logger().warn(f'{self.state} timeout — opening 포기, 순찰 재개')
            self._finish_opening()

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

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
            self.get_logger().info(f'진짜 opening 확인 (gap={gap:.3f}m) — 설치 지시')
            # db 저장(opening_confirmed) 후 최초 설치. 이후 재점검/재설치는
            # trap_check 이벤트로 이어진다. 최초 설치는 재설치 카운트에 안 넣음.
            self.event_pub.publish(String(data=fleet_msg.event(
                'opening_confirmed', *self.target)))
            self.hole = self.target
            self.reinstall_count = 0
            self._send_install()
        else:
            self.get_logger().info(
                f'opening 아님 (gap={gap:.3f}m < {self.depth_gap}) — 순찰 재개')
            self._finish_opening()

    def _verify_miss(self, why):
        self.verify_count += 1
        if self.verify_count >= self.verify_timeout:
            self.get_logger().info(f'{why} — 포기, 복귀')
            self._finish_opening()

    def _inspect(self, result, img_shape, depth, depth_frame):
        """구멍 점검 — trap 감지해 좌표 뽑고 trap_check에 inspect job.

        trap이 verify_timeout 프레임까지 안 보이면 사람이 아직 안 놓은 것으로 보고
        재설치(_reinstall)로 넘긴다.
        """
        box, conf = self._pick(result, 'trap', img_shape)
        if box is None:
            self._inspect_miss('trap 미검출')
            return
        xy = self._box_to_map(box, img_shape, depth, depth_frame)
        if xy is None:
            self._inspect_miss('trap 좌표 실패')
            return
        job = self._make_job('inspect', self.hole, trap=xy)
        self.job_pub.publish(job)           # 판정은 trap_check → trap_ok/bad 이벤트
        self.state = 'AWAIT_TRAP'           # trap_ok/bad 이벤트 대기
        self.get_logger().info(
            f'trap 감지 bbox=({box[0]:.0f},{box[1]:.0f},{box[2]:.0f},{box[3]:.0f}) '
            f'conf={conf:.2f} -> map=({xy[0]:.2f}, {xy[1]:.2f}) — 점검 요청')

    def _inspect_miss(self, why):
        """점검 중 trap 못 봄 — timeout 넘으면 재설치 (아직 안 놓인 것으로 봄)."""
        self.verify_count += 1
        if self.verify_count >= self.verify_timeout:
            self.get_logger().info(f'{why} — 재설치')
            self._reinstall()

    def _send_install(self):
        """install job 발행 후 설치완료 대기 (AWAIT_TRAP)."""
        self.job_pub.publish(self._make_job('install', self.hole))
        self.state = 'AWAIT_TRAP'
        self.verify_count = 0

    def _reinstall(self):
        """trap 이상/미검출 → beep+설치동작 재요청. 한도 넘으면 경고 후 순찰재개."""
        self.reinstall_count += 1
        if self.reinstall_count > self.reinstall_max:
            self.get_logger().warn(
                f'재설치 {self.reinstall_max}회 초과 — 경고, 순찰 재개')
            self._finish_opening()
            return
        self.get_logger().info(f'재설치 {self.reinstall_count}회차 — install job')
        self._send_install()

    def trap_event_cb(self, msg):
        """trap_check 응답으로 상태 전이. AWAIT_TRAP일 때만, 이벤트 이름으로 분기.

        설치완료(trap_installed) → 다시 봐서 15cm 재점검(INSPECTING). 정상(trap_ok)
        → 순찰 재개. 이상(trap_bad) → 재설치. opening_confirmed 등은 자연 무시.
        """
        if self.state != 'AWAIT_TRAP':
            return
        name, x, y = fleet_msg.parse_event(msg.data)
        if name == 'trap_installed':
            self.state = 'INSPECTING'       # 설치동작 끝 — 15cm 재점검
            self.verify_count = 0
        elif name == 'trap_ok':
            self.get_logger().info('점검 정상 — 순찰 재개')
            self._finish_opening()
        elif name == 'trap_bad':
            self.get_logger().info('설치 이상 — 재설치')
            self._reinstall()

    def _make_job(self, phase, hole, trap=(0.0, 0.0)):
        job = TrapJob()
        job.phase = phase
        job.hole_x, job.hole_y = float(hole[0]), float(hole[1])
        job.trap_x, job.trap_y = float(trap[0]), float(trap[1])
        return job

    def _hold_patrol(self, hold):
        self.hold_pub.publish(Bool(data=hold))

    def _finish_opening(self):
        """opening 처리 종료 — 순찰 재개하고 SEARCHING으로."""
        self._hold_patrol(False)
        self._reset()

    def _reset(self):
        self.state = 'SEARCHING'
        self.target = None
        self.goal = None
        self.hole = None
        self.verify_count = 0
        self.reinstall_count = 0

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


def _self_check():
    # 포획: 반경 안에서 capture_secs 머무르면 True
    rt = RatTracker(capture_radius=0.3, capture_secs=10.0, lost_secs=5.0)
    assert rt.seen(1.0, 1.0, 0.0) is False          # 방금 자리잡음
    assert rt.seen(1.1, 1.05, 5.0) is False          # 반경 안, 아직 5초
    assert rt.seen(1.05, 1.1, 10.0) is True          # 10초 경과 → 포획
    # 움직이면 anchor 리셋되어 타이머 다시 시작
    rt.reset()
    assert rt.seen(0.0, 0.0, 0.0) is False
    assert rt.seen(0.0, 0.0, 9.0) is False
    assert rt.seen(5.0, 5.0, 9.5) is False           # 반경 밖 → 재설정(t=9.5)
    assert rt.seen(5.0, 5.0, 20.0) is True           # 9.5+10.5초 → 새 자리서 포획
    # 놓침: 마지막 감지 후 lost_secs 경과
    rt.reset()
    assert rt.is_lost(100.0) is False                # 추적 시작 전 → 놓침 아님
    rt.seen(2.0, 2.0, 100.0)
    assert rt.is_lost(104.0) is False                # 4초 → 아직
    assert rt.is_lost(105.0) is True                 # 5초 → 놓침
    # TRACK 진입 후 관측 0회 → 시작 시각 기준으로 놓침 (문제 5: 영구 고착 방지)
    rt.start(10.0)
    assert rt.is_lost(14.9) is False                 # 아직 5초 안 됨
    assert rt.is_lost(15.0) is True                  # 관측 없이 5초 → 놓침
    # 관측이 생기면 last_seen 기준으로 연장
    rt.start(0.0)
    rt.seen(1.0, 1.0, 3.0)
    assert rt.is_lost(7.9) is False and rt.is_lost(8.0) is True

    # opening watchdog 판정 (문제 4: 영구 대기 방지)
    assert watchdog_action('SEARCHING', 9999.0) is None       # 대기 상태 아님
    assert watchdog_action('APPROACHING', 59.0) is None       # 한도 내
    assert watchdog_action('APPROACHING', 61.0) == 'finish'   # 접근 포기
    assert watchdog_action('QUERYING', 6.0) == 'verify'       # db 무응답 → 검증 진행
    assert watchdog_action('AWAIT_TRAP', 21.0) == 'finish'    # trap_check 무응답
    assert watchdog_action('VERIFYING', 31.0) == 'finish'     # 카메라 사망 백스톱
    print('detector RatTracker self-check ok')


if __name__ == '__main__':
    import sys
    _self_check() if '--check' in sys.argv else main()
