"""로봇 카메라 감지 — YOLO로 opening·rat 감지.

opening: DB에 좌표 조회 → 없으면 접근·depth_spread 검증(기존 camera_node 로직
이동) → 진짜 구멍이면 trap 설치단계(로그)+DB기록. 있으면 trap 점검으로.
rat: target_pose로 추적 goal + /fleet/event 발행.

oakd 원본 rgb·depth·camera_info를 직접 sync한다 (camera_node 통합). namespace로 실행.
opening 검증만 실제 동작, rat·DB 분기는 TODO(팀원).
"""
import math
import os

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  PointStamped TF 변환 등록
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from turtle_interfaces.msg import TrapJob
from turtle_interfaces.srv import ListHoles, QueryHole
from turtle_project import fleet_msg
from turtle_project.depth_math import (decode_depth, depth_at, depth_spread,
                                       deproject, side_px, to_depth_px,
                                       wall_depth)
from turtle_project.nav_controller import approach_point, make_pose

FRAME = 'map'

# oakd 원본 토픽 (상대 — __ns가 접두). camera_node를 통합해 여기서 직접 sync한다.
RGB_IN = 'oakd/rgb/image_raw/compressed'
DEPTH_IN = 'oakd/stereo/image_raw/compressedDepth'
INFO_IN = 'oakd/stereo/camera_info'

# 상태별 wall timeout(초) — 어떤 상태에서도 영구 대기하지 않도록 한다.
# VERIFYING/INSPECTING은 프레임 카운트 timeout이 따로 있지만, 카메라가 죽어
# 프레임 자체가 끊기면 그것도 안 돌므로 wall 백스톱을 함께 둔다.
DEADLINES = {'APPROACHING': 60.0, 'QUERYING': 5.0, 'VERIFYING': 30.0,
             'INSPECTING': 30.0, 'AWAIT_TRAP': 35.0, 'SWEEP_NAV': 60.0,
             'SWEEP_WAIT': 10.0}


def marker_spec(name):
    """클래스명 → (opening이면 wall depth 쓸까?, RViz 마커 (r,g,b) 색).

    opening은 관통 depth를 피하려고 벽면 밴드로 좌표를 잡아야 하니 wall=True.
    rat/trap은 실물이라 중심 depth(False). 미지 클래스는 회색.
    """
    color = {'opening': (1.0, 0.2, 0.2), 'rat': (1.0, 1.0, 0.2),
             'trap': (0.2, 0.5, 1.0)}.get(name, (0.6, 0.6, 0.6))
    return name == 'opening', color


def nearest_hole(holes, rx, ry):
    """(rx, ry)에서 가장 가까운 구멍의 인덱스. 목록 비면 None. greedy 순회용."""
    if not holes:
        return None
    return min(range(len(holes)),
               key=lambda i: math.hypot(holes[i][0] - rx, holes[i][1] - ry))


def lost_action(lost, spin_until, now):
    """놓침 처리 결정 — None|'spin_start'|'spin_stop'|'lost'. 순수함수.

    안 보인다고 바로 rat_lost 하지 않고 제자리 탐색 회전 한 바퀴를 먼저
    시도한다. 회전 중 재발견하면 추적 재개, 끝까지 없으면 진짜 놓침.
    """
    if spin_until is not None:
        if not lost:
            return 'spin_stop'
        return 'lost' if now >= spin_until else None
    return 'spin_start' if lost else None


def watchdog_action(state, elapsed):
    """상태·경과시간 -> timeout 처리 ('verify'|'finish'|None). 순수함수.

    QUERYING timeout은 db가 응답 없는 것 — 처음 본 셈 치고 검증으로 진행.
    나머지는 opening 작업을 포기하고 순찰 재개(finish).
    """
    limit = DEADLINES.get(state)
    if limit is None or elapsed <= limit:
        return None
    return 'verify' if state == 'QUERYING' else 'finish'


def recently_done(xy, recent, recent_t, now, cooldown, dist):
    """방금 처리 끝낸 구멍(recent) 근처를 cooldown 안에 또 감지했으면 True. 순수함수.
    로봇이 구멍 앞에 선 채 완료 → 다음 프레임 재감지 → 무한 재점검을 막는다."""
    if recent is None or now - recent_t > cooldown:
        return False
    return math.hypot(xy[0] - recent[0], xy[1] - recent[1]) <= dist


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
        res = os.path.join(get_package_share_directory('turtle_project'), 'resource')
        self.model_path = self.declare_parameter(
            'model_path', os.path.join(res, 'best.pt')).value
        self.conf = self.declare_parameter('conf', 0.6).value
        # 구멍 앞(approach_dist)까지 가면 구멍이 화면을 크게 채워 스케일이
        # 학습 데이터와 달라져 opening 확신도가 conf 밑으로 떨어진다 → 재검출
        # 실패로 검증을 못 한다. '이미 구멍 앞에 간' 재검출(접근·verify)만
        # 이 낮은 문턱을 쓴다. 순찰 중 최초 발견은 오탐 방지로 conf(0.6) 유지.
        self.opening_verify_conf = self.declare_parameter(
            'opening_verify_conf', 0.4).value
        self.approach_dist = self.declare_parameter('approach_dist', 0.8).value
        self.arrive_tol = self.declare_parameter('arrive_tol', 0.27).value
        # 방금 처리 끝낸 구멍을 그 자리에서 곧바로 재감지해 무한 재점검하는 걸 막는다.
        # 이 시간(초)/거리(m) 안이면 무시 → 패트롤이 로봇을 그 앞에서 치울 시간 확보.
        self.hole_cooldown = self.declare_parameter('hole_cooldown', 20.0).value
        self.hole_cooldown_dist = self.declare_parameter('hole_cooldown_dist', 0.5).value
        self.depth_gap = self.declare_parameter('depth_gap', 0.05).value
        self.side_margin = self.declare_parameter('side_margin', 0.05).value
        self.verify_timeout = self.declare_parameter('verify_timeout', 30).value
        cap_r = self.declare_parameter('capture_radius', 0.3).value
        cap_s = self.declare_parameter('capture_secs', 10.0).value
        lost_s = self.declare_parameter('lost_secs', 5.0).value
        self.reinstall_max = self.declare_parameter('reinstall_max', 3).value
        # 추적 goal 재발행 주기(초) — 매 프레임 쏘면 Nav2 goal이 폭주한다.
        self.rat_goal_period = self.declare_parameter('rat_goal_period', 1.0).value
        # 접근 중 opening 재감지 goal 갱신 주기(초). 가까울수록 depth가 정확해져 목표가 자기교정된다. 매 프레임 갱신하면 Nav2가 계속 재계획해 덜컹댄다.
        self.approach_goal_period = self.declare_parameter(
            'approach_goal_period', 0.5).value
        # 쥐 놓침 직전 제자리 탐색 회전 시간(초) — 이 시간에 정확히 한 바퀴 돌도록 각속도를 정하므로(2π/spin_secs) 시간이 곧 회전 속도다.
        self.spin_secs = self.declare_parameter('search_spin_secs', 10.0).value
        # 디버그 창 (show:=true). headless/SSH에선 imshow가 터지므로 기본 off.
        self.show = self.declare_parameter('show', False).value

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
        self.recent_hole = None   # 방금 처리 끝낸 구멍 좌표 (cooldown 재감지 억제용)
        self.recent_hole_t = 0.0
        # 쥐 추적 판정기 (포획/놓침). tracking True일 때만 굴린다.
        self.rat = RatTracker(cap_r, cap_s, lost_s)
        self.tracking = False
        self._last_rat_goal = 0.0   # 마지막 추적 goal 발행 시각 (rat_goal_period 조절용)
        self._last_appr_goal = 0.0  # 마지막 접근 goal 갱신 시각 (approach_goal_period용)
        self.spin_until = None      # 탐색 회전 종료 시각. None=회전 안 함
        # 쥐대응 B 역할 — DB 구멍을 가까운 순서로 순회 점검(SWEEP). sweeping True일 때만.
        self.sweeping = False
        self.sweep_remaining = []   # 아직 점검 안 한 구멍 [(x, y)]

        self.event_pub = self.create_publisher(String, '/fleet/event', 10)
        # detector는 goal을 직접 안 쏜다. 접근·추적 목표는 target_pose로 발행만
        # 하고, 실제 Nav2 주행은 robot_agent가 한다 (로봇당 Nav2 주인 1개).
        self.pose_pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        # opening 처리 동안 순찰을 멈춰둔다 (명시 신호). robot_agent가 구독.
        self.hold_pub = self.create_publisher(Bool, 'patrol_hold', 10)
        # trap 설치/점검 지시 — trap_check가 받아 주행/판정만 한다.
        self.job_pub = self.create_publisher(TrapJob, 'trap_job', 10)
        # 탐색 회전용 직접 주행 — Nav2 goal로는 회전 속도를 못 정해서 cmd_vel.
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # 탐색 회전 전 진행 중인 Nav2 goal을 끊어달라는 신호 (robot_agent 구독).
        # 안 끊으면 Nav2 controller와 spin_tick이 cmd_vel을 놓고 싸운다.
        self.cancel_pub = self.create_publisher(Bool, 'cancel_drive', 10)
        # YOLO가 잡은 것들을 map 위 마커로 (진단·시각화). RViz에서 좌표가
        # 튀는지/고정인지 눈으로 확인 → 노이즈 vs 계통 오차 구분에 쓴다.
        self.marker_pub = self.create_publisher(MarkerArray, 'detections', 10)
        self.db = self.create_client(QueryHole, '/db/query_hole')
        self.db_list = self.create_client(ListHoles, '/db/list_holes')

        self.rgb_sub = Subscriber(self, CompressedImage, RGB_IN,
                                  qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, CompressedImage, DEPTH_IN,
                                    qos_profile=qos_profile_sensor_data)
        self.info_sub = Subscriber(self, CameraInfo, INFO_IN,
                                   qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub],
            queue_size=10, slop=0.1)
        self.sync.registerCallback(self.synced_cb)
        # 쥐 추적은 central의 TRACK 명령으로 켠다 (내 로봇이 A일 때만).
        self.create_subscription(String, '/fleet/command', self.command_cb, 10)
        # trap_check가 발행하는 설치완료/점검결과를 받아 상태를 전이한다.
        # 상대 토픽(네임스페이스 분리) — /fleet/event를 쓰면 다른 로봇의
        # trap_ok/installed를 내 것으로 오인해 상태가 잘못 전이된다.
        self.create_subscription(String, 'trap_event', self.trap_event_cb, 10)
        # 놓침은 감지가 끊긴 걸로 판단 — 프레임 콜백이 안 오므로 타이머로 감시.
        self.create_timer(1.0, self.lost_tick)
        # 탐색 회전 유지 — Create3는 cmd_vel이 끊기면 곧 멈추므로 10Hz로 쏜다.
        self.create_timer(0.1, self.spin_tick)
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
        if self.state == 'SWEEP_NAV':       # 순회 중 구멍으로 이동 — 도착만 감시
            self._check_sweep_arrival()
            if self.show:
                self._show(self.bridge.compressed_imgmsg_to_cv2(rgb_msg, 'bgr8'))
            return
        if self.state in ('QUERYING', 'AWAIT_TRAP'):
            return                          # db 응답/trap_check 응답 대기 — 지각 불필요
        self.K = np.array(info_msg.k).reshape(3, 3)
        img = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = decode_depth(depth_msg.data, depth_msg.format)
        if depth is None:       # 손상 프레임 — imdecode 실패가 노드를 죽이면 안 됨
            self.get_logger().warn('depth 디코드 실패 — 프레임 스킵',
                                   throttle_duration_sec=5.0)
            return
        depth_frame = depth_msg.header.frame_id
        # 바닥값은 클래스별 문턱 중 가장 낮은 값 — 여기서 걸러버리면 _pick이
        # 클래스별로 다시 못 살린다. 클래스별 최종 문턱은 _pick의 min_conf.
        result = self.model(img, conf=min(self.conf, self.opening_verify_conf),
                            verbose=False)[0]
        if self.show:
            self._show(result.plot())       # ultralytics가 박스·라벨까지 그려준다
        self._publish_markers(result, img.shape, depth, depth_frame)

        if self.state == 'APPROACHING':     # 주행은 robot_agent — 접근하며 목표 갱신
            self._refresh_approach(result, img.shape, depth, depth_frame)
            self._check_arrival()
            return

        # rat 감지 → 추적 goal/포획 판정 (SEARCHING·TRACK 무관하게 매 프레임 확인).
        self._detect_rat(result, img.shape, depth, depth_frame)

        # 쥐대응 중엔 opening 무시 — A는 추적만, B는 순회만. 여기서 opening 흐름에
        # 들어가면 APPROACHING 조기 return으로 _detect_rat이 끊겨 rat_lost로
        # 쥐대응 전체가 조기 종료된다.
        if self.state == 'SEARCHING' and not (self.tracking or self.sweeping):
            box = self._pick(result, 'opening', img.shape)
            if box is not None:
                self._on_opening(box, img.shape, depth, depth_frame)
        elif self.state == 'VERIFYING':
            self._verify(self._pick(result, 'opening', img.shape,
                                    self.opening_verify_conf), img.shape, depth)
        elif self.state == 'INSPECTING':
            self._inspect(result, img.shape, depth, depth_frame)

    def _detect_rat(self, result, img_shape, depth, depth_frame):
        """쥐 감지 → map좌표 → RatTracker 판정. 포획이면 rat_captured 발행.

        TRACK 모드(self.tracking)일 때만 포획/놓침을 판정한다. 순찰 중 우연히
        쥐가 보이면 rat_detected만 쏴 central이 쥐대응을 시작하게 한다.
        """
        box = self._pick(result, 'rat', img_shape)
        if box is None:
            return
        xy = self._box_to_map(box, img_shape, depth, depth_frame)
        if xy is None:
            return
        if not self.tracking:
            # 아직 쥐대응 전 — 최초 감지만 알림 (central이 역할배정 시작).
            self.event_pub.publish(String(data=fleet_msg.event(
                'rat_detected', *xy)))
            return
        # 추적 중 — 쥐 위치로 추적 goal 발행 + 포획 판정. robot_agent가 target_pose를
        # 받아 실제로 몬다. 매 프레임 쏘면 goal 폭주라 rat_goal_period마다 한 번만.
        if self.spin_until is not None:     # 탐색 회전 중 재발견 — 즉시 회전 정지
            self._stop_spin('탐색 회전 중 쥐 재발견 — 추적 재개')
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

    def _show(self, img):
        """디버그 창. 상태를 같이 찍어 지금 어느 단계인지 화면에서 보이게 한다."""
        cv2.putText(img, f'{self.robot_id} {self.state}', (8, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow('detector', img)
        cv2.waitKey(1)

    def _publish_markers(self, result, img_shape, depth, depth_frame):
        """감지된 박스 전부를 map 위 SPHERE 마커로 발행 (클래스별 색, 실시간).

        매 프레임 DELETEALL 후 다시 그려 안 보이는 건 바로 사라진다(잔상 없음).
        opening은 벽면 depth, rat/trap은 중심 depth로 좌표를 잡는다(marker_spec).
        """
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL     # 지난 프레임 마커 싹 지우고 새로
        arr.markers.append(clear)
        i = 0
        for b in result.boxes:
            name = self.model.names[int(b.cls)]
            wall, (r, g, bl) = marker_spec(name)
            xy = self._box_to_map(b.xyxy[0].tolist(), img_shape, depth,
                                  depth_frame, wall=wall)
            if xy is None:
                continue
            m = Marker()
            m.header.frame_id = FRAME
            m.ns, m.id, m.type, m.action = name, i, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = \
                xy[0], xy[1], 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, bl, 0.9
            arr.markers.append(m)
            i += 1
        self.marker_pub.publish(arr)

    def _pick(self, result, cls_name, img_shape, min_conf=None):
        """cls_name 박스 중 화면 중앙에 가장 가까운 것 -> xyxy or None.

        min_conf: 이 클래스에 적용할 최소 확신도. None이면 conf(기본 0.6).
        모델은 낮은 바닥값으로 돌리므로 클래스별 실제 문턱은 여기서 건다.
        """
        if self.model is None:
            return None
        if min_conf is None:
            min_conf = self.conf
        cx = img_shape[1] / 2
        best, best_d = None, None
        for b in result.boxes:
            if self.model.names[int(b.cls)] != cls_name:
                continue
            if float(b.conf) < min_conf:
                continue                # 이 클래스 문턱 미달 — 버림
            u = float(b.xywh[0][0])
            d = abs(u - cx)
            if best_d is None or d < best_d:
                best_d, best = d, b.xyxy[0].tolist()
        return best

    def _on_opening(self, box, img_shape, depth, depth_frame):
        """opening 감지 → 접근 goal 계산 성공 후에만 순찰 정지(hold).

        hold를 먼저 걸면 TF 실패 시 순찰만 멈추고 접근은 안 하는 채로 남는다
        — hold는 _start_approach 안에서 goal이 확정된 뒤 건다.
        """
        xy = self._box_to_map(box, img_shape, depth, depth_frame, wall=True)
        if xy is None:
            return
        if recently_done(xy, self.recent_hole, self.recent_hole_t, self._now(),
                         self.hole_cooldown, self.hole_cooldown_dist):
            return          # 방금 처리한 구멍 — 패트롤이 지나갈 때까지 무시
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
        if self.state != 'QUERYING':
            return          # watchdog timeout으로 이미 넘어감 — 늦은 응답 무시
        try:
            resp = fut.result()
        except Exception as e:
            self.get_logger().warn(f'db 응답 실패 ({e}) — 처음 본 셈 치고 검증')
            self._enter_verify()
            return
        if resp.exists:
            self.hole = (resp.hole_x, resp.hole_y)   # 15cm 비교 기준
            self.state = 'INSPECTING'
            self.get_logger().info('기존 구멍 — trap 점검 시작')
        else:
            self._enter_verify()

    def _enter_verify(self):
        self.state = 'VERIFYING'
        self.verify_count = 0

    def _box_to_map(self, box, img_shape, depth, depth_frame, wall=False):
        """bbox 중심 → depth → deproject → TF map좌표 (x, y). 무효면 None.

        wall=True(opening용): 중심 depth는 구멍을 관통해 벽 '뒤'를 재므로,
        그대로 쓰면 접근점이 벽 안/뒤에 찍혀 Nav2가 경로를 못 만든다(FAILED).
        구멍 옆 벽면 밴드의 depth로 좌표를 벽면 위에 잡는다. rat/trap은 실물이라
        중심 depth가 맞다 — 기본값 False.
        """
        du1, dv1 = to_depth_px(box[0], box[1], img_shape, depth.shape)
        du2, dv2 = to_depth_px(box[2], box[3], img_shape, depth.shape)
        du, dv = (du1 + du2) // 2, (dv1 + dv2) // 2
        z = None
        if wall:
            # 밴드폭은 bbox 폭에 비례 — 멀어져 bbox가 작아지면 밴드도 준다.
            z = wall_depth(depth, du1, dv1, du2, dv2,
                           side=max(4, (du2 - du1) // 4))
        # 구멍 안쪽은 어둡고 텍스처가 없어 stereo가 매칭을 못 하는 경우도 있다 —
        # 그땐 중심이 0(무효). 벽 밴드도 무효면 중심 → bbox 전체 순서로 폴백.
        z = z or depth_at(depth, du, dv) or depth_at(
            depth, du, dv, patch=max(du2 - du1, dv2 - dv1) // 2)
        if not z:
            self.get_logger().warn('감지했지만 depth 전부 무효 — 좌표 못 냄',
                                   throttle_duration_sec=2.0)
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

    def _refresh_approach(self, result, img_shape, depth, depth_frame):
        """APPROACHING 중 opening 재감지 → 접근 goal 갱신 (approach_goal_period).

        가까워질수록 depth가 정확해져 목표가 자기교정된다. 재감지 실패(구멍이
        프레임 밖 등)면 기존 goal 유지 — 그대로 마저 간다. hold heartbeat는
        watchdog_tick이 따로 쏘므로 여기선 goal만 다룬다.
        """
        now = self._now()
        if now - self._last_appr_goal < self.approach_goal_period:
            return
        box = self._pick(result, 'opening', img_shape, self.opening_verify_conf)
        if box is None:
            return
        xy = self._box_to_map(box, img_shape, depth, depth_frame, wall=True)
        if xy is None:
            return
        robot = self.robot_xy()
        if robot is None:
            return
        rx, ry = robot
        g = approach_point(xy[0] - rx, xy[1] - ry, self.approach_dist)
        self.target = xy                    # 검증·db 조회는 갱신된 구멍좌표 기준
        self._last_appr_goal = now
        if g is None:
            return                          # 이미 approach_dist 안 — _check_arrival이 처리
        gx, gy, yaw = rx + g[0], ry + g[1], g[2]
        self.goal = (gx, gy)
        self.pose_pub.publish(make_pose(FRAME, gx, gy, yaw))

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
        """내 로봇 명령 처리 — A(TRACK)면 추적, B(SWEEP)면 구멍 순회 점검.

        쥐대응 종료 명령(PATROL/STOP/DOCK)이 오면 추적·순회를 모두 끈다.
        """
        robot, cmd = fleet_msg.parse_command(msg.data)
        if robot != self.robot_id:
            return                  # 내 로봇 명령 아님 (robot6용 TRACK 등)
        if cmd == 'TRACK' and not self.tracking:
            if self.state != 'SEARCHING':
                # opening 처리 중이면 프레임이 조기 return돼 _detect_rat이 안 돌고,
                # lost 타이머만 돌아 쥐를 한 번도 안 쫓고 rat_lost가 난다.
                # 쥐대응이 우선 — opening 작업을 접고 추적부터.
                self.get_logger().info('TRACK 수신 — 진행 중이던 opening 작업 중단')
                self._finish_opening()
            self.tracking = True
            self.rat.start(self._now())  # 관측 0회여도 lost_secs 뒤 놓침 판정
            self.get_logger().info('TRACK 시작 — 쥐 포획/놓침 판정 on')
        elif cmd == 'SWEEP' and not self.sweeping:
            self._start_sweep()     # B 역할 — DB 구멍 순회 점검 시작
        elif cmd in ('PATROL', 'STOP', 'DOCK'):
            if self.tracking:
                self._stop_tracking()   # 쥐대응 종료 — 추적 판정 off
            if self.sweeping:
                self._stop_sweep()      # 쥐대응 종료 — 순회 즉시 중단

    def lost_tick(self):
        """추적 놓침 감시 — 안 보이면 제자리 탐색 회전부터, 그래도 없으면 rat_lost.

        회전은 Nav2가 아니라 cmd_vel 직접(spin_tick) — 마지막 추적 goal은
        lost_secs 전 것이라 보통 끝나 있어 Nav2 주행과 겹치지 않는다.
        """
        if not self.tracking:
            return
        now = self._now()
        act = lost_action(self.rat.is_lost(now), self.spin_until, now)
        if act == 'spin_start':
            self.cancel_pub.publish(Bool(data=True))    # 남은 추적 goal 끊기
            self.spin_until = now + self.spin_secs
            self.get_logger().info(
                f'쥐 안 보임 — 제자리 탐색 회전 {self.spin_secs:.0f}초')
        elif act == 'spin_stop':
            self._stop_spin('탐색 회전 중 쥐 재발견 — 추적 재개')
        elif act == 'lost':
            self._stop_spin(None)
            self.get_logger().info('쥐 놓침 판정 — rat_lost 발행')
            # 놓친 위치는 마지막 anchor (없으면 0,0 — central은 이름만 보고 판단).
            x, y = self.rat.anchor or (0.0, 0.0)
            self.event_pub.publish(String(data=fleet_msg.event('rat_lost', x, y)))
            self._stop_tracking()

    def spin_tick(self):
        """탐색 회전 중 10Hz로 회전 명령 유지. spin_secs에 정확히 한 바퀴."""
        if self.spin_until is None:
            return
        t = Twist()
        t.angular.z = 2.0 * math.pi / self.spin_secs
        self.cmd_pub.publish(t)

    def _stop_spin(self, why):
        self.spin_until = None
        self.cmd_pub.publish(Twist())       # 회전 정지
        if why:
            self.get_logger().info(why)

    def _stop_tracking(self):
        if self.spin_until is not None:
            self._stop_spin(None)           # 명령(STOP/PATROL)으로 끝나도 회전 정지
        self.tracking = False
        self.rat.reset()

    # --- 쥐대응 B 역할: DB 구멍 순회 점검(SWEEP) ---

    def _start_sweep(self):
        """SWEEP 진입 — DB에서 구멍 목록을 받아 순회 점검을 시작한다."""
        if not self.db_list.service_is_ready():
            self.get_logger().warn('db 목록 서비스 없음 — 점검할 구멍 없음, 완료')
            self._sweep_done_and_finish()
            return
        self.sweeping = True
        # SEARCHING이면 watchdog·heartbeat가 안 돌아 목록 응답이 영영 안 오면
        # 여기서 영구 스톨한다 — 대기 전용 상태로 바꿔 watchdog가 커버하게.
        self.state = 'SWEEP_WAIT'
        self._hold_patrol(True)         # 순회 동안 순찰 정지
        self.get_logger().info('SWEEP 시작 — DB 구멍 순회 점검')
        self.db_list.call_async(ListHoles.Request()).add_done_callback(
            self._holes_done)

    def _holes_done(self, fut):
        """DB 구멍 목록 수신 → 순회 시작. 중간에 종료됐으면 무시."""
        if not self.sweeping:
            return
        resp = fut.result()
        self.sweep_remaining = list(zip(resp.xs, resp.ys))
        self.get_logger().info(f'점검 대상 구멍 {len(self.sweep_remaining)}개')
        self._next_sweep_hole()

    def _next_sweep_hole(self):
        """가장 가까운 구멍으로 이동 시작. 남은 구멍 없으면 sweep_done."""
        if not self.sweeping:
            return
        robot = self.robot_xy()
        if robot is None:
            self.state = 'SWEEP_WAIT'   # TF 실패 — watchdog가 10초 뒤 재시도
            return
        rx, ry = robot
        idx = nearest_hole(self.sweep_remaining, rx, ry)
        if idx is None:
            self.get_logger().info('구멍 순회 완료 — sweep_done 발행')
            self._sweep_done_and_finish()
            return
        self.hole = self.sweep_remaining.pop(idx)
        self.verify_count = 0
        g = approach_point(self.hole[0] - rx, self.hole[1] - ry,
                           self.approach_dist)
        if g is None:                   # 이미 구멍 앞 — 바로 점검
            self.state = 'INSPECTING'
            return
        gx, gy, yaw = rx + g[0], ry + g[1], g[2]
        self.pose_pub.publish(make_pose(FRAME, gx, gy, yaw))
        self.goal = (gx, gy)
        self.state = 'SWEEP_NAV'
        self.get_logger().info(
            f'구멍 ({self.hole[0]:.2f}, {self.hole[1]:.2f}) 점검하러 이동 '
            f'(남은 {len(self.sweep_remaining)}개)')

    def _check_sweep_arrival(self):
        """SWEEP_NAV 중 구멍 근처(arrive_tol) 도착하면 INSPECTING으로."""
        robot = self.robot_xy()
        if robot is None or self.goal is None:
            return
        d = math.hypot(self.goal[0] - robot[0], self.goal[1] - robot[1])
        if d <= self.arrive_tol:
            self.state = 'INSPECTING'
            self.verify_count = 0
            self.get_logger().info('구멍 도착 — trap 점검')

    def _sweep_done_and_finish(self):
        self.event_pub.publish(String(data=fleet_msg.event('sweep_done', 0.0, 0.0)))
        self._finish_sweep()

    def _stop_sweep(self):
        """쥐대응 종료 — 순회 즉시 중단, 순찰 상태로 복귀 (sweep_done 안 쏨)."""
        self.get_logger().info('SWEEP 중단 — 복귀')
        self._finish_sweep()

    def _finish_sweep(self):
        self.sweeping = False
        self.sweep_remaining = []
        self._hold_patrol(False)
        self._reset()

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
            if self.sweeping:               # 순회 중 timeout — 이 구멍 스킵, 다음 구멍
                self.get_logger().warn(f'{self.state} timeout — 이 구멍 스킵')
                self._next_sweep_hole()
            else:
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
        box = self._pick(result, 'trap', img_shape)
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
        self.get_logger().info(f'trap 감지 ({xy[0]:.2f}, {xy[1]:.2f}) — 점검 요청')

    def _inspect_miss(self, why):
        """점검 중 trap 못 봄 — timeout 넘으면 재설치 (아직 안 놓인 것으로 봄).

        순회 점검(SWEEP) 중이면 재설치 없이 다음 구멍으로 넘어간다.
        """
        self.verify_count += 1
        if self.verify_count >= self.verify_timeout:
            if self.sweeping:
                self.get_logger().info(f'{why} — 다음 구멍')
                self._next_sweep_hole()
                return
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
        if self.sweeping:               # 순회 점검 — 결과 기록만, 재설치 없이 다음 구멍
            if name in ('trap_ok', 'trap_bad'):
                self.get_logger().info(f'구멍 점검 결과 {name} — 다음 구멍')
                self._next_sweep_hole()
            return
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
        """opening 처리 종료 — 순찰 재개하고 SEARCHING으로.

        처리한 구멍 좌표를 기록해 두면(recent_hole) 그 앞에서 곧바로 재감지해도
        cooldown 동안 무시된다 → 패트롤이 로봇을 치울 시간을 준다."""
        hole = self.hole or self.target
        if hole is not None:
            self.recent_hole = hole
            self.recent_hole_t = self._now()
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
    assert watchdog_action('AWAIT_TRAP', 34.0) is None        # 설치 주행(~24s) 대기 중
    assert watchdog_action('AWAIT_TRAP', 36.0) == 'finish'    # trap_check 무응답
    assert watchdog_action('VERIFYING', 31.0) == 'finish'     # 카메라 사망 백스톱
    assert watchdog_action('SWEEP_NAV', 61.0) == 'finish'     # 순회 이동 백스톱
    assert watchdog_action('SWEEP_WAIT', 9.0) is None         # 목록/TF 대기 중
    assert watchdog_action('SWEEP_WAIT', 11.0) == 'finish'    # 대기 초과 → 재시도

    # recently_done: 방금 처리한 구멍 재감지 억제 (무한 재점검 방지)
    assert recently_done((0.0, 0.0), None, 0.0, 5.0, 20.0, 0.5) is False       # 기록 없음
    assert recently_done((0.0, 0.0), (0.1, 0.0), 0.0, 5.0, 20.0, 0.5) is True  # 근처+시간내 → 무시
    assert recently_done((1.0, 0.0), (0.0, 0.0), 0.0, 5.0, 20.0, 0.5) is False # 멀다(1m>0.5) → 처리
    assert recently_done((0.1, 0.0), (0.0, 0.0), 0.0, 25.0, 20.0, 0.5) is False # cooldown 지남 → 처리

    # lost_action: 놓침 → 즉시 포기 대신 탐색 회전 → 그래도 없으면 진짜 lost
    assert lost_action(False, None, 0.0) is None            # 정상 추적 중
    assert lost_action(True, None, 0.0) == 'spin_start'     # 1차 놓침 — 회전 시작
    assert lost_action(True, 15.0, 12.0) is None            # 회전 중 — 계속
    assert lost_action(False, 15.0, 12.0) == 'spin_stop'    # 회전 중 재발견
    assert lost_action(True, 15.0, 15.0) == 'lost'          # 회전 끝 — 진짜 놓침

    # greedy 순회 — 로봇에서 가장 가까운 구멍 인덱스 (B 구멍 순회 점검)
    assert nearest_hole([], 0.0, 0.0) is None                 # 빈 목록
    assert nearest_hole([(1.0, 0.0), (5.0, 0.0)], 0.0, 0.0) == 0
    assert nearest_hole([(5.0, 0.0), (1.0, 0.0)], 0.0, 0.0) == 1  # 순서 무관
    assert nearest_hole([(0.0, 3.0), (0.0, 1.0), (0.0, 2.0)], 2.0, 0.0) == 1

    # marker_spec: opening만 벽면 depth, 클래스별 색이 서로 달라야 구분됨
    assert marker_spec('opening')[0] is True          # opening → wall depth
    assert marker_spec('rat')[0] is False             # 실물 → 중심 depth
    assert marker_spec('trap')[0] is False
    assert marker_spec('opening')[1] != marker_spec('rat')[1]   # 색 구분
    assert marker_spec('???')[1] == (0.6, 0.6, 0.6)   # 미지 클래스 → 회색
    print('detector RatTracker self-check ok')


if __name__ == '__main__':
    import sys
    _self_check() if '--check' in sys.argv else main()
