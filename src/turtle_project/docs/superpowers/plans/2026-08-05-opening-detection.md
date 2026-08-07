# Opening 감지 시 추가 조치 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** camera_node가 YOLO로 opening을 보면 Nav2로 앞 ~50cm까지 접근한 뒤 bbox depth 편차로 진짜 구멍인지 판정하고 로그로 낸다.

**Architecture:** 자립형(self-contained) — mini_turtle4 구현 방식만 참고하고 지원 모듈(depth_math, nav_controller)을 이 워크스페이스에 새로 만든다. camera_node는 SEARCHING→APPROACHING→VERIFYING 3단계 상태머신. RGB+depth를 ApproximateTimeSynchronizer로 짝지어 받고, Nav2 도착(SUCCEEDED) 콜백으로 상태를 전이한다.

**Tech Stack:** ROS 2 Humble (rclpy), ultralytics YOLO, cv_bridge, OpenCV, numpy, message_filters, tf2, nav2_msgs.

## Global Constraints

- 패키지: `turtle_project`, ament_python, Python 3.
- 순수 함수(depth_math, nav_controller의 approach_point/make_pose)는 `if __name__ == '__main__'`에서 `_self_check()` assert로 검증. ROS/YOLO/하드웨어 경로는 테스트 제외.
- 이 워크스페이스는 git 저장소가 아니다 — 커밋 스텝은 생략한다 (사용자가 git init 하기 전까지).
- 토픽: RGB=`oakd/rgb/image_raw/compressed`, Depth=`oakd/stereo/image_raw/compressedDepth`, Info=`oakd/stereo/camera_info`.
- 파라미터 기본값: `model_path=''`, `target_class='opening'`, `conf=0.6`, `approach_dist=0.5`, `depth_gap=0.05`, `verify_timeout=30`.
- depth는 16UC1 mm 단위. 판정/거리 계산은 미터.

---

### Task 1: depth_math.py — depth 좌표/편차 순수 함수

**Files:**
- Create: `turtle_project/depth_math.py`

**Interfaces:**
- Consumes: (없음 — cv2, numpy만)
- Produces:
  - `decode_depth(data: bytes, fmt: str) -> np.ndarray`  (16UC1 mm)
  - `to_depth_px(u, v, rgb_shape, depth_shape) -> (int, int)`
  - `depth_at(depth_mm, u, v, patch=5) -> float | None`  (미터)
  - `deproject(u, v, z, K) -> (x, y, z)`  (카메라 광학 프레임)
  - `depth_spread(depth_mm, x1, y1, x2, y2, min_valid=30) -> float | None`  (미터, p90−p10)

- [ ] **Step 1: 파일 작성 (함수 5개 + self-check)**

`turtle_project/depth_math.py`:

```python
"""depth 카메라 좌표/편차 계산. ROS 무관, 노드가 import해서 사용."""
import cv2
import numpy as np

PATCH = 5   # depth 샘플 반경 (픽셀). 한 점만 보면 0(무효)이 잘 나옴


def decode_depth(data, fmt):
    """compressedDepth CompressedImage.data -> 16UC1 depth 배열 (mm).

    compressedDepth는 PNG 앞에 12바이트 ConfigHeader가 붙어 있고 cv_bridge가
    이걸 못 벗긴다. 16UC1은 양자화 없이 raw PNG라 헤더만 잘라내면 그대로 mm
    값이 나온다. 헤더 없는 일반 compressed도 통과시킨다.
    """
    buf = np.frombuffer(data, np.uint8)
    if 'compressedDepth' in fmt:
        buf = buf[12:]
    return cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)


def to_depth_px(u, v, rgb_shape, depth_shape):
    """RGB 픽셀 -> depth 픽셀. 두 이미지 해상도가 달라서 필요.

    RGB와 depth는 화각이 같게 맞춰져 있어 해상도만 비례로 맞추면 픽셀이 대응한다.
    """
    rh, rw = rgb_shape[:2]
    dh, dw = depth_shape[:2]
    return int(u / rw * dw), int(v / rh * dh)


def depth_at(depth_mm, u, v, patch=PATCH):
    """(u, v) 주변 patch의 유효값 중앙값 -> 미터. 유효값 없으면 None."""
    h, w = depth_mm.shape
    win = depth_mm[max(0, v - patch):min(h, v + patch + 1),
                   max(0, u - patch):min(w, u + patch + 1)]
    win = win[win > 0]
    return float(np.median(win)) / 1000.0 if win.size else None


def deproject(u, v, z, K):
    """depth 픽셀 + 거리 -> 카메라 광학 프레임 3D (x우, y하, z전방)."""
    return ((u - K[0, 2]) * z / K[0, 0],
            (v - K[1, 2]) * z / K[1, 1],
            z)


def depth_spread(depth_mm, x1, y1, x2, y2, min_valid=30):
    """bbox 영역 유효 depth의 p90 - p10 (미터). 유효 픽셀 부족하면 None.

    진짜 구멍이면 구멍 안쪽(멀다)과 테두리 벽(가깝다)이 섞여 편차가 크고,
    평평한 벽이면 depth가 고르니 편차가 작다. min-max 대신 퍼센타일이라
    depth 노이즈 몇 픽셀이 튀어도 안 흔들린다.
    """
    h, w = depth_mm.shape
    x1, y1 = max(0, int(x1)), max(0, int(y1))
    x2, y2 = min(w, int(x2)), min(h, int(y2))
    roi = depth_mm[y1:y2, x1:x2]
    valid = roi[roi > 0]
    if valid.size < min_valid:
        return None
    p90, p10 = np.percentile(valid, [90, 10])
    return float(p90 - p10) / 1000.0


def _self_check():
    d = np.zeros((400, 640), np.uint16)
    d[195:205, 315:325] = 1500                 # 중앙에 1.5m 블록
    assert depth_at(d, 320, 200) == 1.5
    assert depth_at(d, 10, 10) is None         # 전부 0 -> 무효
    assert depth_at(d, 0, 0) is None           # 경계에서 안 터짐

    assert to_depth_px(125, 125, (250, 250), (400, 640)) == (320, 200)
    assert to_depth_px(0, 0, (250, 250), (400, 640)) == (0, 0)

    K = np.array([[500., 0., 320.], [0., 500., 200.], [0., 0., 1.]])
    assert deproject(320, 200, 2.0, K) == (0.0, 0.0, 2.0)   # 정중앙
    x, y, z = deproject(420, 200, 2.0, K)                   # 오른쪽 100px
    assert abs(x - 0.4) < 1e-9 and abs(y) < 1e-9 and z == 2.0

    # decode_depth: 12바이트 헤더 벗김 + 헤더 없는 경우
    dd = np.zeros((704, 704), np.uint16)
    dd[340:365, 340:365] = 1234
    png = cv2.imencode('.png', dd)[1].tobytes()
    out = decode_depth(b'\x00' * 12 + png, '16UC1; compressedDepth png')
    assert out.dtype == np.uint16 and depth_at(out, 352, 352) == 1.234
    assert depth_at(decode_depth(png, '16UC1; png'), 352, 352) == 1.234

    # depth_spread: 평평한 벽 -> 작은 편차
    wall = np.full((100, 100), 1000, np.uint16)
    assert depth_spread(wall, 0, 0, 100, 100) == 0.0
    # 구멍 패턴: 절반은 1m 벽, 절반은 1.5m 구멍 안쪽 -> p90-p10 = 0.5m
    hole = np.full((100, 100), 1000, np.uint16)
    hole[:, 50:] = 1500
    assert abs(depth_spread(hole, 0, 0, 100, 100) - 0.5) < 1e-9
    # 유효 픽셀 부족 -> None
    assert depth_spread(np.zeros((100, 100), np.uint16), 0, 0, 100, 100) is None
    # bbox가 이미지 밖으로 나가도 클램프되어 안 터짐
    assert depth_spread(wall, -10, -10, 200, 200) == 0.0
    print('depth_math self-check ok')


if __name__ == '__main__':
    _self_check()
```

- [ ] **Step 2: self-check 실행**

Run: `python3 turtle_project/depth_math.py`
Expected: `depth_math self-check ok` (assert 통과)

---

### Task 2: nav_controller.py — Nav2 접근 주행 래퍼

**Files:**
- Create: `turtle_project/nav_controller.py`

**Interfaces:**
- Consumes: (없음 — nav2_msgs, geometry_msgs, rclpy.action)
- Produces:
  - `approach_point(x, y, stop_dist) -> (gx, gy, yaw) | None`
  - `make_pose(frame, x, y, yaw) -> PoseStamped`
  - `Navigator(node, on_arrived=None)` — `.go(pose) -> bool`. 도착 SUCCEEDED 시 `on_arrived()` 호출.

- [ ] **Step 1: 파일 작성 (함수 2개 + Navigator + self-check)**

`turtle_project/nav_controller.py`:

```python
"""로봇 이동 제어 (Nav2). 노드에서 import해서 사용.

    nav = Navigator(node, on_arrived=cb)
    nav.go(make_pose('map', x, y, yaw))   # 도착하면 cb() 호출
"""
import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

SUCCEEDED = 4   # action_msgs GoalStatus.STATUS_SUCCEEDED


def approach_point(x, y, stop_dist):
    """base_link 기준 물체 (x, y) -> 물체 앞 stop_dist 지점 (gx, gy, yaw).

    물체를 바라보는 방향으로 yaw를 잡는다. 이미 stop_dist 안이면 None.
    """
    d = math.hypot(x, y)
    if d <= stop_dist:
        return None
    r = (d - stop_dist) / d
    return x * r, y * r, math.atan2(y, x)


def make_pose(frame, x, y, yaw):
    """(x, y, yaw) -> PoseStamped. stamp는 0 = TF 최신값 사용."""
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2)
    p.pose.orientation.w = math.cos(yaw / 2)
    return p


class Navigator:
    """NavigateToPose 액션 래퍼. 한 번에 goal 하나. 도착하면 on_arrived 콜백.

    spin 재탐색은 이 기능(일회성 접근)에 불필요하여 제외.
    """

    def __init__(self, node, on_arrived=None, action='navigate_to_pose'):
        self.log = node.get_logger()
        self.client = ActionClient(node, NavigateToPose, action)
        self.on_arrived = on_arrived
        self.active = False
        self.handle = None

    def go(self, pose):
        """goal 전송 (비동기). 서버가 받아들이면 True.

        주행 중에 다시 불러도 Nav2가 선점(preempt)해서 목표만 갈아끼운다.
        """
        if not self.client.server_is_ready():
            self.log.warn('navigate_to_pose 서버 아직 없음 — Nav2가 켜져 있나요?')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.active = True
        self.client.send_goal_async(goal).add_done_callback(self._accepted)
        self.log.info(f'goal 전송: ({pose.pose.position.x:.2f}, '
                      f'{pose.pose.position.y:.2f}) @ {pose.header.frame_id}')
        return True

    def cancel(self):
        if self.handle:
            self.handle.cancel_goal_async()

    def _accepted(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.log.error('goal 거부됨')
            self.active = False
            return
        self.handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._done(f, handle))

    def _done(self, fut, handle):
        if handle is not self.handle:
            return              # 선점당한 예전 goal — 무시
        self.active = False
        self.handle = None
        status = fut.result().status
        self.log.info(f'주행 종료 (status={status})')
        if status == SUCCEEDED and self.on_arrived:
            self.on_arrived()


def _self_check():
    x, y, yaw = approach_point(2.0, 0.0, 0.5)          # 정면 2m
    assert abs(x - 1.5) < 1e-9 and abs(y) < 1e-9 and abs(yaw) < 1e-9

    x, y, yaw = approach_point(0.0, 2.0, 0.5)          # 왼쪽 2m
    assert abs(x) < 1e-9 and abs(y - 1.5) < 1e-9
    assert abs(yaw - math.pi / 2) < 1e-9

    x, y, _ = approach_point(3.0, 4.0, 1.0)            # d=5 -> r=0.8
    assert abs(x - 2.4) < 1e-9 and abs(y - 3.2) < 1e-9

    assert approach_point(0.3, 0.0, 0.5) is None       # 이미 가까움

    p = make_pose('map', 1.0, 0.0, math.pi / 2)
    assert abs(p.pose.orientation.z - math.sqrt(0.5)) < 1e-9
    assert abs(p.pose.orientation.w - math.sqrt(0.5)) < 1e-9
    print('nav_controller self-check ok')


if __name__ == '__main__':
    _self_check()
```

- [ ] **Step 2: self-check 실행**

Run: `python3 turtle_project/nav_controller.py`
Expected: `nav_controller self-check ok`

주의: import는 ROS 환경(`source /opt/ros/humble/setup.bash`)이 필요할 수 있다. `_self_check`는 approach_point/make_pose만 쓰므로 geometry_msgs/nav2_msgs import만 되면 통과한다.

---

### Task 3: camera_node.py — 상태머신 본체

**Files:**
- Modify: `turtle_project/camera_node.py` (현재 스켈레톤 전체 교체)

**Interfaces:**
- Consumes: `depth_math.{decode_depth, to_depth_px, depth_at, deproject, depth_spread}`, `nav_controller.{Navigator, approach_point, make_pose}`
- Produces: `main()` 엔트리포인트 (setup.py가 이미 `camera_node:main` 등록)

- [ ] **Step 1: 파일 전체 교체**

`turtle_project/camera_node.py`:

```python
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
                                       deproject, to_depth_px)
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
            m = YOLO(self.model_path)
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
        """target_class 박스 중 depth 유효한 것 하나 -> box(xyxy) or None.

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
        gap = depth_spread(depth, du1, dv1, du2, dv2)
        if gap is None:
            self.verify_count += 1
            if self.verify_count >= self.verify_timeout:
                self.get_logger().info('depth 유효 픽셀 부족 — 포기, 복귀')
                self._reset()
            return
        if gap >= self.depth_gap:
            self.get_logger().info(f'진짜 opening 확인 (gap={gap:.3f}m)')
        else:
            self.get_logger().info(f'opening 아님 (gap={gap:.3f}m < {self.depth_gap})')
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
```

- [ ] **Step 2: import/문법 확인**

Run: `python3 -c "import ast; ast.parse(open('turtle_project/camera_node.py').read()); print('syntax ok')"`
Expected: `syntax ok`

(ROS 미소스 환경에서는 런타임 import가 안 될 수 있으니 문법만 확인. 빌드 검증은 Task 4.)

---

### Task 4: setup.py 확인 + colcon 빌드 검증

**Files:**
- Modify: `setup.py` (필요 시 — 이미 `camera_node = turtle_project.camera_node:main` 등록되어 있으면 그대로)

**Interfaces:**
- Consumes: Task 1-3 파일
- Produces: 빌드 가능한 패키지

- [ ] **Step 1: setup.py entry_points 확인**

`setup.py`의 `console_scripts`에 아래가 있는지 확인 (이미 있음):
```python
'camera_node = turtle_project.camera_node:main'
```
없으면 추가. depth_math/nav_controller는 라이브러리 모듈이라 entry point 불필요.

- [ ] **Step 2: 순수 함수 self-check 재실행 (회귀 확인)**

Run:
```bash
python3 turtle_project/depth_math.py && python3 turtle_project/nav_controller.py
```
Expected: `depth_math self-check ok` / `nav_controller self-check ok` 둘 다 출력

- [ ] **Step 3: colcon 빌드**

Run:
```bash
cd /home/rokey/turtlebot4_ws && colcon build --packages-select turtle_project
```
Expected: `Finished <<< turtle_project` (에러 없음)

- [ ] **Step 4: 노드 기동 스모크 테스트 (모델 없이)**

Run:
```bash
cd /home/rokey/turtlebot4_ws && source install/setup.bash && \
  timeout 5 ros2 run turtle_project camera_node
```
Expected: `model_path 없음 — 탐지 스킵` 경고 + `대기 — target='opening'...` 로그가 뜨고 5초 뒤 timeout 종료 (크래시 없음)

---

## Self-Review

**Spec coverage:**
- 입력 3토픽 구독 → Task 3 ✓
- ApproximateTimeSynchronizer → Task 3 ✓
- depth_math 5함수(신규 depth_spread 포함) → Task 1 ✓
- nav_controller(Navigator 도착 콜백, approach_point, make_pose) → Task 2 ✓
- 상태머신 3단계 → Task 3 ✓
- 파라미터 6개 → Task 3 ✓
- 에러 처리(모델 없음/depth 무효/TF 실패/Nav2 없음) → Task 2, 3 ✓
- self_check(depth_spread, approach_point, make_pose) → Task 1, 2 ✓
- 도착 판정 Nav2 SUCCEEDED → Task 2 (`_done` status==4) ✓

**Placeholder scan:** 없음 — 모든 코드 스텝에 실제 코드 포함.

**Type consistency:** `depth_spread(depth_mm, x1, y1, x2, y2, min_valid)` 시그니처가 Task 1 정의와 Task 3 호출 일치. `Navigator(node, on_arrived)` Task 2 정의와 Task 3 생성 일치. `approach_point`/`make_pose` 반환 형태 일치.
