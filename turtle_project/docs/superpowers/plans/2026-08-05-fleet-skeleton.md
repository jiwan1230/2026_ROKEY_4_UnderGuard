# Under-Guard 노드 구조 (스켈레톤) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 2대 로봇 협업 방역 시스템의 전체 노드 구조를 뼈대로 세운다 — 토픽/서비스 배관은 실제로 연결, 동작 로직은 `# TODO(팀원)` placeholder.

**Architecture:** camera_node를 순수 이미지 파이프(sync 재발행)로 재작성하고, 기존 opening 검증 로직은 detector_node로 이동한다. fleet 통신은 String 토픽 + fleet_msg 헬퍼, DB 조회만 커스텀 srv(turtle_interfaces). 로봇 PC(camera/detector/trap_check/robot_agent) + 중앙 PC(central/db/rat_herding/webcam) 분산.

**Tech Stack:** ROS 2 Humble (rclpy), ament_python + ament_cmake(interfaces), std_msgs/String, geometry_msgs/PoseStamped, message_filters, ultralytics YOLO, cv_bridge.

## Global Constraints

- 기존 파일 유지: `depth_math.py`, `nav_controller.py`, `opening_test_node.py` — 건드리지 않음.
- 순수 함수(fleet_msg, central next_command)는 `if __name__ == '__main__'`에서 `_self_check()` assert 검증. ROS/YOLO/하드웨어는 self-check 제외.
- 이 워크스페이스는 git 저장소가 아니다 — 커밋 스텝 생략.
- fleet 통신 포맷: `/fleet/status`=`"<robot>:<state>:<battery%>"`, `/fleet/command`=`"<robot>:<command>"`, `/fleet/event`=`"<event>:<x>:<y>"`.
- 신규 노드 로직은 placeholder(`# TODO(팀원)` + 로그). detector의 opening 검증만 기존 동작 코드 이동.
- 빌드는 `colcon build`, 실행 시 discovery server 환경(`ROS_SUPER_CLIENT=True`, `ROS_DOMAIN_ID=4`) 필요 — tty 터미널에서.
- 노드 이름/파라미터는 spec `2026-08-05-fleet-skeleton-design.md` 따름.

---

### Task 1: turtle_interfaces 패키지 + QueryHole.srv

**Files:**
- Create: `../turtle_interfaces/package.xml`
- Create: `../turtle_interfaces/CMakeLists.txt`
- Create: `../turtle_interfaces/srv/QueryHole.srv`

**Interfaces:**
- Consumes: (없음)
- Produces: `turtle_interfaces/srv/QueryHole` — req `float64 x, float64 y` / resp `bool exists, bool trap_installed`

- [ ] **Step 1: package.xml 작성**

`../turtle_interfaces/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>turtle_interfaces</name>
  <version>0.0.0</version>
  <description>Under-Guard 커스텀 인터페이스 (srv/msg)</description>
  <maintainer email="tmdwodl12@gmail.com">rokey</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: CMakeLists.txt 작성**

`../turtle_interfaces/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(turtle_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/QueryHole.srv"
)

ament_package()
```

- [ ] **Step 3: QueryHole.srv 작성**

`../turtle_interfaces/srv/QueryHole.srv`:
```
float64 x
float64 y
---
bool exists
bool trap_installed
```

- [ ] **Step 4: 빌드 확인**

Run: `cd /home/rokey/turtlebot4_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select turtle_interfaces`
Expected: `Finished <<< turtle_interfaces`

- [ ] **Step 5: 타입 생성 확인**

Run: `cd /home/rokey/turtlebot4_ws && source install/setup.bash && ros2 interface show turtle_interfaces/srv/QueryHole`
Expected: `float64 x` ... `bool exists` 출력

---

### Task 2: fleet_msg.py — String 포맷 헬퍼

**Files:**
- Create: `turtle_project/fleet_msg.py`

**Interfaces:**
- Consumes: (없음)
- Produces:
  - `status(robot, state, battery) -> str` / `parse_status(s) -> (robot, state, battery:int)`
  - `command(robot, cmd) -> str` / `parse_command(s) -> (robot, cmd)`
  - `event(name, x, y) -> str` / `parse_event(s) -> (name, x:float, y:float)`

- [ ] **Step 1: 파일 작성 (헬퍼 + self-check)**

`turtle_project/fleet_msg.py`:
```python
"""fleet 통신 String 포맷 파싱/조립. 모든 노드가 공용 — 포맷을 한 곳에 둔다.

  status  : "robot4:PATROLLING:85"
  command : "robot6:UNDOCK"
  event   : "rat_detected:1.20:3.40"

문자열 자르기를 노드마다 재구현하면 오타로 깨지므로 여기로 통일. ROS 무관.
"""


def status(robot, state, battery):
    return f'{robot}:{state}:{int(battery)}'


def parse_status(s):
    robot, state, battery = s.split(':')
    return robot, state, int(battery)


def command(robot, cmd):
    return f'{robot}:{cmd}'


def parse_command(s):
    robot, cmd = s.split(':')
    return robot, cmd


def event(name, x, y):
    return f'{name}:{x:.2f}:{y:.2f}'


def parse_event(s):
    name, x, y = s.split(':')
    return name, float(x), float(y)


def _self_check():
    assert status('robot4', 'PATROLLING', 85) == 'robot4:PATROLLING:85'
    assert parse_status('robot4:PATROLLING:85') == ('robot4', 'PATROLLING', 85)
    # round-trip
    assert parse_status(status('robot6', 'DOCKED', 20)) == ('robot6', 'DOCKED', 20)

    assert command('robot6', 'UNDOCK') == 'robot6:UNDOCK'
    assert parse_command('robot6:UNDOCK') == ('robot6', 'UNDOCK')
    assert parse_command(command('robot4', 'PATROL')) == ('robot4', 'PATROL')

    assert event('rat_detected', 1.2, 3.4) == 'rat_detected:1.20:3.40'
    name, x, y = parse_event('rat_detected:1.20:3.40')
    assert name == 'rat_detected' and abs(x - 1.2) < 1e-9 and abs(y - 3.4) < 1e-9
    # round-trip
    n2, x2, y2 = parse_event(event('opening_confirmed', -0.5, 2.0))
    assert n2 == 'opening_confirmed' and abs(x2 + 0.5) < 1e-9 and abs(y2 - 2.0) < 1e-9
    print('fleet_msg self-check ok')


if __name__ == '__main__':
    _self_check()
```

- [ ] **Step 2: self-check 실행**

Run: `python3 turtle_project/fleet_msg.py`
Expected: `fleet_msg self-check ok`

---

### Task 3: camera_node.py 재작성 — sync 이미지 파이프

**Files:**
- Modify: `turtle_project/camera_node.py` (기존 검증 로직은 Task 4에서 detector로 이동하므로 여기선 전체 교체)

**Interfaces:**
- Consumes: 원본 카메라 토픽 (`{ns}/oakd/rgb/image_raw/compressed` 등)
- Produces: `{ns}/synced/rgb`, `{ns}/synced/depth`, `{ns}/synced/camera_info` (CompressedImage/CameraInfo, 같은 stamp)

- [ ] **Step 1: 파일 전체 교체**

`turtle_project/camera_node.py`:
```python
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
```

- [ ] **Step 2: 문법 확인**

Run: `python3 -c "import ast; ast.parse(open('turtle_project/camera_node.py').read()); print('ok')"`
Expected: `ok`

---

### Task 4: detector_node.py — opening 검증(이동) + rat 감지 뼈대

**Files:**
- Create: `turtle_project/detector_node.py`

**Interfaces:**
- Consumes: `{ns}/synced/rgb`, `{ns}/synced/depth`, `{ns}/synced/camera_info`; `fleet_msg.event`; `depth_math.*`; `nav_controller.*`; `turtle_interfaces/srv/QueryHole`
- Produces: `/fleet/event` 발행, `{ns}/target_pose` 발행, `/db/query_hole` 호출

- [ ] **Step 1: 파일 작성 (기존 검증 로직 이동 + rat/DB 뼈대)**

`turtle_project/detector_node.py`:
```python
"""로봇 카메라 감지 — YOLO로 opening·rat 감지.

opening: DB에 좌표 조회 → 없으면 접근·depth_spread 검증(기존 camera_node 로직
이동) → 진짜 구멍이면 trap 설치단계(로그)+DB기록. 있으면 trap 점검으로.
rat: target_pose로 추적 goal + /fleet/event 발행.

synced/rgb·synced/depth(camera_node 발행)를 구독한다. namespace로 실행.
opening 검증만 실제 동작, rat·DB 분기는 TODO(팀원).
"""
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
from turtle_project.nav_controller import (Navigator, approach_point,
                                           make_pose)

FRAME = 'map'


class DetectorNode(Node):

    def __init__(self):
        super().__init__('detector_node')
        self.model_path = self.declare_parameter('model_path', '').value
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
        self.target = None
        self.verify_count = 0

        self.event_pub = self.create_publisher(String, '/fleet/event', 10)
        # 추적 goal은 robot_agent가 Nav2로 쓰도록 PoseStamped (rat TODO에서 발행).
        self.pose_pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.db = self.create_client(QueryHole, '/db/query_hole')
        self.nav = Navigator(self, on_arrived=self._arrived)

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
        if self.model is None or self.state == 'APPROACHING':
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
        if g is None:
            self.target = xy
            self.state = 'VERIFYING'
            self.verify_count = 0
            return
        gx, gy, yaw = rx + g[0], ry + g[1], g[2]
        if self.nav.go(make_pose(FRAME, gx, gy, yaw)):
            self.target = xy
            self.state = 'APPROACHING'
            self.get_logger().info(f'opening ({xy[0]:.2f}, {xy[1]:.2f}) 접근')

    def _arrived(self):
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
```

- [ ] **Step 2: 문법 확인**

Run: `python3 -c "import ast; ast.parse(open('turtle_project/detector_node.py').read()); print('ok')"`
Expected: `ok`

---

### Task 5: central_node.py — 교대 상태머신 뼈대 + next_command

**Files:**
- Create: `turtle_project/central_node.py`

**Interfaces:**
- Consumes: `/fleet/status`, `/fleet/event`, `fleet_msg.*`
- Produces: `/fleet/command` 발행; `next_command(prev_state, new_state)` 순수함수

- [ ] **Step 1: 파일 작성 (뼈대 + next_command 순수함수 + self-check)**

`turtle_project/central_node.py`:
```python
"""중앙 조율 — 순찰 교대, 쥐대응 역할배정.

순찰 교대: 로봇A 배터리<임계 → RETURNING → DOCKED 수신하면 로봇B UNDOCK →
로봇B PATROL. 쥐대응: rat_detected 수신 시 PATROLLING 로봇=A, 나머지=B.

전이 판정은 next_command() 순수함수. 나머지 조율은 TODO(팀원).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from turtle_project import fleet_msg

# 로봇 A가 이 상태로 바뀌면 로봇 B에게 보낼 명령 (순찰 교대 핵심 전이).
_TRANSITIONS = {
    'DOCKED': 'UNDOCK',     # A가 도킹 완료 -> B undock (교대 시작)
}


def next_command(new_state):
    """로봇 A의 새 상태 -> 로봇 B에게 보낼 명령. 없으면 None.

    DOCKED 완료 후에만 B를 깨운다 (감시 공백 최소화하되 한 번에 한 대).
    """
    return _TRANSITIONS.get(new_state)


class CentralNode(Node):

    def __init__(self):
        super().__init__('central_node')
        self.robots = {}        # robot -> (state, battery)
        self.patroller = None   # 현재 순찰 로봇 (역할 A)

        self.cmd_pub = self.create_publisher(String, '/fleet/command', 10)
        self.create_subscription(String, '/fleet/status', self.status_cb, 10)
        self.create_subscription(String, '/fleet/event', self.event_cb, 10)
        self.get_logger().info('중앙 노드 시작 — status/event 대기')

    def status_cb(self, msg):
        robot, state, battery = fleet_msg.parse_status(msg.data)
        prev = self.robots.get(robot)
        self.robots[robot] = (state, battery)
        if state == 'PATROLLING':
            self.patroller = robot
        # 교대 핵심 전이: A가 DOCKED되면 B를 undock.
        cmd = next_command(state)
        if cmd and (prev is None or prev[0] != state):
            other = self._other(robot)
            if other:
                self.send(other, cmd)
        # TODO(팀원): PATROL 이어받기, 재도킹, 상태 정합성 등 나머지 시퀀스.

    def event_cb(self, msg):
        name, x, y = fleet_msg.parse_event(msg.data)
        self.get_logger().info(f'이벤트 {name} at ({x:.2f}, {y:.2f})')
        # TODO(팀원): rat_detected -> 쥐대응 모드, 역할 A/B 배정, TRACK/HERD 명령.

    def send(self, robot, cmd):
        self.cmd_pub.publish(String(data=fleet_msg.command(robot, cmd)))
        self.get_logger().info(f'명령 → {robot}:{cmd}')

    def _other(self, robot):
        """robots 중 이 로봇이 아닌 다른 하나. 없으면 None."""
        others = [r for r in self.robots if r != robot]
        return others[0] if others else None


def main():
    rclpy.init()
    node = CentralNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


def _self_check():
    assert next_command('DOCKED') == 'UNDOCK'   # 교대 핵심 전이
    assert next_command('PATROLLING') is None    # 순찰 중엔 교대 명령 없음
    assert next_command('RETURNING') is None
    print('central_node self-check ok')


if __name__ == '__main__':
    import sys
    _self_check() if '--check' in sys.argv else main()
```

- [ ] **Step 2: self-check 실행**

Run: `python3 turtle_project/central_node.py --check`
Expected: `central_node self-check ok`

---

### Task 6: robot_agent.py — 주행/도킹/배터리 뼈대

**Files:**
- Create: `turtle_project/robot_agent.py`

**Interfaces:**
- Consumes: `/fleet/command`(자기 것 필터), `{ns}/battery_state`, `{ns}/target_pose`, `fleet_msg.*`
- Produces: `/fleet/status` 발행

- [ ] **Step 1: 파일 작성**

`turtle_project/robot_agent.py`:
```python
"""로봇 1대 제어 — 주행(Nav2)·dock/undock·배터리 감시·상태보고.

namespace 파라미터로 로봇마다 실행 (robot4/robot6). /fleet/command에서 자기
명령만 필터해 실행하고, /fleet/status로 상태를 보고한다.

배터리 임계 감지만 실제 동작, 주행·도킹 실행은 TODO(팀원).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from turtle_project import fleet_msg


class RobotAgent(Node):

    def __init__(self):
        super().__init__('robot_agent')
        self.ns = self.declare_parameter('namespace', 'robot4').value
        self.threshold = self.declare_parameter('battery_threshold', 20).value
        period = self.declare_parameter('battery_check_period', 10.0).value

        self.state = 'IDLE'
        self.battery = 100

        self.status_pub = self.create_publisher(String, '/fleet/status', 10)
        self.create_subscription(String, '/fleet/command', self.command_cb, 10)
        self.create_subscription(BatteryState, f'{self.ns}/battery_state',
                                 self.battery_cb, 10)
        self.create_timer(period, self.report)
        self.get_logger().info(f'{self.ns} agent 시작 — 임계 {self.threshold}%')

    def command_cb(self, msg):
        robot, cmd = fleet_msg.parse_command(msg.data)
        if robot != self.ns:
            return                          # 내 명령 아님
        self.get_logger().info(f'명령 수신: {cmd}')
        # TODO(팀원): UNDOCK/DOCK/PATROL/TRACK/HERD 실제 실행 (액션/Nav2).
        # 뼈대는 명령을 상태로만 반영.
        self.state = {'UNDOCK': 'PATROLLING', 'DOCK': 'DOCKED',
                      'PATROL': 'PATROLLING', 'TRACK': 'TRACKING',
                      'HERD': 'HERDING', 'STOP': 'IDLE'}.get(cmd, self.state)

    def battery_cb(self, msg):
        self.battery = int(msg.percentage * 100) if msg.percentage <= 1.0 \
            else int(msg.percentage)
        if self.battery < self.threshold and self.state == 'PATROLLING':
            self.state = 'RETURNING'
            self.get_logger().info(f'배터리 {self.battery}% < 임계 — 복귀')
            # TODO(팀원): dock 스테이션 이동 + Dock 액션. 완료되면 state=DOCKED.

    def report(self):
        self.status_pub.publish(String(data=fleet_msg.status(
            self.ns, self.state, self.battery)))


def main():
    rclpy.init()
    node = RobotAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: 문법 확인**

Run: `python3 -c "import ast; ast.parse(open('turtle_project/robot_agent.py').read()); print('ok')"`
Expected: `ok`

---

### Task 7: db_node / rat_herding_node / trap_check_node / webcam_node 뼈대

**Files:**
- Create: `turtle_project/db_node.py`
- Create: `turtle_project/rat_herding_node.py`
- Create: `turtle_project/trap_check_node.py`
- Create: `turtle_project/webcam_node.py`

**Interfaces:**
- Consumes: db=`QueryHole` 서버; herding=`/fleet/event`; trap=`synced/*`; webcam=`cv2`
- Produces: db=응답, herding=`/robot6/target_pose`, trap/webcam=`/fleet/event`

- [ ] **Step 1: db_node.py 작성**

`turtle_project/db_node.py`:
```python
"""구멍/trap 좌표 DB — QueryHole 서비스 서버.

좌표를 받아 전에 감지된 구멍이 있는지, trap이 설치됐는지 응답. 저장/조회
로직은 TODO(팀원). 뼈대는 빈 저장소 + 항상 exists=False 응답.
"""
import rclpy
from rclpy.node import Node

from turtle_interfaces.srv import QueryHole


class DbNode(Node):

    def __init__(self):
        super().__init__('db_node')
        self.holes = []     # TODO(팀원): [(x, y, trap_installed)] 기록/조회
        self.srv = self.create_service(QueryHole, '/db/query_hole', self.query)
        self.get_logger().info('DB 노드 시작 — /db/query_hole 대기')

    def query(self, req, resp):
        # TODO(팀원): self.holes에서 (req.x, req.y) 근처 구멍 찾기.
        self.get_logger().info(f'조회 ({req.x:.2f}, {req.y:.2f}) — 뼈대: 없음')
        resp.exists = False
        resp.trap_installed = False
        return resp


def main():
    rclpy.init()
    node = DbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: rat_herding_node.py 작성**

`turtle_project/rat_herding_node.py`:
```python
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
```

- [ ] **Step 3: trap_check_node.py 작성**

`turtle_project/trap_check_node.py`:
```python
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
```

- [ ] **Step 4: webcam_node.py 작성**

`turtle_project/webcam_node.py`:
```python
"""고정 웹캠 쥐 감시 — 노트북 웹캠에서 YOLO로 rat 감지.

감지하면 homography로 map 좌표를 구해 /fleet/event(rat_detected:x:y) 발행 →
중앙이 쥐대응 진입. YOLO·homography는 TODO(팀원). 뼈대는 프레임 읽기·발행 배관.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WebcamNode(Node):

    def __init__(self):
        super().__init__('webcam_node')
        self.camera_index = self.declare_parameter('camera_index', 0).value
        self.model_path = self.declare_parameter('model_path', '').value
        self.homography_file = self.declare_parameter('homography_file', '').value

        self.event_pub = self.create_publisher(String, '/fleet/event', 10)
        # TODO(팀원): cv2.VideoCapture(camera_index) 열고, 타이머로 프레임 읽어
        # YOLO 감지 + homography 변환 → rat_detected event 발행.
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'웹캠 노드 시작 — index {self.camera_index}')

    def tick(self):
        # TODO(팀원): 프레임 읽기 → YOLO → homography → event.
        pass


def main():
    rclpy.init()
    node = WebcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 5: 네 파일 문법 확인**

Run:
```bash
for f in db_node rat_herding_node trap_check_node webcam_node; do
  python3 -c "import ast; ast.parse(open('turtle_project/$f.py').read()); print('$f ok')"
done
```
Expected: `db_node ok` / `rat_herding_node ok` / `trap_check_node ok` / `webcam_node ok`

---

### Task 8: setup.py entry_points + package.xml 의존 + 빌드 검증

**Files:**
- Modify: `setup.py`
- Modify: `package.xml`

**Interfaces:**
- Consumes: Task 1-7 전부
- Produces: 빌드·실행 가능한 두 패키지

- [ ] **Step 1: package.xml에 turtle_interfaces 의존 추가**

`package.xml`의 `<license>` 줄 다음에 추가:
```xml
  <depend>turtle_interfaces</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
```

- [ ] **Step 2: setup.py entry_points 갱신**

`setup.py`의 `console_scripts`를 아래로 교체:
```python
        'console_scripts': [
            'camera_node = turtle_project.camera_node:main',
            'opening_test_node = turtle_project.opening_test_node:main',
            'detector_node = turtle_project.detector_node:main',
            'trap_check_node = turtle_project.trap_check_node:main',
            'robot_agent = turtle_project.robot_agent:main',
            'central_node = turtle_project.central_node:main',
            'db_node = turtle_project.db_node:main',
            'rat_herding_node = turtle_project.rat_herding_node:main',
            'webcam_node = turtle_project.webcam_node:main',
        ],
```

- [ ] **Step 3: 순수함수 self-check 재실행 (회귀)**

Run:
```bash
python3 turtle_project/fleet_msg.py && python3 turtle_project/central_node.py --check
```
Expected: `fleet_msg self-check ok` / `central_node self-check ok`

- [ ] **Step 4: 두 패키지 빌드**

Run:
```bash
cd /home/rokey/turtlebot4_ws && source /opt/ros/humble/setup.bash && \
  colcon build --packages-select turtle_interfaces turtle_project
```
Expected: `Finished <<< turtle_interfaces` / `Finished <<< turtle_project` (에러 없음)

- [ ] **Step 5: 노드 스모크 테스트 (뼈대가 뜨는지)**

Run:
```bash
cd /home/rokey/turtlebot4_ws && source install/setup.bash && \
  for n in central_node db_node rat_herding_node; do
    timeout 4 ros2 run turtle_project $n > /tmp/$n.log 2>&1; \
    echo "$n: $(head -1 /tmp/$n.log)"; \
  done
```
Expected: 각 노드의 시작 로그가 출력 (크래시 없음). 예 `central_node: [INFO] ... 중앙 노드 시작`

---

## Self-Review

**Spec coverage:**
- turtle_interfaces + QueryHole.srv → Task 1 ✓
- fleet_msg 헬퍼 → Task 2 ✓
- camera_node sync 파이프 재작성 → Task 3 ✓
- detector_node (opening 검증 이동 + rat/DB 뼈대) → Task 4 ✓
- central_node (교대 시퀀스 + next_command) → Task 5 ✓
- robot_agent (배터리 감지 + 명령 뼈대) → Task 6 ✓
- db/rat_herding/trap_check/webcam 뼈대 → Task 7 ✓
- setup.py/package.xml/빌드 → Task 8 ✓
- 분산 실행: namespace 파라미터(robot_agent, camera 상대토픽) → Task 3, 6 ✓
- 통신 3토픽 + target_pose + QueryHole 서비스 → Task 2,4,5,6,7 ✓

**Placeholder scan:** 코드 내 `# TODO(팀원)`은 의도된 뼈대 표시(placeholder 아님, 실제 배관은 연결됨). 계획 자체엔 미완성 스텝 없음.

**Type consistency:**
- `fleet_msg.status/command/event` 시그니처가 Task 2 정의와 central(Task5)·robot_agent(Task6)·detector(Task4) 호출 일치.
- `next_command(new_state)` Task 5 정의·사용 일치.
- `QueryHole.Request().x/y`, `.exists/.trap_installed` Task 1 srv와 detector(Task4)·db(Task7) 일치.
- target_pose는 `PoseStamped`로 통일 (spec 일치). detector Task 4에서 import·publisher 반영 완료. `PointStamped`는 deproject 좌표 계산용으로 계속 사용(용도 다름).
