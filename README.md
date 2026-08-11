# Under-Guard

ROS 2 Humble 기반 2로봇(robot4 + robot6) AMR 방역 감시 시스템.
순찰 중 침입구(opening)와 쥐(rat)를 감지해 대응한다.

## 패키지

- **`src/turtle_project`** — 노드 전체 (감지·주행·중앙조율·DB·쥐몰이·웹캠)
- **`src/turtle_interfaces`** — 커스텀 서비스 (`QueryHole.srv`)

## 노드 구성

| 노드 | 역할 | 실행 위치 |
|------|------|-----------|
| `detector_node` | opening·rat 감지 → 검증/추적 target_pose 발행 (oakd rgb+depth sync 포함) | 각 로봇 PC |
| `trap_check_node` | trap 설치 상태 점검 | 각 로봇 PC |
| `robot_agent` | 순찰 주행(Nav2)·dock·배터리 감시·유일한 goal 주행자 | 각 로봇 PC |
| `central_node` | 순찰 교대·쥐 역할배정(A 추적/B 몰이) | 중앙 PC |
| `db_node` | 구멍·사건·미션 기록 DB (`/db/query_hole`, UI 조회 `/db/query`) | 중앙 PC |
| `rat_herding_node` | 쥐몰이 알고리즘 → 로봇B goal 발행 | 중앙 PC |
| `webcam_node` | 고정 웹캠 쥐 감시 (homography) | 중앙 PC |

로봇 A/B는 고정이 아니라 central이 쥐 감지 시점에 동적 배정한다
(순찰 중이던 로봇 = A, 나머지 = B).

## 빌드

```bash
cd ~/turtlebot4_ws
colcon build --packages-select turtle_interfaces turtle_project
source install/setup.bash
```

## 실행

**로봇 PC** (namespace 예: robot6):
```bash
ros2 run turtle_project detector_node   --ros-args -r __ns:=/robot6 -p model_path:=<모델.engine>
ros2 run turtle_project trap_check_node --ros-args -r __ns:=/robot6
ros2 run turtle_project robot_agent     --ros-args -p namespace:=robot6 -p waypoints:=<wp.yaml>
```

**중앙 PC**:
```bash
ros2 run turtle_project central_node
ros2 run turtle_project db_node
ros2 run turtle_project rat_herding_node
ros2 run turtle_project webcam_node
```

> ⚠️ `camera`·`detector`·`trap_check`는 `-r __ns:=/robotN`,
> `robot_agent`는 `-p namespace:=robotN` — 방식이 다르다 (섞으면 이중 prefix로 깨짐).
