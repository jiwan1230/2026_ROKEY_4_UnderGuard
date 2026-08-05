# ROS 2 인터페이스 초안

현재 `rokey_4_mini/main`에서 확인된 인터페이스를 기준으로 작성했다.

## 기존 인터페이스

| Namespace 상대 이름 | 타입 | System Monitor 사용 |
|---|---|---|
| `webcam/detections` | `vision_msgs/Detection3DArray` | 외부 웹캠 탐지 클래스·신뢰도·거리 |
| `oakd/detections` | `vision_msgs/Detection3DArray` | OAK-D 추적 클래스·신뢰도·거리 |
| `dummy_cloud` | `sensor_msgs/PointCloud2` | 가상 장애물 발행 여부 및 후속 상태 표시 |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 현재 goal 결과·feedback 연동 예정 |
| `tf`, `tf_static` | TF | map 기준 로봇 위치 연동 예정 |

`goal_manager_node.py`의 기존 동작은 웹캠 좌표로 접근하고, OAK-D가 대상을 잡으면 OAK-D 추적으로 전환하며, 1.5초 유실 시 goal을 취소하는 구조다.

### 탐지 좌표계 처리

`Detection3D.bbox.center`는 `Detection3DArray.header.frame_id` 좌표계의 값이다.
System Monitor는 `frame_id`가 설정된 `ROS_MAP_FRAME`과 같은 경우에만 중심의
x/y를 `map_x/map_y`로 저장한다. 카메라 등 센서 좌표계로 수신한 탐지는 거리와
분류 결과만 저장하며, 지도 좌표는 TF 변환이 연결될 때까지 비워 둔다.

## 이번 스타터에서 추가 구독한 후보

| 토픽 | 목적 | 확인 상태 |
|---|---|---|
| `/<ns>/odom` | 로봇 위치·속도 | 현장 `ros2 topic list`로 확인 필요 |
| `/<ns>/battery_state` | 배터리 | 현장 `ros2 topic list`로 확인 필요 |

## 실제 제어 명령

현재는 안전상 ROS 제어 명령을 연결하지 않았다. PM·로봇 담당과 아래를 먼저 확정한다.

- 추적 시작/중단: Topic, Service, Action 중 선택
- 탐색 시작/중단
- 복귀
- 쥐덫 설치
- 비상정지

합의 전 UI의 ROS 모드 명령 API는 `accepted: false`를 반환한다.

## 저장소 업데이트 후 맞출 설정

변경 가능성이 큰 값은 `system_monitor/config.py`의 `RosInterfaceConfig`로 모았다.
아래 환경변수를 실제 `ros2 topic list -t`와 맵 YAML에 맞추면 구독 처리와 화면
코드는 수정하지 않아도 된다.

| 환경변수 | 기본값 | 의미 |
|---|---|---|
| `ROBOT_NAMESPACES` | `robot4,robot5` | 모니터링할 로봇 namespace |
| `ROS_WEBCAM_DETECTIONS_TOPIC` | `webcam/detections` | 웹캠 탐지 토픽 |
| `ROS_OAKD_DETECTIONS_TOPIC` | `oakd/detections` | OAK-D 탐지 토픽 |
| `ROS_ODOMETRY_TOPIC` | `odom` | 위치·속도 토픽 |
| `ROS_BATTERY_TOPIC` | `battery_state` | 배터리 토픽 |
| `ROS_MAP_FRAME` | `map` | 지도와 마커가 공유할 TF frame |
| `MAP_YAML_PATH` | 프로젝트 환경의 `my_map.yaml` | 정적 맵 YAML 경로 |

토픽 값은 세 형태를 지원한다.

- 상대 이름 `odom`: 로봇마다 `/<namespace>/odom`으로 변환
- 절대 이름 `/fleet/status`: namespace를 붙이지 않고 그대로 사용
- 템플릿 `/{namespace}/odometry/filtered`: 로봇별 namespace를 치환

내일 확인할 순서는 `ros2 topic list -t` → 각 메시지 `header.frame_id` → 맵 YAML의
`resolution`, `origin`, `image` → 필요 시 TF 트리 순서다. 메시지 타입 자체가 바뀐
경우에만 `ros_bridge.py`의 구독 타입과 변환 함수를 수정한다.
