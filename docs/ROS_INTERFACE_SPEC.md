# ROS 2 인터페이스 명세

`main` 브랜치의 `turtle_project`와 `turtle_interfaces`를 기준으로 System Monitor가
읽는 계약을 정리한다. Fleet String 토픽이 1순위 경계이며 로봇별 센서 토픽은
위치·상세 탐지 정보를 보충하는 선택 입력이다.

## main Fleet 계약

| 토픽 | 타입 | 포맷 | 관제 처리 |
|---|---|---|---|
| `/fleet/status` | `std_msgs/String` | `robot:state:battery` | 연결·상태·배터리 갱신 |
| `/fleet/event` | `std_msgs/String` | `event:x:y` | 현재 화면의 탐지·트랩 위치 갱신 |

main 상태는 다음처럼 관제 상태로 변환한다.

| main 상태 | 관제 상태 |
|---|---|
| `IDLE` | `IDLE` |
| `PATROLLING` | `SEARCHING` |
| `RETURNING` | `RETURNING` |
| `DOCKED` | `COMPLETED` |
| `TRACKING` | `TRACKING` |
| `HERDING` | `NAVIGATING` |

main 사건은 `rat_detected` → `LIVE_RODENT`, `opening_confirmed` →
`ENTRY_POINT`, `trap_ok` → 설치 트랩 마커로 변환한다. x/y는 main detector가 TF로
변환한 map 좌표로 취급한다.

### 현재 계약 제약

`/fleet/event` 포맷에는 `robot_id`, 신뢰도, 거리, 이미지 경로가 없다. 따라서
System Monitor는 가장 최근에 활동 상태를 보낸 로봇을 사건 소유자로 사용하고,
그 정보도 없으면 설정된 첫 로봇을 사용한다. 정확한 다중 로봇 사건 귀속이 필요하면
main 계약을 `robot:event:x:y` 또는 커스텀 메시지로 확장해야 한다.

System Monitor는 `/fleet/command`를 발행하지 않는다. 로봇 제어는
`central_node`와 로봇 노드의 책임이며 관제 화면은 `/fleet/status`로 확인된 결과만
표시한다.

## 선택적 로봇별 입력

| 토픽 | 타입 | 목적 |
|---|---|---|
| `/<ns>/odom` | `nav_msgs/Odometry` | 위치·방향·속도 |
| `/<ns>/battery_state` | `sensor_msgs/BatteryState` | Fleet status 보조 배터리 |
| `/<ns>/webcam/detections` | `vision_msgs/Detection3DArray` | 상세 웹캠 탐지 |
| `/<ns>/oakd/detections` | `vision_msgs/Detection3DArray` | 상세 OAK-D 탐지·대상 유실 |

Detection3D 중심 좌표는 header frame이 `ROS_MAP_FRAME`과 같을 때만 지도 좌표로
사용한다. 다른 센서 frame은 TF 연결 전까지 분류·거리만 표시한다. odom 위치 역시
source frame을 보존하며 map frame이 아니면 지도에 억지로 표시하지 않는다.

## turtle_interfaces

main의 `/db/query_hole` 서비스는 `turtle_interfaces/srv/QueryHole`을 사용한다.

```text
float64 x
float64 y
---
bool exists
bool trap_installed
```

현재 이 서비스는 `db_node`와 `detector_node` 사이의 로봇 동작용 계약이다.
System Monitor는 이 DB를 만들거나 직접 공유하지 않는다. 로봇 DB 조회가 필요해지면
파일 접근이 아니라 별도의 Service/API 계약과 데이터 소유권을 먼저 확정한다.

## 환경변수

| 환경변수 | 기본값 |
|---|---|
| `ROBOT_NAMESPACES` | `robot4,robot6` |
| `ROS_FLEET_STATUS_TOPIC` | `/fleet/status` |
| `ROS_FLEET_EVENT_TOPIC` | `/fleet/event` |
| `ROS_WEBCAM_DETECTIONS_TOPIC` | `webcam/detections` |
| `ROS_OAKD_DETECTIONS_TOPIC` | `oakd/detections` |
| `ROS_ODOMETRY_TOPIC` | `odom` |
| `ROS_BATTERY_TOPIC` | `battery_state` |
| `ROS_MAP_FRAME` | `map` |
| `OFFLINE_TIMEOUT_SEC` | `15.0` |

토픽 값은 상대 이름, `/fleet/status` 같은 절대 이름,
`/{namespace}/odometry/filtered` 같은 namespace 템플릿을 지원한다.
Offline 제한은 main `robot_agent`의 기본 상태 보고 주기 10초보다 길게 설정한다.
