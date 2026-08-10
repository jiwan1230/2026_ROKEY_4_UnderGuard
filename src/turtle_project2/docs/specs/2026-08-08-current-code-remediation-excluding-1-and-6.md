# 현재 코드 전수검토 문제 및 해결 설계 — 1번·6번 제외

> 작성일: 2026-08-08  
> 기준: 현재 작업 트리에 존재하는 소스·설정·리소스  
> 상태: 해결 설계 문서이며, 이 문서 작성으로 소스 코드는 변경하지 않는다.

## 0. 범위

이번 전수검토에서 보고한 항목 중 아래 두 항목은 사용자 요청에 따라 이 문서의
해결 범위에서 제외한다.

1. **제외 1번** — `nav2_activate.sh`의 lifecycle 준비 판정과 이중 활성화 소유자 문제
2. **제외 6번** — `RatTracker`의 포획 판정이 실제 포획이 아니라 정지 판정인 문제

위 두 항목을 제외한 나머지 문제와 파생 장애를 다룬다. 특히 단순히 예외를 막는
수준이 아니라 다음 계약을 만족하는 것을 목표로 한다.

- 상태 보고는 실제 센서 또는 action 결과로 확인된 사실만 나타낸다.
- 한 로봇의 물리 이동은 항상 `robot_agent` 한 곳에서만 실행한다.
- 모든 비동기 명령·결과에는 작업 식별자를 붙여 이전 작업의 늦은 응답을 거부한다.
- 모든 action과 작업 단계에는 유한 timeout과 실제 cancel 경로가 있다.
- DB 저장 성공은 관찰용 토픽 발행이 아니라 commit ACK로 확인한다.
- 실행만 되고 기능이 비어 있는 노드는 capability에서 준비되지 않은 것으로 보고한다.
- 프로세스 watchdog은 ROS clock 정지와 무관한 `time.monotonic()`을 사용한다.

기존 상세 설계인
[`2026-08-08-camera-merge-and-review-fixes-design.md`](2026-08-08-camera-merge-and-review-fixes-design.md)와
일부 내용이 겹친다. 이 문서는 **현재 코드 전수검토 결과와 이번 제외 범위**를
기준으로 한 실행 명세이며, 구현 시 두 문서가 충돌하면 이 문서의 범위와 완료
조건을 우선한다.

## 1. 우선순위 요약

| 우선순위 | 문제 | 실패 영향 | 해결 핵심 |
|---|---|---|---|
| P0 | 언도킹 성공 미확인 | 도크에 붙은 채 `PATROLLING` 허위 보고 | action 결과 + fresh DockStatus 확인 |
| P0 | `target_pose` 소유권 없음 | opening·rat·trap·herd 목표가 서로 선점 | typed motion request/result + operation ID |
| P0 | trap 설치 성공 가장 | 이동 실패에도 `trap_installed` 발행 | 논블로킹 스텝머신 + motion result |
| P0 | central rat mode 영구 고착 | 명령 유실·노드 사망 후 전체 조율 정지 | command ACK·재시도 + session timeout |
| P0 | 지도 unknown을 free로 분류 | 미확인 영역으로 경로 생성 가능 | map 임계값·planner 정책 동시 수정 |
| P0 | RGB-depth/TF 시점 불일치 | 구멍·trap·rat map 좌표 계통 오차 | aligned depth + 관측 stamp TF |
| P1 | 저전압 역할 수행 지속 | TRACK/HERD/SWEEP 중 방전 위험 | battery freshness + 모든 이동 상태 override |
| P1 | B 역할 자격 미검사 | 복귀·저전압·고장 로봇에 역할 배정 | 상태·배터리·capability·TTL 필터 |
| P1 | Nav action 무한 대기/거절 무시 | agent callback과 status 보고 정지 | 비동기 adapter + accepted/result timeout |
| P1 | String 파서 예외 | 잘못된 전역 메시지 하나로 노드 종료 | 방어 파싱 + enum/range 검증 |
| P1 | 늦은 DB/trap 응답 오인 | 이전 작업 결과가 새 작업을 전이 | operation/attempt/request ID |
| P1 | DB 상대경로·event 기반 저장 | 실행 위치별 DB 분리·저장 유실 | 절대 경로 + typed write service ACK |
| P1 | 미구현 노드가 준비된 것처럼 실행 | HERD/webcam/droppings 기능 공백 은폐 | capability/health gate |
| P2 | `rat_detected` 프레임당 발행 | DDS·로그 과부하 | transition event와 observation 분리 |
| P2 | 문서·의존성·테스트 불일치 | 새 환경 실행 실패·회귀 미검출 | 실행 문서 갱신 + 직접 의존성 + 통합 테스트 |

---

## 문제 2 — 언도킹 성공 확인과 상태 진실성

### 현재 문제

[`robot_agent.py`](../../turtle_project/robot_agent.py)의 `UNDOCK` 단계는
`isUndockComplete()`가 참이면 action의 성공·거절·취소 여부와 dock sensor를 확인하지
않고 후속 순찰 또는 역할을 시작한다. 사용 중인 TurtleBot4 Navigator의 complete
함수는 실패한 action도 "처리가 끝났다"는 의미로 참을 반환할 수 있다.

또한 시작 직후 `nav.is_docked`가 아직 `None`인 경우 `bool(None)`은 거짓이므로,
실제로 도크 위에 있어도 필드에 있는 것으로 오인할 수 있다.

### 해결 설계

`TurtleBot4Navigator`를 직접 상태기계에서 호출하지 않고 프로젝트 내부에
`NavActionAdapter`를 둔다. vendor helper의 blocking 편의 함수 대신 각 action의
goal handle과 result future를 보존한다.

```text
UndockOutcome = SUCCEEDED | REJECTED | FAILED | CANCELED | TIMEOUT | SERVER_UNAVAILABLE
DockObservation = DOCKED | UNDOCKED | STALE | UNKNOWN
```

언도킹 성공은 아래 세 조건이 모두 참일 때만 인정한다.

1. undock goal이 서버에 accepted 됨
2. result status가 `SUCCEEDED`
3. `dock_status_ttl_sec` 이내에 받은 `DockStatus.is_docked`가 명시적으로 `False`

권장 파라미터:

```yaml
dock_server_wait_sec: 3.0
dock_goal_accept_sec: 3.0
dock_action_timeout_sec: 45.0
dock_status_ttl_sec: 3.0
```

`robot_agent`의 단계는 다음처럼 분리한다.

```text
IDLE/DOCKED
  -> UNDOCK_SERVER_WAIT
  -> UNDOCK_ACCEPT_WAIT
  -> UNDOCK_RESULT_WAIT
  -> UNDOCK_SENSOR_WAIT
  -> 후속 PATROL 또는 ROLE
```

실패 시 처리:

- action success + fresh `is_docked=False`: 후속 작업 실행
- action success + fresh `is_docked=True`: `DOCKED`, 후속 작업 금지
- sensor stale/unknown: `FAULT`, 후속 작업 금지
- rejected/failed/canceled/timeout/server unavailable: `FAULT`
- timeout 시 matching goal handle에 cancel을 정확히 한 번 요청
- `dock_phase`, `after_undock`, pending motion/role을 모두 정리
- 상태가 바뀌면 10초 보고 타이머를 기다리지 않고 status 즉시 발행

도킹도 같은 accepted/result/sensor freshness 계약을 사용한다. `isDockComplete()`가
참인 것만으로 성공 처리하지 않는다.

### 검증 기준

- action success + fresh undocked에서만 PATROL 시작
- rejected/failed/canceled/timeout에서 PATROL과 역할 상태 금지
- action success지만 sensor가 계속 docked면 `DOCKED`
- sensor 미수신 또는 stale이면 `FAULT`
- agent 시작 후 첫 fresh DockStatus 전에는 `IDLE`/`DOCKED`를 추측하지 않음
- 각 실패 경로에서 후속 waypoint/target goal이 0건인지 확인

---

## 문제 3 — 이동 명령 소유권과 stale goal 차단

### 현재 문제

`detector_node`, `trap_check_node`, `rat_herding_node`가 같은 로봇 로컬
`target_pose`를 발행하고, `robot_agent.target_cb()`는 현재 역할이나 발행자를 확인하지
않고 모든 `PoseStamped`를 실행한다.

이 구조에서는 다음 경합이 가능하다.

- TRACK 전환 후 늦은 opening 접근 goal이 rat goal을 선점
- rat 추적 중 trap 설치/점검 goal이 추적 goal을 선점
- STOP 뒤 DDS 큐에 남은 goal이 다시 주행을 시작
- 도킹 완료 후 stale goal이 들어와 도크 위에서 NavigateToPose 실행
- 한 작업의 cancel이 새 작업의 goal handle을 취소

### 인터페이스

`turtle_interfaces`에 다음 메시지를 추가한다.

```text
# MotionRequest.msg
builtin_interfaces/Time stamp
string robot
string owner          # patrol | opening | rat | trap | herd | sweep | dock
uint64 operation_id
uint32 step_id
string motion_type    # NAVIGATE | FOLLOW_WAYPOINTS | BACKUP | CANCEL
geometry_msgs/PoseStamped target
geometry_msgs/PoseStamped[] waypoints
float32 distance
float32 speed
float32 timeout_sec

# MotionResult.msg
string robot
string owner
uint64 operation_id
uint32 step_id
string outcome        # accepted | succeeded | rejected | failed | canceled | timeout
string detail
geometry_msgs/PoseStamped final_pose
```

토픽은 로봇 로컬 상대경로를 사용한다.

```text
motion/request   -> robot_agent만 구독
motion/result    <- robot_agent만 발행
```

### robot_agent 규칙

`robot_agent`를 로봇당 유일한 motion executor로 유지하되 다음 규칙을 적용한다.

1. 현재 `(owner, operation_id, step_id)`를 `active_motion`으로 저장한다.
2. 요청의 `robot`이 현재 namespace와 다르면 거부한다.
3. 현재 역할과 맞지 않는 owner는 `rejected`로 응답한다.
4. 이미 canceled/finished된 operation ID의 늦은 요청은 거부한다.
5. 새 요청이 기존 요청을 선점할 수 있는지는 명시적인 priority 표로 결정한다.
6. 선점 시 기존 goal의 cancel result를 받은 뒤 새 goal을 보낸다.
7. action accepted를 확인한 뒤에만 `accepted`를 발행한다.
8. action 결과는 같은 owner/operation/step으로 돌려준다.

권장 우선순위:

```text
STOP/비상저전압 > dock 복귀 > rat 대응 > trap/opening/sweep/herd > patrol
```

동일 우선순위 요청은 implicit preemption을 허용하지 않는다. 움직이는 rat의 goal
갱신처럼 선점이 필요한 경우 같은 operation ID에서 증가하는 step ID를 사용하고,
robot_agent가 직전 step cancel 또는 action server의 명시적 update 계약을 적용한다.

`PoseStamped target_pose`, `Bool cancel_drive`, `Bool patrol_hold`는 전환 기간 후
제거한다. 한 번에 제거하기 어렵다면 adapter가 legacy 토픽을 MotionRequest로 바꾸되,
legacy 요청은 owner와 operation ID를 추론할 수 없으므로 실기 기본에서는 비활성화한다.

### 상위 명령 전환

TRACK/DOCK/STOP 같은 fleet 명령을 받으면 detector의 내부 bool만 바꾸지 않는다.

```text
1. 현재 operation을 cancel
2. MotionResult(canceled) 또는 cancel timeout 확인
3. 현재 operation ID 폐기
4. 새 역할/작업 시작
```

trap_check에도 matching cancel job을 보내 사람이 설치하는 중인 이전 작업을 종료한다.

### 검증 기준

- opening goal 직후 TRACK을 넣어 최종 active owner가 rat 하나인지 확인
- STOP 후 이전 operation의 request를 넣어 goal이 발행되지 않는지 확인
- trap과 rat request를 역순·동시에 넣어 priority가 항상 동일한 결과인지 확인
- cancel 완료 전에 새 goal이 발행되지 않는지 확인
- action 결과가 다른 operation의 상태를 바꾸지 않는지 확인
- 두 namespace에서 robot4 request가 robot6에서 실행되지 않는지 확인

---

## 문제 4 — trap 설치를 논블로킹 스텝머신으로 변경

### 현재 문제

`trap_check_node._install()`은 접근 goal 발행 후 고정 시간 sleep, 후퇴 goal 발행 후
고정 시간 sleep을 하고 실제 이동 결과와 무관하게 `trap_installed`를 발행한다.
TF 실패로 goal을 하나도 못 보내도 성공 처리되며, sleep 동안 executor가 새 job이나
cancel을 처리하지 못한다.

### 인터페이스 보강

```text
# TrapJob.msg
uint64 operation_id
uint32 attempt_id
string phase          # install | inspect | cancel
float64 hole_x
float64 hole_y
float64 trap_x
float64 trap_y

# TrapResult.msg
uint64 operation_id
uint32 attempt_id
string outcome        # installed | ok | bad | failed | canceled | busy
float64 hole_x
float64 hole_y
string detail
```

상태 전이용 결과는 `trap/result` 로컬 typed 토픽만 사용한다. 전역
`/fleet/event`는 detector가 현재 operation의 성공 결과를 수락한 뒤 관찰용으로 한
번 발행한다.

### 상태기계

```text
IDLE
  -> APPROACH_REQUEST
  -> APPROACH_WAIT
  -> HUMAN_WAIT
  -> BACKUP_REQUEST
  -> BACKUP_WAIT
  -> COMPLETE
```

세부 규칙:

- `APPROACH_REQUEST`: TF로 유효한 접근 pose를 계산하지 못하면 즉시 `failed`
- `APPROACH_WAIT`: MotionResult가 succeeded이고 TF 거리도 tolerance 이내일 때만 다음 단계
- `HUMAN_WAIT`: **도착 확인 뒤부터** 사람이 trap을 놓을 시간을 계산
- `BACKUP_REQUEST`: 단순 뒤쪽 map pose가 아니라 Nav2 `BackUp` behavior를 요청
- `BACKUP_WAIT`: succeeded 확인 뒤에만 `installed`
- 각 단계는 `time.monotonic()` 기반 timeout 보유
- cancel은 현재 matching motion을 취소하고 `canceled`를 한 번만 발행
- install 중 새 install/inspect는 기존 job을 덮지 않고 `busy`
- beep는 `cmd_audio`만 사용하거나 `subprocess.Popen` fire-and-forget으로 실행
- `time.sleep()`과 blocking `subprocess.run()`은 제거

권장 시작 파라미터:

```yaml
approach_dist: 0.20
approach_arrive_tol: 0.10
approach_timeout_sec: 30.0
install_wait_sec: 8.0
backup_dist: 0.25
backup_speed: 0.05
backup_timeout_sec: 10.0
result_margin_sec: 5.0
```

detector의 trap 대기 timeout은 위 단계의 최대 합으로 계산한다. 현재처럼 별도 상수
`AWAIT_TRAP=20`을 복제하지 않는다.

inspect 결과도 `operation_id + attempt_id`가 맞을 때만 수락한다. 좌표는 finite인지,
map frame인지, hole과의 최대 허용 거리를 벗어나지 않는지 먼저 검증한다.

### 검증 기준

- approach/backup success에서만 installed 1회
- rejected/failed/canceled/timeout/TF 실패에서 installed 0회, failed 1회
- HUMAN_WAIT 시작 시점이 approach result 이후인지 확인
- job 중 새 job은 busy이고 기존 job은 유지되는지 확인
- cancel 뒤 늦은 motion success가 installed로 바뀌지 않는지 확인
- callback 실행 중 blocking sleep이 없는지 정적 검사

---

## 문제 5 — central 쥐 대응 세션의 ACK·timeout·복구

### 현재 문제

central은 `rat_detected`를 받자마자 `rat_mode=True`로 만들고 String 명령을 한 번
발행한다. TRACK 명령이 유실되거나 detector 프로세스가 죽으면 종료 이벤트가 오지
않아 rat mode가 영구 유지된다. rat mode 중에는 일반 커버리지 복구도 중단된다.

이 절은 제외된 6번의 **포획 판정 방식 자체는 변경하지 않는다**. 기존
`rat_captured`/`rat_lost`를 세션 종료 입력으로 사용하되 전달·timeout을 안전하게 만든다.

### typed command와 ACK

```text
# FleetCommand.msg
uint64 command_id
string robot
string command
uint64 session_id

# CommandAck.msg
uint64 command_id
string robot
string command
uint64 session_id
string component     # robot_agent | detector | rat_herding | trap_check
string outcome       # accepted | rejected
string detail
```

central은 process random prefix와 counter로 command/session ID를 생성한다. ACK가
`command_ack_timeout_sec` 안에 없으면 같은 command ID로 제한 재전송한다.
robot_agent와 detector는 최근 command ID를 bounded cache에 보관해 중복 실행하지 않고
같은 ACK만 다시 발행한다. 하나의 명령을 여러 컴포넌트가 소비하므로 central은
`command_id` 하나만 보지 않고 `(command_id, component)`별 ACK를 기다린다.

명령별 필수 ACK 집합:

```text
TRACK   -> robot_agent + detector
SWEEP   -> robot_agent + detector
HERD    -> robot_agent + rat_herding
PATROL  -> robot_agent + detector
DOCK    -> robot_agent + detector
UNDOCK  -> robot_agent
STOP    -> 현재 command를 소비하는 모든 active component
```

필수 컴포넌트 중 하나라도 reject하면 해당 명령은 전체 accepted가 아니다. 일부만
accepted된 경우 central은 같은 session ID로 STOP/cancel을 보내 부분 전이를 원복한다.

### 세션 상태

```text
IDLE -> STARTING -> ACTIVE -> ENDING -> IDLE
```

- `STARTING`: A의 robot_agent와 detector TRACK ACK를 모두 기다림
- A accepted 후에만 `ACTIVE`
- B가 있으면 SWEEP ACK를 별도로 확인
- A reject 또는 ACK timeout: 세션 취소, rat mode 해제
- B reject 또는 ACK timeout: `(A, None)` 단독 추적으로 축소
- `rat_captured`/`rat_lost`: `ENDING`에서 A PATROL, B DOCK 명령과 ACK 처리
- detector/agent status 또는 capability stale: 세션 실패 종료
- `rat_observation_lost_sec` 동안 관측 없음: 실패 종료
- `rat_session_timeout_sec` 초과: 실패 종료
- 종료 처리는 session ID당 한 번만 실행

권장 파라미터:

```yaml
command_ack_timeout_sec: 2.0
command_retry_max: 3
rat_observation_lost_sec: 5.0
rat_session_timeout_sec: 120.0
ending_timeout_sec: 10.0
```

central의 stale 정리는 rat mode에서도 계속 수행한다. A가 사라지면 rat session을
종료하고, B가 사라지면 A 단독 모드로 축소한다. 커버리지 복구는 session 정리가
끝난 뒤 재개한다.

### 검증 기준

- 첫 TRACK 명령 유실 후 같은 ID로 재전송되고 중복 실행은 1회인지 확인
- A reject/ACK timeout에서 rat mode가 해제되는지 확인
- B reject에서 A 단독 추적으로 계속되는지 확인
- detector 사망·observation 단절·전체 session timeout에서 종료되는지 확인
- 늦은 이전 session의 captured/lost/ACK가 새 session을 종료하지 않는지 확인
- ENDING 중 중복 종료 이벤트가 PATROL/DOCK 명령을 중복 실행하지 않는지 확인

---

## 문제 7 — 지도 unknown과 Nav2 계획 안전 설정

### 현재 문제

현재 `room_map.pgm`은 16,241픽셀 중 5,614픽셀이 값 205다. `negate: 0`일 때
occupancy는 약 `0.196078`인데 `room_map.yaml`의 `free_thresh: 0.25`로 인해 이 회색
픽셀이 free로 분류된다. 원래 unknown으로 보존해야 할 영역이 주행 가능 영역이 될
수 있다.

`nav2.yaml`의 planner도 `allow_unknown: true`라 threshold를 고쳐 unknown으로 만든
뒤에도 계획 정책이 이를 허용할 수 있다.

### 해결 설계

실기 기본 프로파일은 다음처럼 둔다.

```yaml
# room_map.yaml
free_thresh: 0.196
occupied_thresh: 0.65

# nav2.yaml
planner_server:
  ros__parameters:
    GridBased:
      allow_unknown: false
```

`0.196`은 값 205의 occupancy `0.196078...`보다 작아서 trinary map에서 unknown으로
남긴다. 임계값은 PGM 샘플과 map_server OccupancyGrid 덤프로 검증한 뒤 확정한다.

추가 검증 도구를 만든다.

1. PGM histogram과 YAML threshold로 free/unknown/occupied 개수를 계산
2. map_server가 발행한 OccupancyGrid와 계산 결과 비교
3. 모든 waypoint와 dock pose가 map 범위 안인지 검사
4. 단일 픽셀이 아니라 robot footprint + inflation 반경 안에 lethal/unknown이 없는지 검사
5. waypoint 사이 global path가 unknown/occupied를 밟지 않는지 검사

simulation에서 unknown 통과가 필요하면 별도 `nav2_sim.yaml` 또는 launch parameter로
명시적으로 켜고 실기 기본과 섞지 않는다.

### 검증 기준

- 값 205 표본이 OccupancyGrid에서 `-1`인지 확인
- global plan의 모든 cell이 free인지 검사
- 36개 waypoint와 두 dock pose의 footprint clearance 검사
- unknown을 가로지르는 목표가 planner에서 실패하는지 확인
- 수정 후 저속으로 외곽·좁은 통로를 시험하고 costmap 스크린샷 보존

---

## 문제 8 — RGB-depth 공간 정합과 관측 시각 TF

### 현재 문제

YOLO bbox는 RGB 픽셀인데 코드는 해상도 비율만으로 stereo depth 픽셀을 계산하고
stereo CameraInfo의 K로 deproject한다. RGB와 depth가 동일 optical frame/FOV로
정렬됐다는 런타임 검사가 없다.

`PointStamped.header.stamp`도 설정하지 않아 TF가 최신 자세를 사용한다. 촬영 후
동기화·전송·YOLO 추론이 끝날 때까지 로봇이 이동하면 map 좌표가 체계적으로 밀린다.

### 해결 설계

#### 공간 정합

우선순위는 다음과 같다.

1. OAK-D가 제공하는 **RGB-aligned depth** 토픽 사용
2. aligned 토픽이 없으면 depth 내·외부 파라미터로 RGB ray를 depth frame에 투영
3. 단순 해상도 비례 변환은 정렬 검증을 통과한 장치에서만 허용

시작 시 다음을 검증한다.

- RGB, depth, CameraInfo의 frame ID
- 선택한 CameraInfo의 width/height와 depth 영상 크기
- aligned depth라면 기준 optical frame이 RGB와 일치하는지
- 허용된 robot별 calibration profile인지

검증 실패 시 detector는 탐지를 계속하는 척하지 않고 capability의
`detector_ready=false`, detail=`RGB_DEPTH_NOT_ALIGNED`를 보고한다.

#### 시간 정합

`synced_cb`가 실제 관측 stamp를 `_box_to_map()`에 전달한다.

```python
pt.header.frame_id = depth_msg.header.frame_id
pt.header.stamp = depth_msg.header.stamp
```

변환 전 `can_transform(map, depth_frame, observation_stamp, timeout)`을 확인하고,
해당 시각 TF가 없으면 그 프레임을 버린다. 최신 TF로 폴백하지 않는다.

추가로 현재 시각과 observation stamp 차이가 `max_observation_age_sec`보다 크면
좌표와 motion request를 발행하지 않는다. ROS clock을 쓰는 센서 stamp 비교와 별도로
process stall 감시는 monotonic clock을 사용한다.

### 검증 기준

- RGB 박스 중심과 aligned depth 위치가 같은 물체를 가리키는 시각 테스트
- 서로 다른 optical frame을 넣으면 detector ready가 false인지 확인
- 과거 stamp의 TF가 있을 때 해당 자세로 좌표가 계산되는지 확인
- 해당 stamp TF가 없을 때 최신 TF로 대체되지 않는지 확인
- 정지/0.2m/s 주행에서 동일 고정 표적의 map 좌표 편차 비교
- robot4/robot6 각각 별도 calibration 결과 기록

---

## 추가 문제 A — 배터리 freshness와 모든 역할에서의 안전 복귀

### 현재 문제

배터리 임계 복귀는 상태가 정확히 `PATROLLING`일 때만 실행된다. TRACKING,
HERDING, SWEEPING, opening/trap 처리 중 저전압은 무시된다. 첫 BatteryState 전에는
기본값 100을 사용하므로 실제 배터리를 모른 채 역할 후보가 될 수도 있다.

### 해결 설계

`battery`를 단일 숫자가 아니라 다음 상태로 관리한다.

```text
BatteryHealth = UNKNOWN | NORMAL | LOW | CRITICAL | STALE
last_battery_at: monotonic timestamp
```

권장 정책:

- 첫 valid BatteryState 전에는 `UNKNOWN`, 역할·순찰 시작 금지
- `battery_status_ttl_sec` 초과 시 `STALE`, 새 역할 배정 금지
- LOW: 새 장기 작업 금지, 현재 안전 단계 종료 후 복귀
- CRITICAL: 현재 motion/trap/rat/sweep를 cancel하고 즉시 dock 복귀
- charging/docked 상태에서 복귀 명령 중복 금지
- LOW 해제에는 hysteresis 적용
- NaN, inf, 음수, 100 초과는 invalid로 거부

예시:

```yaml
battery_low_percent: 25
battery_critical_percent: 15
battery_resume_percent: 35
battery_status_ttl_sec: 20.0
```

BatteryState 구독 QoS는 실제 publisher endpoint와 대조한다. publisher가 BestEffort면
`qos_profile_sensor_data`를 사용하고 시작 시 QoS incompatibility를 진단한다.

central status는 배터리 숫자와 함께 freshness/health를 typed field로 받는다. B 역할
선정과 wake 후보 선정에서 UNKNOWN/STALE/LOW/CRITICAL을 제외한다.

### 검증 기준

- TRACK/HERD/SWEEP/opening/trap 각 상태에서 critical 입력 시 motion cancel + RETURNING
- 첫 BatteryState 전 UNDOCK/PATROL/역할 거부
- stale battery 로봇이 B 또는 wake 후보에서 제외
- LOW 경계에서 hysteresis로 반복 복귀/재시작하지 않는지 확인
- QoS를 BestEffort/RELIABLE 조합으로 바꾼 연결 테스트

---

## 추가 문제 B — 역할 후보 자격과 wake 재선정

### 현재 문제

현재 `assign_roles()`는 patroller가 아닌 첫 로봇을 B로 고른다. B가 RETURNING,
저전압, stale, FAULT 또는 다른 작업 중이어도 선택될 수 있다.

wake timeout이 지나도 실패한 로봇에 cooldown이 없어 dict 삽입 순서상 같은 IDLE
로봇을 계속 다시 선택할 수 있다.

### 해결 설계

central 파라미터로 허용 로봇을 명시한다.

```yaml
allowed_robots: [robot4, robot6]
status_ttl_sec: 25.0
wake_timeout_sec: 60.0
wake_retry_cooldown_sec: 120.0
```

B 후보 조건:

- allowed robot
- fresh status와 fresh capability
- state가 `IDLE` 또는 `DOCKED`
- battery health가 NORMAL
- dock/undock/motion/trap 작업 중이 아님
- wake cooldown 중이 아님

우선순위는 필드 위 IDLE, 정상 DOCKED 순으로 한다. B가 없으면 A 단독 추적을 허용하고
`robot_b=None`으로 저장한다. `_end_rat()`와 모든 후속 명령은 B가 None일 수 있게
가드한다.

wake 실패 처리:

- waking 로봇이 IDLE/FAULT를 보고하면 즉시 실패
- timeout이면 실패
- 실패한 로봇은 cooldown에 넣고 다른 후보 우선
- status가 stale이면 후보에서 제거
- 1초 coordination timer가 status 수신과 독립적으로 재선정
- 모든 시간은 monotonic 사용

### 검증 기준

- RETURNING/FAULT/LOW/STALE B가 제외되는지 확인
- B가 없을 때 A 단독 세션이 정상 종료되는지 확인
- robot4 wake 실패 후 robot6이 다음 후보가 되는지 확인
- 둘 다 cooldown/stale이면 명령을 추측해 보내지 않는지 확인
- 허용 목록 밖 `robot9:PATROLLING`이 patroller가 되지 않는지 확인

---

## 추가 문제 C — Nav action의 유한 timeout과 결과 확인

### 현재 문제

현재 코드는 `followWaypoints()`와 `goToPose()`의 반환값을 무시한 채
`patrolling=True`, `driving=True`를 설정한다. 사용 중인 helper는 action server가
없을 때 wait loop에 머물 수 있고 cancel future에도 timeout이 없어 agent callback과
status timer를 막을 수 있다.

### 해결 설계

문제 2의 `NavActionAdapter`가 모든 action을 다음 공통 계약으로 실행한다.

```text
SERVER_WAIT -> GOAL_ACCEPT_WAIT -> RESULT_WAIT -> FINISHED
                                  \-> CANCEL_WAIT
```

각 단계는 timer tick에서 future의 완료 여부만 확인하며 callback 안에서
`spin_until_future_complete()`나 무한 `wait_for_server()`를 호출하지 않는다.

적용 대상:

- FollowWaypoints
- NavigateToPose
- BackUp
- Dock
- Undock
- 필요 시 Spin

공통 결과:

```text
accepted | rejected | succeeded | failed | canceled | timeout | server_unavailable
```

상태 변경 시점:

- FollowWaypoints accepted 후에만 PATROLLING
- NavigateToPose accepted 후에만 driving/active_motion 설정
- rejected/server unavailable이면 이전 상태로 복구하거나 FAULT
- result timeout이면 matching goal cancel 후 timeout result
- cancel timeout이면 FAULT로 올리고 새 motion 발행 금지

Nav2 readiness도 agent 시작 시 확인한다. readiness가 확인되기 전 status는 FAULT이며
central이 자동 역할을 주지 않는다.

### 검증 기준

- 각 action server 부재 시 지정 시간 안에 callback이 반환되고 status timer가 계속 동작
- rejected goal에서 PATROLLING/driving이 설정되지 않음
- result timeout에서 cancel 1회와 timeout result 1회
- cancel 응답 전 후속 goal 0건
- action 결과가 현재 goal handle과 일치하는지 확인

---

## 추가 문제 D — fleet String 메시지 방어 파싱

### 현재 문제

`fleet_msg.parse_*()`는 `split` 언패킹과 숫자 변환 예외를 그대로 던진다. 전역
`/fleet/status`, `/fleet/event`, `/fleet/command`에 잘못된 메시지가 한 번 들어오면
여러 노드의 `rclpy.spin()`이 종료될 수 있다.

### 해결 설계

command는 문제 5의 typed message로 전환한다. 전환 전과 관찰용 status/event에는
방어 파서를 적용한다.

```python
def parse_status(value):
    """Return parsed status or None; never raise for input data."""

def parse_event(value):
    """Return parsed event or None; never raise for input data."""
```

검증 항목:

- 정확한 필드 수
- robot ID 정규식 `[A-Za-z0-9_-]+`
- 허용 state/event enum
- battery 0..100
- 좌표 `math.isfinite()`
- 빈 문자열·공백·콜론·경로 구분자 거부

모든 callback은 다음 패턴을 사용한다.

```python
parsed = fleet_msg.parse_event(msg.data)
if parsed is None:
    logger.warning('invalid fleet event', throttle_duration_sec=5.0)
    return
```

유효하지 않은 외부 입력은 경고만 남기고 상태를 변경하지 않는다. parser가 특정
robot4/robot6을 하드코딩하지는 않되 central의 `allowed_robots`에서 별도 제한한다.

### 검증 기준

- 필드 부족/초과, 숫자 오류, NaN/inf, battery 범위 밖, 빈 robot 테스트
- 무작위 문자열 fuzz 10,000건에서 예외 0건
- invalid status/event 후 각 노드가 계속 timer callback을 수행하는지 확인
- 알 수 없는 enum이 상태기계에 들어가지 않는지 확인

---

## 추가 문제 E — DB·trap 비동기 응답의 작업 상관관계

### 현재 문제

현재 `_db_done()`은 `state == QUERYING`만 확인한다. 이전 opening의 DB 응답이 늦게
도착했을 때 마침 새 opening도 QUERYING이면 이전 결과를 새 작업에 적용할 수 있다.
로컬 trap 토픽은 두 로봇 교차 수신은 줄였지만 같은 로봇에서 이전 설치 결과가 새
`AWAIT_TRAP`에 들어오는 문제는 남아 있다.

### 해결 설계

detector가 작업 시작 시 operation ID를 만든다.

```text
operation_id = (random_32bit_process_prefix << 32) | monotonic_counter
attempt_id   = reinstall count
request_id   = DB request counter or operation-derived ID
```

DB future callback은 요청 시점의 ID를 캡처한다.

```python
future = client.call_async(req)
pending_db[request_id] = (operation_id, future)
future.add_done_callback(lambda fut, rid=request_id: db_done(rid, fut))
```

응답 수락 조건:

- request ID가 pending에 있음
- 응답의 operation ID가 현재 operation과 일치
- 현재 state가 해당 요청을 기다리는 상태
- timeout/cancel된 operation이 아님

trap은 문제 4의 operation + attempt를 모두 검사한다. timeout 또는 상위 명령 전환 시
현재 operation을 canceled set에 넣고 DB future, trap job, motion을 함께 취소한다.

### 검증 기준

- op10 DB timeout 후 op11 QUERYING 중 op10 응답을 넣어 op11 무변경
- 같은 op에서 attempt 0 결과가 attempt 1을 전이하지 않음
- detector 재시작 직전의 local trap 결과가 새 operation에 적용되지 않음
- cancel 뒤 늦은 DB/trap/motion 결과가 모두 무시됨

---

## 추가 문제 F — DB 절대경로와 저장 ACK

### 현재 문제

기본 `holes.db`가 상대경로라 node를 실행한 shell의 cwd에 따라 서로 다른 DB가
생긴다. 구멍과 trap 저장은 관찰용 `/fleet/event` 구독에 의존하므로 DB가 잠시
꺼져 있거나 subscriber가 늦게 붙으면 영구 유실된다.

### 해결 설계

기본 경로는 확장된 절대경로로 한다.

```text
~/.ros/turtle_project/holes.db
```

launch에서 경로를 넘기고 부모 디렉터리 생성·쓰기 권한·DB open 실패 시 node 시작을
실패시킨다. 상대경로 파라미터는 거부한다.

저장 인터페이스:

```text
# RecordHole.srv
uint64 request_id
float64 x
float64 y
---
bool success
int64 hole_id
float64 canonical_x
float64 canonical_y
string detail

# UpdateTrap.srv
uint64 request_id
int64 hole_id
bool installed
---
bool success
string detail
```

DB 설정:

- schema version table
- `PRAGMA busy_timeout`
- 필요 시 WAL mode
- transaction 안에서 중복 검사와 INSERT/UPDATE
- 처리한 request ID unique 기록으로 idempotency 보장
- 좌표는 finite/range 검증
- shutdown에서 commit/connection close

detector는 service commit ACK를 받은 뒤에만 `opening_confirmed`, `trap_ok/bad`를
관찰용 event로 발행한다. timeout이면 **같은 request ID**로 제한 재시도한다.

### 검증 기준

- 서로 다른 cwd에서 실행해도 같은 절대 DB 사용
- DB down 상태에서 저장 성공 event를 발행하지 않음
- ACK 유실 후 같은 request ID 재시도 시 row가 1개만 존재
- 동시 query/write와 busy timeout 테스트
- 재시작 후 hole/trap 상태 유지 및 schema version 확인

---

## 추가 문제 G — rat event 스로틀과 observation 분리

### 현재 문제

TRACK 전에는 쥐가 보이는 카메라 프레임마다 `rat_detected`를 발행한다. central은 첫
이벤트만 사용하지만 DDS 트래픽과 로그는 프레임 속도로 증가한다. herding이 구현될
경우 transition event를 위치 스트림으로 오용할 위험도 있다.

### 해결 설계

세션 시작 신호와 지속 관측을 분리한다.

```text
# RatObservation.msg
builtin_interfaces/Time stamp
string source
string frame_id       # map만 허용
uint64 track_id
float64 x
float64 y
float32 confidence
```

- `/fleet/event/rat_detected`: mode 진입용, 최대 1Hz 및 edge-trigger
- `/fleet/rat_observation`: 지속 위치, 기본 1Hz
- observation은 문제 8의 실제 camera stamp와 map frame 사용
- herding은 observation TTL보다 오래된 좌표를 사용하지 않음
- 서로 다른 source 관측은 stamp, confidence, calibration health로 선택
- central은 마지막 observation 시각을 문제 5의 session watchdog에 사용

이 절은 포획 판정 알고리즘을 변경하지 않는다.

### 검증 기준

- 30fps 입력에서 transition event와 observation이 설정 주기 이하인지 확인
- TRACK 전후 observation은 계속되고 transition event는 폭주하지 않음
- stale, 과거 track ID, map 이외 frame 관측 거부
- observation 중단 시 herding motion 정지 및 central session timeout 작동

---

## 추가 문제 H — capability·health gate와 미구현 기능 처리

### 현재 문제

`rat_herding_node`는 목표 계산이 TODO이고 `webcam_node.tick()`은 `pass`다. YOLO 모델의
`droppings` 클래스도 업무 흐름에서 소비되지 않는다. detector는 모델 로드 실패 시
살아 있는 node처럼 보이지만 탐지를 전부 건너뛴다. central은 이러한 기능 준비도를
확인하지 않고 역할을 배정한다.

### 해결 설계

```text
# NodeCapability.msg
builtin_interfaces/Time stamp
string node
string robot
bool ready
string[] capabilities
string detail
```

capability 예시:

```text
detector: opening_detection, rat_detection, trap_detection
trap_check: trap_install, trap_inspect
rat_herding: herd_path_generation
webcam: fixed_camera_rat_detection
db: hole_query, hole_record, trap_update
```

ready 조건:

- 필수 모델/리소스 로드 성공
- 필수 publisher/subscriber/action/service 연결
- camera/TF/DB의 최근 정상 입력이 TTL 이내
- 핵심 알고리즘이 구현됨
- 최근 치명 오류 없음

미구현 기능은 다음 둘 중 하나로 명확히 처리한다.

1. entry point는 유지하되 `ready=false`, `NOT_IMPLEMENTED`를 지속 보고하고 central이
   절대 해당 역할을 주지 않음
2. 제품 실행 범위 밖이면 central role과 실행 문서에서 제거

`droppings`가 제품 범위라면 typed observation과 DB/UI 저장 계약을 별도 구현한다.
범위가 아니라면 모델 클래스가 감지돼도 업무 이벤트가 발생하지 않는다는 사실을
문서화하고 capability에서 false로 표시한다.

YOLO 추론은 ROS single-thread callback에서 장시간 실행하지 않도록 최신 프레임 하나만
보존하는 bounded worker queue를 사용한다. 추론 worker가 느려져도 watchdog,
cancel, spin stop, capability timer가 계속 실행돼야 한다. 오래된 추론 결과는 관측
stamp age로 폐기한다.

### 검증 기준

- model load 실패, camera 단절, TF 단절에서 detector ready가 false
- herding TODO 상태에서 central이 HERD를 보내지 않음
- webcam 미구현 상태가 시작 로그만으로 ready가 되지 않음
- inference를 인위적으로 지연해도 cancel/watchdog/status timer가 진행
- capability TTL 만료 로봇이 역할 후보에서 제외

---

## 추가 문제 I — verification timeout 단위 명확화

### 현재 문제

`verify_timeout`이라는 이름은 시간처럼 보이지만 VERIFYING/INSPECTING에서는 frame
miss 개수로 사용한다. 실제 추론 FPS가 장치와 부하에 따라 달라서 같은 30이 약 1초일
수도, 10초 이상일 수도 있다. trap 설치 직후 점검이 지나치게 빨리 재설치로 넘어갈
수 있다.

### 해결 설계

운영 timeout은 monotonic 초 단위로 통일한다.

```yaml
opening_verify_timeout_sec: 10.0
trap_detect_timeout_sec: 10.0
min_verify_frames: 5
```

성공·실패 결정은 최소 frame 수와 wall timeout을 함께 사용한다.

- 충분한 유효 프레임 전에는 단일 miss로 실패하지 않음
- timeout까지 유효 frame이 부족하면 `insufficient_observation`
- 카메라 자체가 끊기면 capability false와 operation failed
- frame count 변수는 이름에 `_frames`를 붙여 시간과 구분

### 검증 기준

- 1/5/30fps 가짜 입력에서 실제 timeout 시간이 동일한지 확인
- 유효 frame 부족과 flat/trap-missing 판정을 구분
- 카메라 단절 시 timeout 후 순찰 재개 또는 상위 operation 실패

---

## 추가 문제 J — 실행 문서·패키지 의존성·설치 정리

### 현재 문제

- `docs/run.md`가 삭제된 `camera_node` 실행을 안내한다.
- 실제 executable은 8개지만 일부 문서는 9개 또는 과거 구현 상태를 설명한다.
- 중앙 실행 안내에 herding/webcam의 준비 상태가 명확하지 않다.
- `package.xml`에 직접 import하는 runtime/build dependency가 빠져 있다.
- app node 전체를 일관된 파라미터와 namespace로 실행하는 launch가 없다.

### 해결 설계

`docs/run.md`를 실제 executable 기준으로 다시 작성한다.

- detector가 OAK-D 원본을 직접 sync하므로 camera_node 단계 삭제
- robot4/robot6별 `__ns`, TF remap, params 파일을 한 명령에서 일치
- central/db/capability node의 실행 순서와 readiness 확인 명령 추가
- herding/webcam/droppings는 구현·capability 상태를 명시
- model checksum, 지원 ultralytics/Python 범위 기록
- DB 절대 경로와 로그 위치 기록

권장 launch 분리:

```text
robot_bringup.launch.py   # localization/Nav2 기반
robot_app.launch.py       # agent/detector/trap + namespace/config
central_app.launch.py     # central/db/선택적 herding/webcam
```

이 문서 범위에서는 제외 1번인 lifecycle 문제의 해결안을 다루지 않지만, app launch는 기반 Nav2의
ready 상태를 확인하지 못하면 역할 capability를 false로 유지해야 한다.

`package.xml` 직접 의존성 후보:

```text
buildtool_depend: ament_python
depend/exec_depend: ament_index_python
depend/exec_depend: irobot_create_msgs
depend/exec_depend: nav2_simple_commander
depend/exec_depend: turtlebot4_navigation
exec_depend: python3-yaml
exec/runtime 문서: ultralytics와 모델 runtime
```

실제 ROS 배포판의 package name과 rosdep key를 확인해 확정한다. transitive dependency에
기대지 않는다.

### 검증 기준

- `ros2 pkg executables turtle_project`와 문서 목록 일치
- 빈 install prefix에 빌드 후 source tree 없이 실행
- model/map/waypoint/config/script/launch 존재 확인
- robot4/robot6 app launch에서 토픽 namespace가 교차하지 않음
- rosdep check에서 직접 의존성 누락 0건

---

## 추가 문제 K — 테스트 체계 정리

### 현재 상태

- Python compile과 내장 self-check는 통과한다.
- pytest는 flake8 13건, pep257 92건으로 실패한다.
- 현재 test 디렉터리는 사실상 lint만 검사한다.
- self-check는 pytest가 자동 실행하지 않는다.
- 기본 pytest 실행 환경은 pytest-anyio 버전 충돌도 있다.
- action 실패, 명령 유실, TF 지연, 두 namespace, DB 재시작 통합 테스트가 없다.

### 해결 설계

1. self-check의 순수 함수를 `test/test_*.py`로 이동하거나 pytest에서 직접 호출한다.
2. lint baseline을 남기지 않고 flake8/pep257을 0 failure로 만든다.
3. pytest와 plugin 버전을 workspace/container에서 고정한다.
4. fake action server와 fake DockStatus publisher로 agent 전이를 자동 테스트한다.
5. fake DB/trap/motion service로 callback 역순과 stale ID를 테스트한다.
6. 두 namespace launch test로 로컬 토픽 격리를 검증한다.
7. 임시 install prefix를 사용하는 install-only 테스트를 추가한다.
8. rosbag 또는 고정 fixture로 RGB-depth/TF timestamp 정합을 검사한다.

필수 테스트 묶음:

```text
unit:
  fleet parser, role selection, battery policy, operation ID, map validation

component:
  robot_agent + fake Nav2
  detector + fake DB/trap/motion
  trap_check + fake motion result
  central + fake status/command ACK/capability

launch:
  robot4 + robot6 namespace isolation
  process death/stale recovery
  install-only resources

hardware gate:
  dock/undock result + sensor 3회
  STOP/critical battery
  map unknown 경계
  RGB-depth 고정 표적 좌표 분산
  trap approach/back-up 저속 실행
```

### 완료 기준

- pytest/ament lint 0 failure
- 모든 action failure matrix 자동 통과
- stale DB/trap/motion/command 결과가 새 작업을 바꾸지 않음
- 두 namespace 교차 motion/trap 결과 0건
- source tree 없이 설치본 실행 성공
- 저속 실기 gate 결과와 로그를 문서에 첨부

---

## 부록 A — 파일별 예상 변경

| 파일 | 변경 내용 |
|---|---|
| `turtle_interfaces/msg/MotionRequest.msg` | owner/operation/step 기반 이동 요청 |
| `turtle_interfaces/msg/MotionResult.msg` | accepted/result/cancel/timeout 결과 |
| `turtle_interfaces/msg/TrapJob.msg` | operation/attempt/cancel 추가 |
| `turtle_interfaces/msg/TrapResult.msg` | correlated trap 결과 추가 |
| `turtle_interfaces/msg/FleetCommand.msg` | command/session ID typed 명령 |
| `turtle_interfaces/msg/CommandAck.msg` | idempotent 명령 ACK |
| `turtle_interfaces/msg/RatObservation.msg` | 지속 rat 좌표와 freshness |
| `turtle_interfaces/msg/NodeCapability.msg` | 기능 준비도와 health TTL |
| `turtle_interfaces/srv/RecordHole.srv` | commit ACK가 있는 구멍 저장 |
| `turtle_interfaces/srv/UpdateTrap.srv` | commit ACK가 있는 trap 상태 변경 |
| `turtle_project/robot_agent.py` | NavActionAdapter, motion executor, battery override, 진실한 상태 보고 |
| `turtle_project/detector_node.py` | aligned depth/stamped TF, operation lifecycle, typed motion/trap/DB |
| `turtle_project/trap_check_node.py` | nonblocking install state machine와 correlated result |
| `turtle_project/central_node.py` | command ACK, rat session timeout, eligible role/wake selection |
| `turtle_project/db_node.py` | 절대경로, schema/request ID, typed write ACK |
| `turtle_project/rat_herding_node.py` | fresh RatObservation 기반 motion 또는 capability false |
| `turtle_project/webcam_node.py` | 구현 전 capability false, 구현 후 동일 observation 계약 |
| `turtle_project/fleet_msg.py` | legacy status/event 방어 파싱 |
| `resource/room_map.yaml` | unknown 보존 임계값 |
| `config/nav2.yaml` | 실기 `allow_unknown: false`와 안전 프로파일 |
| `launch/*.launch.py` | app launch와 공통 namespace/config/readiness |
| `package.xml`, `setup.py` | 직접 의존성과 설치 리소스 정합 |
| `docs/run.md`, `docs/architecture.md`, `docs/flowchart.md` | 현재 executable·typed 계약·capability 반영 |
| `test/*` | unit/component/launch/install/hardware gate |

## 부록 B — 권장 구현 순서

각 단계는 독립 커밋으로 만들고, 매 단계 후 compile·self-check·pytest를 실행한다.

1. **지도 안전 설정** — unknown 분류·planner 정책·waypoint clearance 검사
2. **fleet 방어 파싱** — 잘못된 전역 입력으로 노드가 죽는 경로 차단
3. **typed command/ACK와 operation ID 기반** — 이후 인터페이스의 공통 식별자 확정
4. **NavActionAdapter + 언도킹 진실성** — accepted/result/sensor/timeout 계약
5. **MotionRequest/Result 단일 소유권** — legacy target/cancel/hold 제거
6. **trap typed 결과 + 논블로킹 스텝머신**
7. **central rat session timeout·ACK·stale 복구**
8. **배터리 freshness·role/wake eligibility**
9. **DB 절대경로 + typed write ACK**
10. **RGB-aligned depth + observation stamp TF**
11. **rat observation 스로틀·freshness**
12. **capability gate와 herding/webcam/droppings 범위 확정**
13. **실행 문서·의존성·app launch 갱신**
14. **component/launch/install 테스트와 저속 실기 gate**

## 부록 C — 배포 차단 조건

다음 중 하나라도 만족하지 못하면 자동 운용 배포를 차단한다.

- undock action success와 fresh `is_docked=False` 없이 PATROLLING/ROLE 진입 금지
- active motion에 owner/operation/step이 없으면 실기 주행 금지
- trap approach/backup 결과 없이 installed 발행 금지
- rat session에 command ACK와 유한 timeout이 없으면 자동 역할 배정 금지
- 값 205 map 픽셀의 분류와 unknown planning 정책이 검증되지 않으면 자동 순찰 금지
- RGB-depth alignment와 observation-time TF가 검증되지 않으면 map 좌표 기반 접근 금지
- battery UNKNOWN/STALE에서 자동 UNDOCK 금지
- DB commit ACK 없이 저장 성공 event 발행 금지
- capability가 false/stale인 기능에 명령 발행 금지
- pytest/ament lint 또는 필수 component test 실패 상태로 배포 금지
