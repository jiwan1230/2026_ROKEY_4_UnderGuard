# Under-Guard 노드 구조 (스켈레톤) 설계

작성일: 2026-08-05
패키지: `turtle_project` (ROS 2 Humble, ament_python) + `turtle_interfaces` (신규)

## 목적

2대 TurtleBot4 협업 방역 시스템의 **전체 노드 구조를 뼈대로** 세운다. 토픽/
서비스 인터페이스는 실제로 연결되게 만들고(배관 통함), 각 노드의 동작 로직은
`# TODO(팀원)` placeholder + 로그로 둔다. 팀원이 노드를 띄우면 인터페이스가
이미 통해 있어 자기 로직만 채우면 된다.

## 실행 분산 (PC 배치)

- **로봇 PC** (로봇당 1대, robot4/robot6): `camera_node`, `detector_node`,
  `trap_check_node`, `robot_agent` — 카메라 이미지가 네트워크 안 타고 로컬 처리
- **중앙 PC** (이 컴퓨터, 노트북): `central_node`, `db_node`, `rat_herding_node`,
  `webcam_node`

## 범위

- **뼈대로 신규 생성**: camera_node(재작성), detector_node, trap_check_node,
  robot_agent, webcam_node, central_node, db_node, rat_herding_node
- **기존 동작 코드 유지·이동**: opening 접근·depth_spread 검증 로직은
  camera_node → **detector_node로 이동**. depth_math, nav_controller,
  opening_test_node는 그대로.
- 실제 알고리즘(몰이, trap 판정, 순찰 주행)은 전부 placeholder

## 구조 변경 요지

기존 camera_node가 (이미지 수신 + YOLO + opening 검증)을 다 했는데, 분산·역할
분리를 위해 셋으로 쪼갠다:

- **camera_node**: 순수 이미지 파이프. rgb+depth를 stamp로 sync해서 두 토픽으로
  **재발행만**. 판단 없음.
- **detector_node**: sync 이미지 구독 → YOLO 1회로 **opening·rat 둘 다 감지** →
  opening 검증(depth_spread)·DB조회·target_pose·event. (기존 검증 로직 이동)
- **trap_check_node**: sync 이미지 구독 → trap 설치 판단 (placeholder)

로봇 PC에서 YOLO는 detector_node **1개만** 돈다 (모델이 rat·opening·droppings
한 모델).

## 노드 전체 맵

| 노드 | 실행 | 역할 |
|------|------|------|
| `camera_node` | 로봇 PC ×2 | rgb+depth sync → 두 토픽 재발행 (판단 없음) |
| `detector_node` | 로봇 PC ×2 | YOLO opening·rat 감지, opening 검증·DB조회·추적 goal |
| `trap_check_node` | 로봇 PC ×2 | sync 이미지로 trap 설치 판정 (placeholder) |
| `robot_agent` | 로봇 PC ×2 | 주행(Nav2)·dock/undock·배터리·상태보고 |
| `central_node` | 중앙 PC ×1 | 모드·역할 A/B 배정·순찰 교대 |
| `db_node` | 중앙 PC ×1 | 구멍/trap 좌표 기록·조회 (서비스) |
| `rat_herding_node` | 중앙 PC ×1 | 로봇B 몰이 goal 발행 (팀원 알고리즘) |
| `webcam_node` | 중앙 PC ×1 | 고정 웹캠 rat 감지 → homography로 map좌표 |

## 통신 인터페이스

### 카메라 재발행 (로봇 PC 내부, namespace)

camera_node가 원본 카메라 토픽을 sync해서 정렬된 것만 재발행. detector와
trap_check가 이걸 구독 (원본 대신 이걸 써야 rgb·depth가 짝맞음).

- 구독(원본): `{ns}/oakd/rgb/image_raw/compressed`,
  `{ns}/oakd/stereo/image_raw/compressedDepth`, `{ns}/oakd/stereo/camera_info`
- 발행(정렬): `{ns}/synced/rgb` (CompressedImage), `{ns}/synced/depth`
  (CompressedImage) — 같은 header.stamp. camera_info는 그대로 통과 재발행.
- 두 토픽 각각 발행 (합치지 않음). 구독자는 다시 ApproximateTimeSynchronizer로 짝.

### Fleet 상태/명령/이벤트 (std_msgs/String, 콜론 포맷)

커스텀 msg는 만들지 않는다 (ponytail). fleet_msg.py 헬퍼로 파싱/조립 통일.

- `/fleet/status` (String) — robot_agent 발행: `"<robot>:<state>:<battery%>"`
  state ∈ IDLE, PATROLLING, RETURNING, DOCKED, TRACKING, HERDING
- `/fleet/command` (String) — central 발행, robot_agent 구독(자기 것 필터):
  `"<robot>:<command>"`  command ∈ UNDOCK, DOCK, PATROL, TRACK, HERD, STOP
- `/fleet/event` (String) — 감지원(detector/webcam/trap_check)이 central에:
  `"rat_detected:<x>:<y>"`, `"opening_confirmed:<x>:<y>"`, `"trap_ok:<x>:<y>"`

### 목표좌표 (geometry_msgs/PoseStamped)

- `{robot}/target_pose` — detector_node(추적) 또는 rat_herding_node(몰이) 발행,
  robot_agent 구독해 Nav2 이동. 한 로봇에 한 발행자만 활성(central 조율).

### DB 조회 (서비스, 커스텀 srv)

- **신규 패키지 `turtle_interfaces`** (ament_cmake), `srv/QueryHole.srv`:
  ```
  float64 x
  float64 y
  ---
  bool exists
  bool trap_installed
  ```
- 서비스: `/db/query_hole` (`turtle_interfaces/srv/QueryHole`)
- db_node = 서버, detector_node = 클라이언트

## 노드별 인터페이스 상세 (뼈대)

### camera_node (재작성 — 순수 파이프)
- 파라미터: `namespace`
- 구독: 원본 rgb/depth/camera_info (위)
- 발행: `{ns}/synced/rgb`, `{ns}/synced/depth`, `{ns}/synced/camera_info`
- 로직: ApproximateTimeSynchronizer로 rgb+depth 짝 → 같은 stamp로 재발행. **완성**
  (판단이 없어 이 노드는 placeholder가 아니라 실제 동작하게 만든다)

### detector_node (기존 opening 검증 로직 이동 + rat 감지)
- 파라미터: `namespace`, `model_path`, `conf`, `depth_gap`, `side_margin`,
  `approach_dist`, `verify_timeout`
- 구독: `{ns}/synced/rgb`, `{ns}/synced/depth`, `{ns}/synced/camera_info`
- 발행: `/fleet/event`(rat_detected, opening_confirmed), `{ns}/target_pose`
- 서비스 클라이언트: `/db/query_hole`
- 로직: YOLO로 opening·rat 감지. opening → DB조회 → 있으면 trap단계 넘김 /
  없으면 depth_spread 검증(기존 로직 그대로) → 진짜면 trap설치단계(로그)+DB기록.
  rat → target_pose 추적 goal + event. **opening 검증은 기존 동작 유지**,
  rat 감지·DB분기는 뼈대(TODO 채움).

### trap_check_node
- 파라미터: `namespace`
- 구독: `{ns}/synced/rgb`, `{ns}/synced/depth`, 트리거(`/fleet/command` HERD 등)
- 발행: `/fleet/event`(trap_ok:x:y)
- 로직: trap 설치 판정 = TODO(기준 미정). 뼈대는 트리거 수신 → 로그만

### robot_agent (namespace로 2 실행)
- 파라미터: `namespace`, `battery_threshold`(20), `battery_check_period`(10s)
- 구독: `/fleet/command`(자기 것 필터), `{ns}/battery_state`, `{ns}/target_pose`
- 발행: `/fleet/status`
- 액션 클라이언트: `{ns}/dock`, `{ns}/undock`, `{ns}/navigate_to_pose`
- 로직: 배터리 임계 감지 시 RETURNING 발행 — 주행/도킹은 TODO(팀원)

### webcam_node (중앙 PC)
- 파라미터: `camera_index`(0), `homography_file`, `model_path`
- 구독: (없음, cv2.VideoCapture로 웹캠 직접)
- 발행: `/fleet/event`(rat_detected:x:y)
- 로직: YOLO 감지 + homography 변환 = TODO(팀원). 뼈대는 프레임 읽기·발행 배관

### central_node (중앙 PC)
- 구독: `/fleet/status`, `/fleet/event`
- 발행: `/fleet/command`
- 로직: 순찰 교대 시퀀스 + 쥐대응 역할배정 = 뼈대는 상태 추적 골격, 전이는
  순수함수 `next_command()`만 실제 구현 + self_check

### db_node (중앙 PC)
- 서비스 서버: `/db/query_hole`
- 로직: 좌표 저장/조회 = TODO(팀원). 뼈대는 빈 리스트 + 항상 exists=False 응답

### rat_herding_node (중앙 PC)
- 구독: `/fleet/event`(rat 위치)
- 발행: `/robot6/target_pose` (혹은 central이 지정한 로봇B)
- 로직: 몰이 알고리즘 = TODO(팀원). 뼈대는 구독→발행 배관만

## 순찰 교대 시퀀스 (central 상태머신, 뼈대)

1. 로봇A PATROLLING, 배터리 확인
2. 배터리<임계 → 로봇A: RETURNING 발행 + dock 이동·Dock
3. 로봇A DOCKED 발행
4. central: DOCKED 수신 → 로봇B에 UNDOCK 명령
5. 로봇B undock 완료 → central: PATROL 명령(중단지점 근처, placeholder)
6. 로봇A dock에서 충전 대기

전이 판정 순수 함수 `next_command(status)`로 빼서 `_self_check()`.

## 쥐 추적 흐름 (뼈대)

1. 감지원(detector/webcam) → `/fleet/event` rat_detected:x:y
2. central: 쥐대응 모드, 역할배정(PATROLLING인 로봇=A, 나머지=B)
3. 로봇A: detector_node가 `{A}/target_pose`로 쥐 추적 goal 갱신
4. 로봇B: UNDOCK → 구멍/trap 점검 → rat_herding_node가 `{B}/target_pose` 몰이 goal

## 파일 구성

```
turtle_interfaces/            신규 패키지 (ament_cmake)
  srv/QueryHole.srv
  CMakeLists.txt, package.xml

turtle_project/               기존 패키지 (ament_python)
  camera_node.py        재작성 — sync 파이프 (동작)
  detector_node.py      신규 — opening 검증(이동) + rat 감지 뼈대
  trap_check_node.py    신규 뼈대
  robot_agent.py        신규 뼈대
  webcam_node.py        신규 뼈대
  central_node.py       신규 뼈대
  db_node.py            신규 뼈대
  rat_herding_node.py   신규 뼈대
  fleet_msg.py          신규 — String 포맷 파싱/조립 헬퍼 (순수함수)
  depth_math.py         기존
  nav_controller.py     기존
  opening_test_node.py  기존
```

`turtle_project`가 `turtle_interfaces`에 의존 (package.xml `<depend>`).

## 검증

- `fleet_msg.py`: 파싱/조립 round-trip assert
- `central_node.py`: `next_command()` 전이 순수함수 assert
- `camera_node`: sync 파이프는 실동작이지만 ROS 필요 → self_check 제외, 스모크로 확인
- ROS/액션/YOLO/웹캠/하드웨어 경로는 self-check 제외

## setup.py entry_points 추가

`camera_node`(기존), `opening_test_node`(기존) +
`detector_node, trap_check_node, robot_agent, webcam_node, central_node,
db_node, rat_herding_node`

## ponytail 메모

- fleet 통신 커스텀 msg 스킵 → String + fleet_msg 헬퍼. 필드 늘면 그때 msg
- DB만 커스텀 srv → turtle_interfaces (좌표 in + 불리언 out은 srv가 정직)
- YOLO 1개(detector_node)만 로봇 PC에서 — rat 감지를 detector에 흡수(중복 방지)
- 신규 노드 로직 placeholder — 구조가 목표, 동작은 팀원
- opening 검증 로직은 새로 안 짬 — camera_node에서 detector_node로 이동만
- camera_node는 판단 없는 순수 파이프라 실제 동작하게 (뼈대 아님)
