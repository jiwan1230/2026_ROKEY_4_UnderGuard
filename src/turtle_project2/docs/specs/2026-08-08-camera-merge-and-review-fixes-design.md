# 카메라 통합 + 리뷰 지적사항(1-2, 2-1~2-13) 해결 설계

> 대상: 2026-08-08 코드 리뷰에서 확인한 문제들.
> 이 문서는 설계만 담는다 — **실제 코드는 이 문서 작성 시점에 변경하지 않는다.**
> 항목 번호(1-2, 2-1~2-13)는 리뷰 보고서의 번호를 그대로 쓴다.

## 구현 상태 (2026-08-08 갱신)

저위험·독립·짧은 항목부터 반영했다. typed 인터페이스 대전환(2-1/2-4/2-8/2-11/
2-13/15-5)은 실기로 opening 루프를 한 번 완주한 뒤로 미룬다.

**✅ 완료**

| 항목 | 반영 내용 | 남은 꼬리 |
|---|---|---|
| A | detector가 oakd 원본 직접 sync + camera_node 삭제 + setup entry 제거 | **TF를 depth header stamp로 조회**하는 부분 미반영 (최신 TF 사용 — 이동 중 좌표 밀림 가능) |
| 2-5 | `next_patroller` 순수함수로 스테일 patroller 해제 + self-check | status TTL 기반 해제(§1-2 timer)는 1-2와 함께 |
| 2-7 | `_verify`·`_box_to_map` 중심 depth 무효 시 bbox 폴백 | — |
| 2-9 | nav2.yaml `dummy_layer` 삭제 + `trans_stopped_velocity` 0.25→0.05 | inflation_radius는 실기 튜닝값이라 미변경 |
| 2-10 | opening_test CameraInfo 센서 QoS + 토픽 param화 | — |
| 15-1 | best.pt·waypoint·map을 share 설치, `/home/rokey` 하드코딩 3곳 제거 | install-only 자동 테스트 미실시 |
| 15-3 | room_map.yaml `free_thresh` 0.25→0.196 | 실기 OccupancyGrid 덤프로 회색=unknown 확인 남음 |

**🔶 부분 반영** (문서 완성형의 일부만)

| 항목 | 반영된 것 | 문서가 더 요구하는 것 |
|---|---|---|
| 2-3 | `driving` 플래그로 접근/추적 goal 성패 로깅 | `_cancel_all_motion`으로 goal handle **실제 취소**, STOP이 driving 취소, MotionRequest 소유권 |
| 2-4 | dock **센서**로 도킹 판정, 도크 위 PATROL은 언도킹 먼저 | `pending_role`, undock 성공 전 역할상태 금지, FAULT, central `assign_roles` B 자격·단독추적 |

**문서 외 별도 반영** — 실기 디버깅에서 나온 것 (리뷰 항목 아님)

- 접근 goal이 발행 즉시 취소되던 버그 (target_cb가 stop_patrol 먼저 호출)
- 순찰 이어하기 (`lap_from` — opening 처리 후 중단 지점부터 재개)
- OpenCV 디버그 창 (`show:=true`, 상태 오버레이)

**⬜ 미착수** — 1-2, 2-1, 2-2, 2-6, 2-8, 2-11, 2-12, 2-13, 15-2, 15-4, 15-5, 15-6, 15-7

> 위 "실제 코드는 변경하지 않는다"는 최초 작성 시점 기준이며, 위 표대로 일부는
> 이미 반영됐다. 각 절의 본문 설계는 완성형 기준으로 유지한다.

## 0. 범위와 원칙

**다루는 것**

- **A.** camera_node의 sync 작업을 detector_node로 통합 (camera_node 삭제)
- **1-2** central `waking` 영구 대기
- **2-1 ~ 2-13** 리뷰에서 찾은 로직 문제 13건

**함께 다루는 실행 차단 문제**

- 1-1 `/home/rokey` 하드코딩과 패키지 설치 후 사라지는 model/map/waypoint 경로
- lifecycle 활성화 스크립트의 `inactive` 오판과 누락 노드 성공 처리
- map 임계값, Nav2 정지 판정 등 실기기 안전에 직접 영향을 주는 설정
- 상태 freshness, 늦은 비동기 응답, 작업 취소처럼 1-2/2-1~2-13을 실제로
  완결하는 데 필요한 항목

**다루지 않는 것**

- architecture.md / flowchart.md 전면 개편 — 단, 이 설계로 틀려지는
  camera_node·trap 결과·rat 관측 계약은 함께 수정
- 몰이 경로 생성 알고리즘과 webcam 탐지 모델 자체 구현. 다만 두 기능이 구현될
  때 사용할 입력 토픽과 freshness 계약은 이 문서에서 확정한다.

**원칙**

- 변경량보다 실패 시 안전을 우선한다. 순수 함수 + `--check` self-check,
  논블로킹 폴링 tick(robot_agent의 `patrol_tick`/`_dock_tick` 방식), 로봇-로컬
  상대토픽은 유지하되 비동기 요청에는 operation/attempt ID를 붙인다.
- 상태 보고는 사실만 보고한다 (성공 확인 전에 성공 상태 금지 — 기존 dock
  진실성 원칙을 undock에도 적용).
- timeout은 반드시 작업을 **실패 또는 취소**로 끝낸다. timeout을 성공으로
  축약하거나 다음 물리 단계로 진행하지 않는다.
- process watchdog에는 ROS clock 대신 `time.monotonic()`을 쓴다. simulation
  time이 멈춰도 취소·복구가 동작해야 한다.
- 의도적으로 남기는 한계는 `ponytail:` 주석과 실기기 검증 항목으로 명시한다.

---

## A. camera_node를 detector_node로 통합

**현상** — camera_node는 rgb/depth/camera_info 3종을 stamp로 sync해
`synced/*`로 재발행만 하는데, 구독자가 detector_node **하나뿐**이다(grep 전수
확인). trap_check는 카메라를 아예 안 쓴다(camera_node docstring의 "trap_check도
구독" 문구는 낡은 것). 재발행은 DDS 복사·직렬화와 프로세스 하나를 추가한다.
실제 WiFi 대역폭 증가는 DDS 배치와 shared-memory 사용 여부에 따라 달라지므로
"항상 2배"라고 단정하지 않는다.

**설계** — detector가 원본 토픽을 직접 sync한다. sync 설정(queue 10, slop
0.1)과 `synced_cb` 로직은 유지하되, RGB와 depth가 실제로 정렬된 영상인지 시작
시 검증한다. 두 영상의 optical frame이 다르면 해상도 비례 좌표 변환만으로 섞지
말고, OAK-D의 RGB-aligned depth 토픽을 사용하거나 depth camera의 내·외부
파라미터로 RGB 픽셀을 depth 좌표계에 투영한다.

3D point를 map으로 바꿀 때 `rclpy.time.Time()`의 최신 TF를 쓰지 않고 동기화된
depth/RGB header stamp의 TF를 조회한다. 해당 시각 TF가 timeout 안에 없으면 그
프레임의 좌표 발행을 건너뛰며 최신 자세로 대체하지 않는다. 이동 중 최신 TF를
쓰면 촬영 시점과 변환 시점의 차이만큼 구멍·쥐 좌표가 체계적으로 밀린다.

`detector_node.py` 변경:

```python
# 파일 상단에 camera_node.py의 토픽 상수 이동 (상대토픽 — __ns가 접두)
RGB_IN = 'oakd/rgb/image_raw/compressed'
DEPTH_IN = 'oakd/stereo/image_raw/compressedDepth'
INFO_IN = 'oakd/stereo/camera_info'

# __init__ 의 구독 3줄에서 토픽명만 교체 ('synced/rgb' → RGB_IN 등)
self.rgb_sub = Subscriber(self, CompressedImage, RGB_IN,
                          qos_profile=qos_profile_sensor_data)
self.depth_sub = Subscriber(self, CompressedImage, DEPTH_IN,
                            qos_profile=qos_profile_sensor_data)
self.info_sub = Subscriber(self, CameraInfo, INFO_IN,
                           qos_profile=qos_profile_sensor_data)
# ApproximateTimeSynchronizer(queue_size=10, slop=0.1) — 무변경
```

정리 작업:

| 파일 | 변경 |
|---|---|
| `turtle_project/camera_node.py` | 삭제 |
| `setup.py` | `camera_node = ...` entry_point 한 줄 삭제 |
| `docs/run.md` | "1) 카메라 sync" 단계 삭제, 번호 재정렬 |
| `detector_node.py` docstring | "synced/rgb·synced/depth(camera_node 발행)를 구독" → "oakd 원본 rgb·depth·camera_info를 직접 sync" |

- 나중에 다른 노드가 sync된 영상이 필요해지면 그때 camera_node를 git 히스토리
  에서 되살리면 된다 (YAGNI).
- opening_test_node는 원래부터 원본 토픽 직접 구독이라 무관.

**검증** — 실기기에서 detector 시작 후 SEARCHING 상태에서 감지 로그가 도는지,
RGB 박스 중심을 depth 영상에 투영한 지점이 같은 물체를 가리키는지 확인한다.
해상도가 다른 녹화 프레임과 frame mismatch를 self-check에 넣어 단순 프레임
수신만으로 정합 성공을 판정하지 않는다.

---

## 1-2. central `waking` 영구 대기 해제

**현상** — `waking`은 깨운 로봇이 `PATROLLING`을 보고할 때만 풀린다
(central_node.py:81-82). 순찰 시작이 실패(waypoint 파일 없음 등)하면 로봇은
IDLE로 남고 `waking`은 영원히 안 풀려 아무도 순찰하지 않는 채 멈춘다.

**설계** — status 수신에 의존하지 않는 1초 coordination timer를 두고, 기상
timeout·status freshness·재선정 cooldown을 함께 처리한다.

1. **조기 실패 감지**: 깨운 로봇이 `IDLE`/`FAULT`를 보고하면 즉시 실패 처리한다.
2. **독립 timeout**: `wake_timeout_sec`(기본 60초)이 지나면 status가 한 건도
   들어오지 않아도 timer가 해제한다.
3. **stale 제외**: 10초 주기 status를 기준으로 `status_ttl_sec` 기본 25초를 넘긴
   로봇은 coverage/역할 후보에서 제외한다.
4. **고장 로봇 연속 재선정 방지**: 실패한 로봇은 `wake_retry_cooldown_sec` 기본
   120초 동안 기상 후보에서 빼고 다른 정상 후보를 먼저 고른다.

```python
import time

# __init__
self.wake_timeout = self.declare_parameter('wake_timeout_sec', 60.0).value
self.status_ttl = self.declare_parameter('status_ttl_sec', 25.0).value
self.wake_retry_cooldown = self.declare_parameter(
    'wake_retry_cooldown_sec', 120.0).value
self.waking_at = 0.0
self.last_status_at = {}
self.wake_blocked_until = {}
self.create_timer(1.0, self.coordination_tick)

def _now(self):
    return time.monotonic()             # use_sim_time 정지와 무관한 process watchdog

def _start_waking(self, robot):
    self.send(robot, 'UNDOCK')
    self.waking = robot
    self.waking_at = self._now()

def _fail_waking(self, reason):
    failed = self.waking
    if failed is None:
        return
    self.wake_blocked_until[failed] = self._now() + self.wake_retry_cooldown
    self.waking = None
    self.get_logger().warn(f'{failed} 기상 실패({reason}) — cooldown 후 재시도')

def coordination_tick(self):
    now = self._now()
    if self.waking and now - self.waking_at > self.wake_timeout:
        self._fail_waking('timeout')
    self._ensure_coverage(now)           # status가 끊겨도 timer가 복구를 계속 시도

# status_cb parse 성공 직후:
now = self._now()
self.last_status_at[robot] = now
...
if state == 'PATROLLING' and self.waking == robot:
    self.waking = None
elif robot == self.waking and state in ('IDLE', 'FAULT'):
    self._fail_waking(state)
```

`coverage_action`과 `assign_roles`에는 `now`, `last_status_at`,
`wake_blocked_until`을 전달한다. `now - last_seen <= status_ttl`이고 cooldown이
끝난 로봇만 후보가 된다. 두 로봇 모두 stale/cooldown이면 명령을 추측해서 보내지
않고 경고만 남긴다. 새 status가 들어오면 다음 timer tick에서 자동 복구한다.

**검증** — central에 `_FakeCentral` self-check 스텁을 신설한다:

```python
# 깨운 로봇이 IDLE 보고(기상 실패) → waking 해제 → 재선정 가능
# waking 후 status가 한 건도 없이 wake_timeout 경과 → timer만으로 waking 해제
# 실패한 robot4가 cooldown이면 fresh한 robot6을 선택
# 25초 넘은 stale DOCKED/PATROLLING 로봇은 coverage/역할 후보에서 제외
# 정상 기상(PATROLLING 보고) → waking 해제 (기존 동작 유지)
```

---

## 2-1. trap 이벤트 로봇 교차 수신 차단

**현상** — trap_check가 전역 `/fleet/event`로 `trap_installed/trap_ok/trap_bad`
를 쏘고, detector는 자기 상태가 AWAIT_TRAP인지만 본다. 두 로봇이 동시에 각자
구멍을 처리하면 서로의 결과로 잘못 전이한다.

**설계** — **상태 전이용 응답은 로봇-로컬 typed 토픽으로 분리**하고,
`operation_id + attempt_id`가 모두 일치할 때만 받아들인다. 네임스페이스는 로봇
간 격리, ID는 같은 로봇에서 이전 작업의 늦은 응답을 격리한다.

인터페이스 변경:

```text
# turtle_interfaces/msg/TrapJob.msg
uint64 operation_id
uint32 attempt_id
string phase          # install | inspect | cancel
float64 hole_x
float64 hole_y
float64 trap_x
float64 trap_y

# turtle_interfaces/msg/TrapResult.msg
uint64 operation_id
uint32 attempt_id
string outcome        # installed | ok | bad | failed | canceled | busy
float64 hole_x
float64 hole_y
string detail
```

detector는 opening을 시작할 때 process 내에서 단조 증가하는 `operation_id`를
부여하고, 최초 설치/점검은 attempt 0, 재설치마다 attempt를 1 증가시킨다. 노드
재시작 시의 충돌을 피하려면 시작 시 random 32-bit prefix를 만들고
`(prefix << 32) | counter`로 uint64 ID를 만든다. counter overflow 전에는 prefix를
다시 만들지 않는다. ID는 DB의 영속 식별자가 아니라 실행 중 상관관계 전용이다.

`trap_check_node.py`:

```python
# __init__
self.result_pub = self.create_publisher(TrapResult, 'trap_result', 10)

def _publish_result(self, job, outcome, detail=''):
    result = TrapResult(operation_id=job.operation_id,
                        attempt_id=job.attempt_id,
                        outcome=outcome,
                        hole_x=job.hole_x, hole_y=job.hole_y,
                        detail=detail)
    self.result_pub.publish(result)       # 상태 전이용: 로컬 + ID 일치 필수

# 전역 /fleet/event는 detector가 현재 ID의 성공 결과를 수락한 뒤 관찰/DB용으로
# 한 번만 발행한다. trap_check가 전역 결과를 먼저 발행하지 않는다.
```

`detector_node.py`:

```python
self.create_subscription(TrapResult, 'trap_result', self.trap_result_cb, 10)

def trap_result_cb(self, msg):
    if self.state not in ('AWAIT_INSTALL', 'AWAIT_INSPECT'):
        return
    if (msg.operation_id, msg.attempt_id) != \
            (self.operation_id, self.attempt_id):
        self.get_logger().warn('stale trap 결과 무시')
        return
    # outcome별 전이. failed/canceled/busy는 성공으로 취급하지 않는다.
```

central/UI는 detector가 확정해 발행하는 기존 `/fleet/event`를 관찰용으로 계속
받는다. DB 저장은 §15-5의 typed service ACK로 바꾼다. `cancel` job도 같은
operation ID를 사용하며, trap_check는 현재 job과 ID가 맞을 때만 중단한다. 새
install이 진행 중일 때 다른 install/inspect가 오면 기존 job을 덮어쓰지 않고
`busy`로 응답한다.

**검증**:

- robot4·robot6이 동시에 대기할 때 결과가 네임스페이스를 넘지 않는다.
- 같은 robot4에서 op=10 timeout 후 op=11을 시작하고 op=10 결과를 늦게 넣어도
  op=11 상태가 바뀌지 않는다.
- op는 같고 attempt가 이전인 재설치 결과도 무시한다.
- busy/canceled/failed 결과가 `trap_installed` 전역 이벤트로 변환되지 않는다.

---

## 2-2. 언도킹 성공 확인

**현상** — `_dock_tick`의 UNDOCK 분기는 "액션 끝남"만 보고 성공을 확인하지
않는다. 언도킹이 실패해 도크에 붙어 있어도 순찰을 시작해 PATROLLING을 보고한다.
도킹 분기는 이미 `is_docked` 센서로 확인하고 있으므로 대칭을 맞춘다.

**설계** — `isUndockComplete()` 하나를 성공으로 해석하지 않는다. Navigator가
undock goal handle·result future를 보관하고 `SUCCEEDED/CANCELED/FAILED`를
정규화해 반환한다. robot_agent는 다음 세 조건이 모두 참일 때만 성공으로 본다.

1. undock action 결과가 `SUCCEEDED`
2. `dock_status_ttl_sec`(기본 3초) 안에 받은 신선한 센서 값이 있음
3. 신선한 `is_docked`가 명시적으로 `False`

`dock_action_timeout_sec`(기본 45초)을 넘기면 goal을 취소한다. 센서 미수신,
action 실패·거부·timeout은 `FAULT`로, 센서가 신선하고 여전히 도킹이면
`DOCKED`로 보고한다. `FAULT`는 실제 위치를 모르는 상태이며 central은 자동 기상
후보에서 제외한다. 도킹 쪽도 같은 result/timeout/freshness 계약을 적용한다.

`robot_agent.py` `_dock_tick` 개념 코드:

```python
elif self.dock_phase == 'UNDOCK':
    if self._dock_action_timed_out():
        self.nav.cancelUndock()
        self._finish_dock_phase('FAULT', '언도킹 timeout')
        return
    result = self.nav.undock_result()
    if result is None:
        return
    if result != 'SUCCEEDED':
        self._finish_dock_phase('FAULT', f'언도킹 action {result}')
        return
    docked = self.nav.fresh_dock_state(self.dock_status_ttl)
    if docked is True:
        self._finish_dock_phase('DOCKED', '언도킹 후에도 dock sensor=True')
        return
    if docked is None:
        self._finish_dock_phase('FAULT', 'dock sensor stale/unknown')
        return
    self.dock_phase = None              # docked is False가 확인된 유일한 성공 경로
    self._run_after_undock()
```

`_finish_dock_phase`는 `dock_phase`, `after_undock`, pending role을 정리하고 상태를
한 번만 바꾼다. `DOCKED`/`FAULT` 전환을 새 순찰 성공으로 간주하지 않으며,
central의 §1-2 cooldown을 통해 다른 fresh 로봇을 먼저 고른다.

**주의 — 기존 self-check 수정 필요**: 성공 케이스는 action result
`SUCCEEDED`와 fresh `docked=False`를 모두 넣어야 한다. 단순히 FakeNav의 bool
기본값만 바꾸면 센서 stale 경로를 검증하지 못한다.

**검증** — self-check 추가:

```python
# action success + fresh docked=False만 PATROL 시작
# action failed/canceled/rejected → FAULT, PATROL 금지
# action success + fresh docked=True → DOCKED, PATROL 금지
# action success + dock sensor stale/unknown → FAULT, PATROL 금지
# 45초 timeout → undock goal cancel 정확히 1회 + FAULT
```

---

## 2-3. STOP이 목표 주행(driving)을 취소하게

**현상** — STOP은 `stop_patrol()`만 불러서, `target_pose`로 받은 goal 주행
중이면 취소가 안 되고 상태만 IDLE이 된다.

**설계** — 모든 이동 소유권을 한 곳에서 정리하는 `_cancel_all_motion()`을 만들고
STOP, 새 docking/undocking 시작, 상위 역할 전환에서 재사용한다. bool만 지우지
말고 실제 goal handle에 cancel을 요청한다.

```python
elif cmd == 'STOP':
    self._cancel_all_motion(reason='STOP')
    self.pending_role = None
    self.after_undock = None
    self.state = 'IDLE'

def _cancel_all_motion(self, reason):
    # followWaypoints/goToPose(NAV 포함)
    if self.patrolling or self.driving or self.dock_phase == 'NAV':
        self.nav.cancelTask()
    # dock/undock은 별도 action client goal handle을 취소
    if self.dock_phase == 'DOCK':
        self.nav.cancelDock()
    elif self.dock_phase == 'UNDOCK':
        self.nav.cancelUndock()
    self.patrolling = False
    self.driving = False
    self.dock_phase = None
    self.active_motion = None
```

`start_docking()`·`start_undocking()`도 새 action을 보내기 전에
`_cancel_all_motion()`을 호출한다. cancellation future가 완료되기 전 새 goal을
보내지 않도록 `CANCELING → 다음 단계`의 짧은 tick 단계를 둔다. 이는 늦게 도착한
이전 goal이 새 goal을 다시 preempt하는 경합을 막는다.

`target_pose`는 §2-8의 `MotionRequest(owner, operation_id, step_id)`로 교체하고,
robot_agent는 현재 owner/ID와 취소된 ID를 기록한다. 취소된 operation의 늦은
request는 무시한다.

**검증** — self-check 추가:

```python
# STOP: patrol / goToPose / NAV / DOCK / UNDOCK 각 단계에서 해당 action cancel
# cancel 완료 전에 후속 goal이 발행되지 않음
# STOP 뒤 도착한 canceled operation의 MotionRequest는 무시
# 상태·dock_phase·after_undock·pending_role이 모두 정리됨
```

---

## 2-4. 도킹 진행 중 TRACK/HERD 처리 (+ central의 B 후보 자격)

**현상** — 도크앞 이동(NAV)·정밀 도킹(DOCK) 중 TRACK/HERD가 오면 상태만
TRACKING/HERDING으로 바뀌고 도킹은 계속 진행, target_pose는 전부 무시된다.
결국 도킹 완료가 상태를 덮어써 central이 오판한다.

**설계 (agent 측)** — TRACK/HERD를 받았다는 사실과 실제 역할 수행 상태를
분리한다. 도크에 있거나 도킹 중이면 `pending_role`에 보관하고, 실제 언도킹
성공 전에는 TRACKING/HERDING을 보고하지 않는다. NAV/DOCK 중 명령도 조용히
버리지 않고 도킹 완료 → 언도킹 → 역할 시작 순서로 실행한다.

```python
# command_cb — 기존 사전 매핑에서 TRACK/HERD 제거
self.state = {'UNDOCK': 'UNDOCKING', 'STOP': 'IDLE'}.get(cmd, self.state)
...
elif cmd in ('TRACK', 'HERD'):
    self.stop_patrol()
    if was_docked or self.dock_phase is not None:
        self.pending_role = cmd
        if self.dock_phase is None:     # 현재 DOCKED
            self.state = 'UNDOCKING'
            self.after_undock = 'ROLE'
            self.start_undocking()
        # NAV/DOCK이면 완료 후 _dock_tick이 pending_role을 보고 undock 시작
        return
    self._enter_role(cmd)               # 이미 필드 위에서만 역할 상태 보고

def _enter_role(self, cmd):
    self.pending_role = None
    self.state = 'TRACKING' if cmd == 'TRACK' else 'HERDING'

# undock 성공(docked=False 확인) 후:
if self.after_undock == 'ROLE' and self.pending_role:
    self._enter_role(self.pending_role)
```

언도킹 실패 시 §2-2에 따라 pending role을 버리고 DOCKED/FAULT를 보고한다.

**설계 (central 측 — 애초에 그런 로봇에 역할을 안 주기)** — `assign_roles`가
B 후보 자격·status freshness·배터리를 함께 거른다. `IDLE`을 우선하고 그다음
`DOCKED`를 선택한다. stale, 저전압/배터리 unknown, FAULT/RETURNING/다른 역할
수행 중 로봇은 제외한다. 후보가 없으면 **A 단독 추적** `(A, None)`을 돌려준다.

```python
def assign_roles(patroller, robots, last_seen, now, status_ttl, battery_threshold):
    """쥐 감지 시 역할 배정 -> (robot_a, robot_b|None). patroller 없으면 None.

    A = 순찰 중이던 로봇. B = 나머지 중 DOCKED/IDLE인 첫 로봇 (복귀·도킹·역할
    수행 중인 로봇은 제외). B 후보가 없으면 (A, None) — A 단독 추적.
    """
    if patroller is None or patroller not in robots:
        return None
    eligible = [(0 if state == 'IDLE' else 1, robot)
                for robot, (state, battery) in robots.items()
                if robot != patroller
                and state in ('IDLE', 'DOCKED')
                and now - last_seen.get(robot, float('-inf')) <= status_ttl
                and battery is not None and battery > battery_threshold]
    eligible.sort()
    return patroller, (eligible[0][1] if eligible else None)
```

`_on_rat`/`_end_rat`는 `robot_b`가 None일 수 있게 가드:

```python
# _on_rat
robot_a, robot_b = roles
self.rat_mode, self.rat_roles = True, roles
self.send(robot_a, 'TRACK')
if robot_b:
    self.send(robot_b, 'HERD')
else:
    self.get_logger().warn('B 후보 없음 (DOCKED/IDLE 로봇 없음) — A 단독 추적')

# _end_rat
self.send(robot_a, 'PATROL')
if robot_b:
    self.send(robot_b, 'DOCK')
```

**명령 전달 계약** — 역할을 골랐다는 것과 로봇이 명령을 받았다는 것도 분리한다.
`/fleet/command` String은 아래 typed message로 교체한다.

```text
# turtle_interfaces/msg/FleetCommand.msg
uint64 command_id
string robot
string command

# turtle_interfaces/msg/CommandAck.msg
uint64 command_id
string robot
string command
string outcome       # accepted | rejected
string detail
```

central은 process random prefix + counter로 command ID를 만들고, ACK가
`command_ack_timeout_sec` 기본 2초 안에 없으면 **같은 ID**로 최대 3회 재전송한다.
robot_agent는 최근 처리 ID와 ACK를 bounded LRU(예: 128개)로 보관해 중복 명령을
다시 실행하지 않고 같은 ACK만 재발행한다. 상태가 바뀌면 10초 주기를 기다리지
않고 status도 즉시 한 번
발행한다. central은 처음에 `rat_mode=STARTING`으로 두고 A의 TRACK accepted ACK가
온 뒤에만 ACTIVE로 확정한다. A가 reject/ACK timeout이면 역할을 모두 취소하고
다음 `rat_detected`에서 재시도한다. B의 HERD가 reject/ACK timeout이면
`rat_roles`를 `(A, None)`으로 바꿔 종료 때 실패한 B에 DOCK을 보내지 않는다.
안전상 STOP은 ACK 실패를 별도 경고한다.

detector도 같은 typed command를 구독해 §2-8의 opening 취소와 tracking 전환에
사용한다. command 자체의 대상·enum은 message 수신부에서 다시 검증한다.

**주의 — 기존 self-check 수정 필요**: 반환 계약이 바뀐다.

- `assign_roles('robot4', {'robot4': (...)})` : 기존 `None` → **`('robot4', None)`**
  (로봇 1대여도 단독 추적)
- 새 케이스: B 후보가 `RETURNING`이면 `('robot4', None)`

**검증** — self-check 추가 (agent):

```python
# DOCKED에서 HERD → 상태는 UNDOCKING, 성공 전 HERDING 금지
# NAV/DOCK 중 TRACK → pending_role에 보존 → dock 성공 뒤 undock 시작
# undock 성공 + fresh docked=False → 그때 TRACKING/HERDING
# undock 실패/센서 stale → pending_role 삭제, 역할 상태 보고 금지
```

self-check 추가 (central 순수함수):

```python
now, seen = 100.0, {'robot4': 100.0, 'robot6': 100.0}
assert assign_roles('robot4',
                    {'robot4': ('PATROLLING', 80),
                     'robot6': ('RETURNING', 20)},
                    seen, now, 25.0, 30) == ('robot4', None)
assert assign_roles('robot4', {'robot4': ('PATROLLING', 80)},
                    seen, now, 25.0, 30) == ('robot4', None)
# stale DOCKED와 저전압 IDLE은 B에서 제외
# fresh IDLE과 fresh DOCKED가 동시에 있으면 IDLE 우선
# 같은 command_id 재전송은 action을 다시 시작하지 않고 ACK만 재발행
# B HERD rejected/ACK timeout → rat_roles == (A, None)
```

---

## 2-5. `patroller` 스테일 해제

**현상** — `patroller`는 PATROLLING 보고 때만 갱신되고 지워지지 않는다. A가
배터리 부족으로 복귀 중일 때 쥐가 감지되면 복귀 중인 A에게 TRACK이 배정된다.

**설계** — `status_cb`에서 그 로봇이 순찰이 아니게 되면 해제한다. 1-2의 waking
해제와 같은 블록을 고치므로, **병합된 최종 형태**를 기준으로 한다:

```python
# status_cb — parse 성공·last_status_at 갱신 다음, rat_mode 검사 앞. 최종 형태:
if state == 'PATROLLING':
    self.patroller = robot
    if self.waking == robot:
        self.waking = None          # 정상 기상 완료 (기존 동작)
else:
    if self.patroller == robot:
        self.patroller = None       # 순찰 중단(RETURNING/DOCKED 등) — A 후보 제외
    if self.waking == robot and state in ('IDLE', 'FAULT'):
        self._fail_waking(state)    # cooldown까지 함께 설정 (§1-2)
```

`coordination_tick`에서도 현재 patroller의 status가 `status_ttl_sec`를 넘기면
`patroller=None`으로 지운다. 마지막으로 받은 문자열 상태가 PATROLLING이라는 이유로
죽은 프로세스를 계속 순찰자로 취급하지 않는다.

결과: 순찰자가 없는 순간의 쥐 감지는 "역할 배정 불가" 경고로 보수적으로
넘어간다. `rat_detected`는 (2-13 스로틀 후에도) 계속 오므로, 다음 순찰자가
생기면 그때 대응이 시작된다. rat_mode 중 A가 TRACKING을 보고해 patroller가
비는 것은 무해하다 — 종료 명령은 `rat_roles`로 기억한 로봇에 보낸다.

**검증** — `_FakeCentral` 시나리오: PATROLLING 보고 후 RETURNING 보고 또는 status
TTL 초과 → `patroller is None`.

---

## 2-6. `_db_done` 예외 방어

**현상** — `fut.result()`는 서비스가 죽으면 예외를 던진다. 콜백 안에서 터지면
QUERYING에 5초 워치독까지 갇힌다(워치독이 구제는 함).

**설계** — 예외 처리와 operation 상관관계를 함께 넣는다. 요청할 때 현재
`operation_id`를 callback에 캡처하고, 완료 시 여전히 같은 operation의 QUERYING
상태인지 먼저 확인한다. 이미 fleet 명령/timeout으로 취소된 요청의 늦은 응답은
아무 상태도 바꾸지 않는다.

```python
op_id = self.operation_id
future.add_done_callback(lambda fut, op=op_id: self._db_done(fut, op))

def _db_done(self, fut, op_id):
    if self.state != 'QUERYING' or op_id != self.operation_id:
        self.get_logger().debug(f'stale DB 응답 무시: op={op_id}')
        return
    try:
        resp = fut.result()
    except Exception as e:
        self.get_logger().warn(f'db 응답 실패 ({e}) — 처음 본 셈 치고 검증')
        self._enter_verify()
        return
    # (이하 기존 exists 분기 그대로)
```

QUERYING timeout/`_abort_opening`에서는 가능한 경우 future를 cancel하고,
성공 여부와 관계없이 `operation_id`를 무효화한다. SQLite 서비스는 이미 실행을
끝냈을 수 있으므로 callback의 ID 가드가 최종 방어선이다.

**검증** — op=10 DB 요청 후 opening을 취소하고 op=11을 시작한 다음 op=10
future를 완료시켜도 op=11이 VERIFYING으로 바뀌지 않는 self-check를 추가한다.
예외가 현재 op에서 발생했을 때만 `_enter_verify()`가 호출되는 경우도 검증한다.

---

## 2-7. `_verify` 중심 depth 무효 시 폴백

**현상** — 검증 단계에서 박스 중심 depth(z)가 무효면 `side_px`가 0이라 옆 벽
확장이 안 된다. "구멍 안쪽은 depth 무효가 정상"이라는 전제와 모순 — 박스가
구멍에 딱 맞으면 진짜 구멍을 못 알아본다. `_box_to_map`에는 이미 패치 확장
폴백이 있는데 `_verify`에만 없다.

**설계** — `_box_to_map`과 동일한 관용구를 한 줄 적용:

```python
# _verify — 기존:
# z = depth_at(depth, (du1 + du2) // 2, (dv1 + dv2) // 2)
cu, cv = (du1 + du2) // 2, (dv1 + dv2) // 2
z = depth_at(depth, cu, cv) or depth_at(
    depth, cu, cv,
    patch=min(self.max_depth_patch_px,
              max(du2 - du1, dv2 - dv1) // 2))  # 중심 무효 → 제한된 테두리 탐색
if z is None or not math.isfinite(z) or z <= 0 or \
        self.K is None or self.K[0, 0] <= 0:
    self._verify_miss('유효 depth/K 없음')
    return
side = side_px(self.side_margin, z, self.K[0, 0])
```

박스 전체 반경을 무제한으로 확장하면 인접 물체의 depth를 구멍 벽으로 오인할 수
있으므로 `max_depth_patch_px`를 두고 실측 영상으로 정한다.

**검증** — depth_math의 `holey` 케이스 외에 patch 밖의 가까운 물체를 선택하지
않는 경우와 K=0/NaN depth를 안전하게 거부하는 self-check를 추가한다.

---

## 2-8. fleet 명령 수신 시 opening 처리 중단

**현상** — opening 처리(APPROACHING~AWAIT_TRAP) 중 TRACK을 받으면 opening
상태기계와 추적이 동시에 돌아 `target_pose`에서 접근 goal과 추적 goal이 서로
밀어낸다. HERD도 마찬가지.

**설계** — **central의 fleet 명령 = 상위 의사결정**으로 보고, 내 로봇 대상
명령이 오면 detector FSM뿐 아니라 trap job과 그 operation이 소유한 실제 주행
goal까지 함께 취소한다. 구멍은 다음 순찰에서 재발견할 수 있다.

`PoseStamped target_pose`만으로는 goal의 소유자와 취소할 작업을 구분할 수 없으므로
로봇-로컬 motion 계약을 typed message로 바꾼다.

```text
# turtle_interfaces/msg/MotionRequest.msg
string owner             # opening | trap | rat | dock
uint64 operation_id
uint32 step_id
string kind              # pose | backup
geometry_msgs/PoseStamped pose
float32 distance         # backup에서만 사용
float32 speed
float32 timeout_sec

# turtle_interfaces/msg/MotionCancel.msg
string owner
uint64 operation_id

# turtle_interfaces/msg/MotionResult.msg
string owner
uint64 operation_id
uint32 step_id
string outcome           # succeeded | failed | canceled | rejected | timeout
string detail
```

robot_agent만 Nav2 goal/action handle을 소유한다. `MotionRequest`를 받으면 현재
`(owner, operation_id, step_id)`를 저장하고 끝날 때 `MotionResult`를 발행한다.
owner별 마지막 canceled ID도 저장해 그 이하 ID의 늦은 요청을 거부한다.

detector에는 `_abort_opening(reason)` 단일 종료 함수를 둔다.

`detector_node.py` `command_cb` — 로봇 필터 통과 직후에 추가:

```python
def command_cb(self, msg):
    if msg.robot != self.robot_id or msg.command not in VALID_COMMANDS:
        return
    cmd = msg.command
    if self.state != 'SEARCHING':       # opening 처리 중 fleet 명령 = 상위 우선
        self._abort_opening(f'fleet command {cmd}')
    # (이하 기존 TRACK/tracking 분기 그대로)

def _abort_opening(self, reason):
    op = self.operation_id
    if op is None:
        return
    self.job_pub.publish(make_cancel_job(op))
    self.motion_cancel_pub.publish(MotionCancel(owner='opening', operation_id=op))
    self.operation_id = None            # 이후 DB/trap/motion callback 모두 stale
    self._hold_patrol(False)
    self._reset()
```

trap_check는 matching cancel을 받으면 스텝머신을 지우고 아직 진행 중인
`MotionRequest`에도 cancel을 발행한다. robot_agent는 TRACK/HERD/DOCK/STOP 같은
상위 명령을 처리할 때도 opening/trap owner의 motion을 먼저 취소한다. 따라서
detector와 robot_agent의 `/fleet/command` callback 순서가 달라도 canceled ID의
늦은 goal이 새 rat goal을 덮어쓰지 못한다.

hold 해제 시 robot_agent가 곧바로 순찰을 재개하는 경합을 막기 위해
`patrol_hold=False`만으로 자동 재개하지 않는다. hold 해제는 "이동 가능"일 뿐이며,
PATROL 명령이나 이미 유지 중인 PATROLLING state에서만 순찰 goal을 재발행한다.

**검증**:

- APPROACHING/QUERYING/AWAIT_INSTALL/INSPECTING 각각에서 TRACK을 넣어 matching
  DB future, trap job, motion goal이 취소되고 tracking만 남는지 확인한다.
- cancel 직후 이전 operation의 retreat request/result를 지연 주입해도 무시한다.
- detector/robot_agent의 command callback 순서를 반대로 실행하는 테스트 모두
  최종 active motion이 rat owner 하나인지 검증한다.

---

## 2-9. nav2.yaml dummy_layer 삭제

**현상** — `dummy_layer`는 `/robot4/dummy_cloud`를 구독하는데 이 리포지토리에
그 토픽을 발행하는 코드가 없다(전수 grep). 같은 yaml을 robot6에도 쓰므로
robot6 코스트맵이 남의 토픽을 구독하는 셈. mini_turtle4에서 온 잔재.

**설계** — 발행자가 없는 dummy layer는 삭제한다. 단, 이 파일에는 속도·costmap
크기·inflation 등 다른 project 조정값이 있으므로 stock과 동일하다고 쓰지 않는다.
함께 확인된 내부 불일치도 정리한다.

```yaml
# local_costmap:
plugins: ["static_layer", "voxel_layer", "inflation_layer"]   # dummy_layer 제거
# dummy_layer: 블록 전체 삭제

# DWB: 선속도 상한과 합성 속도/정지 판정의 관계를 일관되게
max_vel_x: 0.20
max_speed_xy: 0.20
trans_stopped_velocity: 0.05   # max_vel_x보다 작아야 실제 정지를 판정 가능

# robot_radius 0.175m에 여유가 7.5cm뿐이던 값을 stock에 가까운 안전값으로 시작
inflation_radius: 0.45         # 최종값은 실기기 좁은 통로 시험으로 조정
```

파일 머리 주석은 "turtlebot4_navigation 기반 project 조정본"으로 수정하고 변경한
키와 기준 upstream 버전을 기록한다. "원본 복사본, 변경 없음"이라고 쓰지 않는다.
나중에 YOLO 장애물 주입이 실제로 필요해지면 그때 재도입 — 그 경우 코스트맵이
상대토픽에 자기 하위 ns를 접두하는 문제(기존 주석 내용)는 git 히스토리와 이
문서에 남는다.

**검증** — yaml 정적 검사로 `0 <= trans_stopped_velocity < max_vel_x <=
max_speed_xy`를 확인한다. `ros2 launch` 후 lifecycle **각 대상 노드가 실제
`active`인지** 확인하고, 좁은 통로·정지·회전·trap 접근을 저속 실기기 시험한다.

---

## 2-10. opening_test_node CameraInfo QoS 정합

**현상** — 다른 노드는 전부 센서 QoS인데 여기만 기본(RELIABLE) 구독이라,
카메라가 best-effort로 발행하면 fx를 영영 못 받아 옆 벽 확장이 항상 0.

**설계** — QoS를 교체하고 CameraInfo 토픽도 RGB/depth처럼 parameter로 만든다.
robot4 절대토픽을 기본값으로 박지 않아 다른 namespace의 K를 잘못 받지 않게 한다.

```python
self.info_topic = self.declare_parameter(
    'camera_info_topic', 'oakd/stereo/camera_info').value
self.create_subscription(CameraInfo, self.info_topic, self.info_cb,
                         qos_profile_sensor_data)
```

**검증** — robot4/robot6 namespace에서 실제 연결 publisher가 각각 하나인지,
CameraInfo frame/K가 depth 영상과 일치하는지 확인한 뒤 화면의 주황 확장 박스 폭을
확인한다.

---

## 2-11. trap_check 설치 동작을 논블로킹 스텝머신으로

**현상** — `_install`이 콜백 안에서 `time.sleep` 합계 ~7초 블로킹:
① 그동안 TF 콜백이 멈춰 후퇴점 계산에 낡은 로봇 위치를 쓴다.
② `install_wait` 4초는 접근점 도착 전일 수 있어 후퇴 goal이 접근 goal을 도중에
밀어낸다(도착 확인 없음).
③ Nav2는 후진 불가(`min_vel_x: 0`)라 "후진"이 실제로는 트랩 바로 옆 제자리
180도 회전 — 갓 놓은 트랩을 칠 수 있다.

**설계** — robot_agent의 `_dock_tick`과 같은 **타이머 폴링 스텝머신**으로
바꾸되, TF 거리만 보고 성공을 추측하지 않는다. §2-8의 `MotionResult`가
`succeeded`이고 최종 TF도 tolerance 안일 때만 다음 단계로 진행한다.

파라미터 (기본값 변경/신설):

| 파라미터 | 값 | 의미 |
|---|---|---|
| `approach_dist` | 0.20 → **0.35** | 접근 거리. 로봇 반경 0.175m + 회전 시 트랩과 이격 확보 |
| `install_wait` | 4.0 → **8.0** | **도착 후** 사람이 trap을 놓을 대기 시간 (의미 변경: 이동 시간 미포함) |
| `approach_timeout` (신설) | **30.0** | 접근 action 한도. 초과 시 cancel + failed |
| `backup_dist` (신설) | **0.25** | trap을 향한 자세를 유지한 실제 직선 후진 거리 |
| `backup_speed` (신설) | **0.05** | Nav2 BackUp 동작 속도(m/s), 실기기 저속 시작값 |
| `backup_timeout` (신설) | **10.0** | 후진 action 한도. 초과 시 cancel + failed |
| `arrive_tol` (신설) | **0.10** | 접근 목표 최종 확인 반경 |
| `result_margin` (신설) | **5.0** | timer/DDS 지연 여유 |

detector의 대기 상태를 `AWAIT_INSTALL`과 `AWAIT_INSPECT`로 분리한다.

```python
DEADLINES = {
    # 기존 항목 생략
    'AWAIT_INSTALL': 30.0 + 8.0 + 10.0 + 5.0,  # 53초, 아래 파라미터로 계산
    'AWAIT_INSPECT': 5.0,
}
```

하드코딩된 53초를 두 군데 복제하지 않는다. detector와 trap_check가 같은 launch
argument/공통 config의 `approach_timeout + install_wait + backup_timeout +
result_margin`을 사용하게 한다. 시작 시 `detector_wait >= trap_max_duration`을
검증하고 아니면 노드를 실패시킨다. 기존 `AWAIT_TRAP=20`은 삭제한다.
detector의 opening watchdog과 RatTracker의 경과시간도 `time.monotonic()`으로
계산한다. ROS message header stamp는 TF/관측 freshness에만 쓰고 process timeout과
섞지 않는다.

```python
# __init__
self.job = None                     # 현재 TrapJob + step/step_id/since
self.create_timer(0.5, self.install_tick)

def _now(self):
    return time.monotonic()

def _install(self, job):
    if self.job is not None:
        self._publish_result(job, 'busy', 'another trap job is active')
        return                         # 기존 job을 덮어쓰지 않는다
    request = self._make_approach_request(job, step_id=1)
    if request is None:               # TF/기하 계산 실패는 성공이 아님
        self._publish_result(job, 'failed', 'cannot compute approach pose')
        return
    self._beep_nonblocking()
    self.motion_pub.publish(request)
    self.job = {'msg': job, 'step': 'APPROACH', 'step_id': 1,
                'since': self._now(), 'motion_result': None}

def install_tick(self):
    """APPROACH 성공 → WAIT → BACKUP 성공 → installed. 실패는 즉시 종료."""
    if self.job is None:
        return
    now = self._now()
    elapsed = now - self.job['since']
    step = self.job['step']
    if step == 'APPROACH':
        if elapsed > self.approach_timeout:
            return self._fail_job('approach timeout', cancel_motion=True)
        result = self.job['motion_result']
        if result is not None and result.outcome != 'succeeded':
            return self._fail_job(f'approach {result.outcome}')
        if result is not None:
            if not self._at_approach_pose(self.arrive_tol):
                return self._fail_job('action success but TF outside tolerance')
            self.job.update(step='WAIT', since=now, motion_result=None)
    elif step == 'WAIT':
        if elapsed >= self.install_wait:
            req = make_backup_request(owner='opening',
                                      operation_id=self.job['msg'].operation_id,
                                      step_id=2, distance=self.backup_dist,
                                      speed=self.backup_speed,
                                      timeout_sec=self.backup_timeout)
            self.motion_pub.publish(req)  # Nav2 BackUp: 회전 없이 실제 후진
            self.job.update(step='BACKUP', step_id=2, since=now,
                            motion_result=None)
    elif step == 'BACKUP':
        if elapsed > self.backup_timeout:
            return self._fail_job('backup timeout', cancel_motion=True)
        result = self.job['motion_result']
        if result is not None and result.outcome != 'succeeded':
            return self._fail_job(f'backup {result.outcome}')
        if result is not None:
            job = self.job['msg']
            self.job = None
            self._publish_result(job, 'installed')
```

`motion_result_cb`는 owner/operation/step이 현재 job과 전부 일치할 때만
`motion_result`를 채운다. `_fail_job`은 필요하면 matching motion cancel을 먼저
보내고 `TrapResult(outcome='failed')`를 정확히 한 번 발행한 뒤 job을 비운다.
cancel job은 같은 방식으로 `canceled`를 한 번 발행한다.

```python
def motion_result_cb(self, result):
    if self.job is None:
        return
    job = self.job
    if (result.owner == 'opening'
            and result.operation_id == job['msg'].operation_id
            and result.step_id == job['step_id']):
        job['motion_result'] = result
```

- `time.sleep`은 전부 제거한다. PC speaker fallback은 `subprocess.Popen`으로
  fire-and-forget하거나 제거하고 `cmd_audio`만 사용해 executor를 막지 않는다.
- inspect도 현재 install이 있으면 `busy`로 응답하며 job을 덮어쓰지 않는다.
- timeout/TF 실패/action 실패를 `installed`로 축약하지 않는다. detector는 failed를
  받으면 제한 횟수 재시도하거나 opening을 종료하고 운영자 경고를 남긴다.
- DWB `min_vel_x=0`과 무관하게 Nav2 behavior server의 BackUp action을
  robot_agent가 실행한다. BackUp action이 없는 배포에서는 설치 기능을 비활성화하고
  성공을 가장하지 않는다.

**검증** — robot_agent의 `_FakeAgent` 패턴으로 `_FakeCheck` 스텁 신설:

- 정상: approach result success + TF 확인 → 8초 WAIT → BackUp success → installed 1회
- approach/backup 각각 rejected, failed, canceled, timeout → failed 1회, installed 0회
- TF `None` 또는 action success지만 tolerance 밖 → failed
- install 중 새 install/inspect → busy, 기존 job 유지
- matching cancel → motion cancel + canceled, 이후 늦은 success 무시
- detector가 최대 53초 동안 기다리고 54초에 timeout되는 경계값
- fake ROS time이 멈춰도 monotonic timeout은 진행

---

## 2-12. fleet_msg 방어 파싱 (형식 오류로 노드 안 죽게)

**현상** — `split` 언패킹·`int()`가 형식 안 맞는 문자열에서 예외 → 콜백 예외는
`rclpy.spin`을 뚫고 나가 노드가 죽는다. String으로 유지하는 `/fleet/status`와
관찰용 `/fleet/event`는 전역 토픽이라 잘못된 `ros2 topic pub` 한 번에 여러
구독 노드가 죽을 수 있다. `/fleet/command`는 §2-4의 typed message로 교체한다.

**설계** — 파서가 문법과 의미를 모두 검증하고 실패 시 **None을 반환**한다.
`parse_*`를 호출하는 모든 구독 콜백은 반드시 None을 가드한다.

`fleet_msg.py`:

```python
def parse_status(s):
    """-> (robot, state, battery) | None. 형식이 틀려도 예외를 던지지 않는다."""
    try:
        robot, state, battery = s.split(':')
        battery = int(battery)
        if not valid_robot(robot) or state not in VALID_STATES:
            return None
        if not 0 <= battery <= 100:
            return None
        return robot, state, battery
    except (AttributeError, TypeError, ValueError):
        return None

def parse_event(s):
    try:
        name, x, y = s.split(':')
        x, y = float(x), float(y)
        if name not in VALID_EVENTS or not math.isfinite(x) or not math.isfinite(y):
            return None
        return name, x, y
    except (AttributeError, TypeError, ValueError):
        return None
```

`VALID_STATES`에는 새 `FAULT`, `VALID_EVENTS`에는 실제 지원 이벤트만 넣는다.
`valid_robot`은 빈 문자열·공백·경로 구분자·콜론을 거부하고 `[A-Za-z0-9_-]+`만
허용한다. 로봇 목록 자체는 배포 파라미터로 제한할 수 있지만 parser가 robot4/6을
하드코딩하지는 않는다. 알 수 없는 enum은 warning 후 버려 오타가 상태기계에
들어오지 않게 한다.

String `command()`/`parse_command()` helper는 §2-4 typed command 전환과 함께
삭제한다. FleetCommand 수신부는 `VALID_COMMANDS`와 robot ID를 직접 검증한다.

String parse를 사용하는 모든 호출부(central status/event, db의 관찰 event,
rat_herding의 legacy event 등)는 다음 공통 패턴을 사용한다. 구현 전에 `rg
'parse_(status|event)'`로 실제 호출부를 다시 세어 누락이 없게 한다.

```python
parsed = fleet_msg.parse_event(msg.data)
if parsed is None:
    self.get_logger().warn(f'형식 오류 무시: {msg.data!r}', throttle_duration_sec=5.0)
    return
name, x, y = parsed
```

**검증** — fleet_msg self-check 추가:

```python
assert parse_status('robot4:PATROLLING') is None     # 필드 부족
assert parse_status('robot4:P:abc') is None          # 숫자 아님
assert parse_event('rat:1.0') is None
assert parse_event('a:b:c') is None                  # 좌표가 숫자 아님
assert parse_event('rat_detected:nan:1') is None     # non-finite
assert parse_event('rat_detected:inf:1') is None
assert parse_status('robot4:PATROLLING:101') is None
assert parse_status(':PATROLLING:50') is None
# typed FleetCommand(command='TYPO')도 수신부에서 rejected ACK
```

---

## 2-13. rat_detected 발행 스로틀

**현상** — 추적 전 쥐가 보이는 매 프레임 `rat_detected`가 나가 초당 십수 개
이벤트로 central/herding 로그가 도배된다.

**설계** — "대응 시작 이벤트"와 "지속 위치 관측"을 분리한다. `/fleet/event`의
`rat_detected`는 central mode 진입용으로 1Hz 제한하고, 추적 중 여부와 무관하게
typed `/fleet/rat_observation`을 1Hz로 계속 발행한다. 몰이 노드는 이벤트 문자열을
좌표 스트림으로 오용하지 않고 observation만 구독한다.

```text
# turtle_interfaces/msg/RatObservation.msg
builtin_interfaces/Time stamp
string source_robot
string frame_id          # 반드시 map
float64 x
float64 y
float32 confidence
uint64 track_id          # 한 번의 연속 관측 구간 ID
```

`detector_node.py`:

```python
# __init__
self._last_rat_evt = 0.0    # 마지막 rat_detected 발행 시각
self._last_rat_obs = 0.0
self.rat_obs_pub = self.create_publisher(
    RatObservation, '/fleet/rat_observation', qos_profile_sensor_data)

# synced_cb가 rgb_msg.header.stamp를 observation_stamp 인자로 넘긴다.
# _detect_rat — tracking 분기 전에 공통 수행:
now = self._now()
if now - self._last_rat_obs >= self.rat_observation_period:  # 기본 1.0초
    self._last_rat_obs = now
    self.rat_obs_pub.publish(make_rat_observation(
        stamp=observation_stamp, source=self.robot_id,
        xy=xy, confidence=confidence, track_id=self.rat_track_id))

if not self.tracking:
    if now - self._last_rat_evt >= self.rat_event_period:    # 기본 1.0초
        self._last_rat_evt = now
        self.event_pub.publish(String(data=fleet_msg.event('rat_detected', *xy)))
    return
```

rat_herding은 `observation_ttl_sec` 기본 2.5초보다 오래된 stamp를 버리고, 현재
rat mode의 `track_id`만 사용한다. 관측이 TTL 동안 끊기면 마지막 좌표로 계속
주행하지 않고 정지/탐색 상태로 전환한다. webcam_node도 구현 시 같은 message,
frame, 주기, freshness 계약을 따른다. 서로 다른 source가 동시에 관측하면 최신
stamp와 confidence를 기준으로 선택하되 map-frame 좌표만 합친다.

central도 ACTIVE rat session의 마지막 observation 수신 시각을 monotonic으로
기록한다. `rat_observation_lost_sec` 기본 5초 동안 새 관측이 없으면 detector의
`rat_lost` 이벤트가 유실되었거나 detector가 죽은 것으로 보고 `_end_rat(False)`를
한 번 실행한다. 전체 대응에는 별도 `rat_session_timeout_sec` 기본 120초를 두어
관측이 계속되더라도 TRACKING/HERDING에 영구 고착되지 않게 한다.

**검증**:

- 추적 전 `/fleet/event`와 observation이 각각 약 1Hz인지 확인한다.
- TRACK 진입 후 `rat_detected` 재트리거는 억제되어도 observation은 계속 1Hz인지
  확인한다.
- B가 첫 이벤트 callback 순서와 무관하게 다음 observation부터 좌표를 받는다.
- 2.5초 stale observation, 이전 track_id, map 이외 frame을 몰이 노드가 거부한다.
- observation이 5초 끊기거나 session이 120초를 넘으면 central이 A/B를 복귀시킨다.

---

## 15. 실행 전 필수 기반 수정

아래 항목은 1-2~2-13과 별개처럼 보여도 설치본이나 실기기에서 상태기계를
깨뜨리므로 같은 배포 전에 완료한다.

### 15-1. 하드코딩 경로 제거와 resource 설치

`setup.py`의 `data_files`에 다음을 명시적으로 설치한다.

```python
('share/' + package_name + '/resource', [
    'resource/best.pt',
    'resource/patrol_waypoints.yaml',
    'resource/room_map.yaml',
    'resource/room_map.pgm',
])
```

launch의 map 기본값과 detector/robot_agent의 model·waypoint 기본값은
`get_package_share_directory('turtle_project')` 아래에서 만든다. 소스 workspace의
`/home/rokey/...`를 기본값으로 사용하지 않는다. `room_map.yaml` 안의 `image`는
같은 설치 디렉터리의 상대경로이므로 pgm도 반드시 함께 설치한다.

검증은 빈 임시 install base에 `colcon build`한 뒤 소스 디렉터리 이름을 잠시
바꾼 상태에서 launch description을 생성하고 네 resource가 모두 존재하는지
확인한다. source tree가 우연히 fallback이 되면 실패로 본다.

### 15-2. lifecycle 활성화의 단일 소유자와 정확한 판정

`nav2_activate.sh`는 다음 계약으로 수정한다.

- `inactive`에 포함된 문자열 `active`를 glob으로 찾지 않는다. CLI 결과를
  normalize해 마지막 lifecycle state가 정확히 `active`인지 비교한다.
- 목록의 마지막 노드 하나가 아니라 **모든 대상 노드**가 service에 나타날 때까지
  기다린다. timeout 시 빠진 노드 이름을 출력하고 실패한다.
- configure/activate 명령 각각의 exit code와 전환 후 상태를 확인한다. 노드 없음은
  skip 성공이 아니라 실패다.
- upstream lifecycle manager와 script가 동시에 전환하지 않도록 launch에
  `autostart:=false`를 명시하고 script만 전환 소유자가 된다. 해당 upstream
  launch가 이 인자를 지원하지 않으면 script를 제거하고 manager 하나만 사용한다.
- localization activation process가 0으로 끝나고 map_server/amcl이 둘 다 정확히
  active일 때만 navigation을 시작한다.

shell fixture로 `unconfigured`, `inactive`, `active`, missing, transition failure 출력을
각각 넣어 `inactive`가 성공 처리되지 않고 missing이 exit 1인지 자동 검증한다.

### 15-3. map·Nav2 안전 설정

현재 PGM의 회색값 205는 `negate: 0`에서 occupancy 약 0.196이다.
`free_thresh: 0.25`면 이 픽셀들이 free가 될 수 있으므로 `free_thresh: 0.196`을
시작값으로 사용해 회색을 unknown으로 보존한다. map_server가 발행한 OccupancyGrid를
덤프해 원본 205 픽셀 표본이 0(free)이 아닌 -1(unknown)인지 확인한다.

§2-9의 DWB/inflation 변경과 함께 다음 실기기 gate를 둔다.

실기기 기본은 `use_sim_time:=false`로 launch에서 모든 Nav2/앱 노드에 일관되게
전파한다. yaml의 여러 `use_sim_time: True` 하드코딩은 제거하거나 launch override가
실제로 적용되는지 parameter dump로 검증한다. simulation/rosbag에서만 true를
명시한다.

- global plan이 unknown/벽 셀을 통과하지 않음
- footprint를 반영한 costmap에서 좁은 통로를 억지로 통과하지 않음
- 0.20m/s 주행 후 정지 판정이 정상적으로 완료됨
- trap 접근/BackUp을 최저속으로 시작하고 비상 STOP을 바로 시험

### 15-4. namespace·dock 좌표·일반 action timeout

실제 namespace는 launch의 `namespace`/`__ns` 하나만 source of truth로 사용한다.
robot4.yaml/robot6.yaml의 별도 `namespace` ROS parameter는 제거하고, 모든 노드는
`get_namespace()`에서 robot ID를 얻는다. 허용 로봇 목록은 central parameter로
검증해 오타 namespace가 fleet에 새 로봇처럼 등록되지 않게 한다.

robot_agent 시작 시 `--docked` 같은 추정 기본값으로 DOCKED/IDLE을 먼저
발행하지 않는다. 첫 fresh DockStatus와 Nav2 readiness가 확인될 때까지 FAULT를
보고하고, 센서가 확인된 뒤에만 DOCKED 또는 IDLE로 전환한다.

robot6의 추정 dock 좌표 `(0, 0, 180°)`는 운영 기본값으로 쓰지 않는다.
`dock_pose_valid: false`를 기본으로 두고 실측·저장 전 DOCK 명령을 거부해 FAULT와
명확한 로그를 낸다. dock/undock 각각 3회 성공, 최종 pose 분산과 센서 전환을
기록한 뒤에만 true로 바꾼다.

patrol, pose, backup, dock, undock과 `waitUntilNav2Active` 모두 finite timeout,
goal accepted/result 확인, matching cancel을 가져야 한다. "is complete"만으로 성공을
판정하지 않는다. 각 timeout은 §2-3의 공통 cancellation 경로로 끝난다.

### 15-5. DB 경로와 저장 확인

`holes.db` 상대경로는 실행한 shell의 cwd에 따라 달라지므로 금지한다. launch에서
쓰기 가능한 절대 `db_path`를 넘기고 부모 디렉터리 생성 실패 시 db_node 시작을
실패시킨다. 개발 기본값은 `~/.ros/turtle_project/holes.db`처럼 명시적인 ROS 상태
디렉터리를 expand한 절대경로로 한다.

구멍/트랩 상태 변경은 유실 가능한 `/fleet/event`를 DB 쓰기 명령으로 사용하지
않는다. `RecordHole`/`UpdateTrap` typed service로 저장하고 commit 성공 ACK를 받은
뒤에만 detector가 전역 관찰 이벤트를 발행한다. DB에는 `busy_timeout`, transaction,
schema version을 두고 재시작·중복 요청에도 idempotent하게 처리한다.
`/fleet/event`는 central/UI 관찰용일 뿐 영속성의 source of truth가 아니다.

두 service 요청에는 `uint64 request_id`와 좌표(및 UpdateTrap의 installed)를 넣고,
응답에는 `bool success`, DB `hole_id`, canonical hole 좌표, `string detail`을 넣는다.
DB는 처리한 request ID를 별도 unique column/table에 기록해 ACK 유실 후 같은 요청을
재전송해도 INSERT/UPDATE가 한 번만 적용되게 한다. detector는 service timeout 시
같은 request ID로 제한 재시도하며 commit ACK 전에는 저장 성공을 로그/이벤트로
보고하지 않는다.

### 15-6. 의존성·테스트 완료 조건

실제 import/launch를 기준으로 `package.xml`과 설치 문서를 보완한다. 최소한
`irobot_create_msgs`, `ament_index_python`, Nav2 simple commander 및 YOLO runtime을
새 install 환경에서 확인한다. YOLO가 rosdep 대상이 아니면 지원 Python 버전과
고정 범위, 설치 명령, model checksum을 `docs/run.md`에 기록한다.

완료 기준은 개별 `--check` 통과만이 아니다.

리뷰 시점 기준 pytest 2건 실패와 flake8 약 50건을 known baseline으로 남겨두지
않는다. 실제 결함이면 수정하고, 생성 코드/환경 문제면 근거와 함께 test 설정에서
명시적으로 제외한 뒤 0 failure 상태를 만든다.

1. 모든 self-check와 pytest/ament lint가 통과한다.
2. 두 namespace를 띄운 launch test에서 local trap 결과가 교차하지 않는다.
3. 늦은 DB/trap/motion 결과, callback 역순, timeout/cancel을 자동 테스트한다.
4. 임시 install-only 환경에서 model/map/waypoint를 찾는다.
5. rosbag 또는 실기기에서 RGB-depth 정합, status 단절 복구, STOP, trap BackUp,
   rat observation freshness를 검증한다.

### 15-7. 미구현 기능 capability gate

현재 rat_herding/webcam/droppings 경로는 TODO 또는 입력을 소비하지 않는 부분이
있으므로 노드가 실행된다는 이유로 기능 준비 완료로 보지 않는다. 각 로봇은
timestamp가 있는 `RobotCapabilities`를 발행하고 central은 status TTL과 같은
방식으로 freshness를 확인한다.

```text
# turtle_interfaces/msg/RobotCapabilities.msg
builtin_interfaces/Time stamp
string robot
bool detector_ready
bool trap_ready
bool herding_ready
bool webcam_ready
```

몰이 알고리즘과 RatObservation 소비·정지 테스트가 끝나기 전
`herding_ready=false`이며 central은 B를 배정하지 않고 A 단독 대응만 한다.
webcam도 실제 frame→map 관측과 freshness 테스트 전에는 false다. YOLO model이
`droppings` class를 내보낸다면 다음 중 하나를 명시적으로 선택한다.

- 제품 범위에 포함: `droppings_detected` typed 관측, DB/UI 저장 계약과 테스트 구현
- 범위 밖: model class allowlist에서 제외하고 문서/로그에 미지원이라고 명시

아무 분기도 없이 검출 결과를 조용히 버리는 상태는 완료로 인정하지 않는다.

---

## 16. 파일별 변경 요약

| 파일 | 항목 | 변경 |
|---|---|---|
| `turtle_interfaces/msg/TrapJob.msg` | 2-1, 2-8 | operation/attempt ID와 cancel phase 추가 |
| `turtle_interfaces/msg/TrapResult.msg` | 2-1, 2-11 | ID가 붙은 typed trap 결과 신설 |
| `turtle_interfaces/msg/Motion{Request,Cancel,Result}.msg` | 2-3, 2-8, 2-11 | 이동 소유권·상관관계·취소·결과 계약 신설 |
| `turtle_interfaces/msg/FleetCommand.msg`, `CommandAck.msg` | 2-4 | 명령 ID·중복 억제·수신 ACK/재전송 계약 신설 |
| `turtle_interfaces/msg/RatObservation.msg` | 2-13 | timestamp/track ID가 있는 지속 쥐 좌표 신설 |
| `turtle_interfaces/msg/RobotCapabilities.msg` | 15-7 | 미구현/비정상 기능을 역할 후보에서 제외 |
| `turtle_interfaces/srv/RecordHole.srv`, `UpdateTrap.srv` | 15-5 | DB 저장 ACK가 있는 typed service 신설 |
| `turtle_interfaces/CMakeLists.txt`, `package.xml` | 인터페이스 | 새 msg/srv 생성 의존성 등록 |
| `turtle_project/camera_node.py` | A | **삭제** |
| `turtle_project/detector_node.py` | A, 2-1, 2-6~2-8, 2-11, 2-13 | 원본 정합 검증 / operation ID / stale DB·trap·motion 결과 거부 / 전체 opening 취소 / 분리 timeout / RatObservation |
| `turtle_project/robot_agent.py` | 2-2~2-4, 2-8, 2-11 | action result·timeout·센서 freshness / 전체 이동 취소 / pending role / motion owner·ID / BackUp 실행 |
| `turtle_project/central_node.py` | 1-2, 2-4, 2-5, 2-13, 15-7 | monotonic timer / status·capability TTL / cooldown / fresh B 선별 / stale patroller / rat session timeout |
| `turtle_project/trap_check_node.py` | 2-1, 2-8, 2-11 | ID 기반 result/cancel / 실패 보존 스텝머신 / MotionResult 대기 / nonblocking beep |
| `turtle_project/fleet_msg.py` | 2-12 | 문법+enum+범위+finite 검증과 None 반환 |
| `turtle_project/opening_test_node.py` | 2-10 | CameraInfo 상대 parameter + sensor QoS |
| `turtle_project/rat_herding_node.py`, `webcam_node.py` | 2-13, 15-7 | RatObservation freshness/track 계약과 readiness gate 적용 |
| `turtle_project/db_node.py` | 15-5 | 절대 DB 경로 / typed 저장 service / transaction·idempotency |
| `config/nav2.yaml` | 2-9 | dummy 삭제 / 속도·정지·inflation 정합 / 정확한 project 주석 |
| `config/robot4.yaml`, `robot6.yaml` | 15-4 | 중복 namespace 제거 / dock pose validity 추가 |
| `resource/room_map.yaml` | 15-3 | 회색 unknown을 보존하는 free threshold |
| `launch/robot_bringup.launch.py` | 15-1, 15-2 | package-share map / lifecycle 단일 소유자 |
| `scripts/nav2_activate.sh` | 15-2 | 모든 노드 대기 / exact active / 전환 실패 전파 |
| `setup.py` | A, 15-1 | camera entry 제거 / model·map·waypoint 설치 |
| `package.xml`, `docs/run.md` | A, 15-6 | runtime 의존성·설치·검증 절차 갱신 |
| `docs/architecture.md`, `docs/flowchart.md` | 계약 변경 | camera 통합 / typed command·trap·motion / RatObservation 흐름 갱신 |

## 17. 구현 순서 (권장 커밋 단위 — 각 단계 후 self-check + colcon build)

인터페이스와 안전 기반을 먼저 만든 뒤 상태기계를 옮긴다. 각 단계에서 unit test,
`colcon build`, 관련 package test를 통과시킨다.

- [ ] **1. fleet_msg 방어 파싱 (2-12)** — 문법/의미 검증 + 호출부 가드.
- [ ] **2. typed interfaces** — TrapJob/TrapResult, Motion*, FleetCommand/CommandAck,
      RatObservation, RobotCapabilities, RecordHole/UpdateTrap 생성과 round-trip test.
- [ ] **3. package resource·경로·의존성 (15-1, 15-6)** — install-only test를 먼저
      통과시켜 이후 실기기 테스트가 소스경로에 의존하지 않게 한다.
- [ ] **4. lifecycle/map/Nav2 설정 (2-9, 15-2~15-4)** — shell fixture와 yaml
      정적 검증 후 저속 bringup. robot6 dock pose는 실측 전 invalid 유지.
- [ ] **5. robot_agent motion owner (2-2, 2-3, 2-4, 2-8)** — action
      result/timeout/cancel, FAULT, pending role, Motion* server 측 계약.
- [ ] **6. trap result 상관관계 (2-1)** — local typed result + op/attempt stale 거부.
- [ ] **7. trap_check 스텝머신 (2-11)** — MotionResult 기반 접근/BackUp,
      busy/failure/cancel, monotonic timeout. 거짓 installed test를 먼저 작성한다.
- [ ] **8. detector 비동기 수명주기 (2-6~2-8)** — DB/trap/motion callback ID
      가드와 `_abort_opening`; callback 역순 테스트.
- [ ] **9. central 복구/역할 (1-2, 2-4, 2-5)** — 독립 timer, TTL, cooldown,
      fresh 역할 후보와 A 단독 fallback.
- [ ] **10. rat 관측·기능 gate (2-13, 15-7)** — 이벤트/observation 분리,
      tracking 중 연속 발행, herding TTL/track ID, capability와 session timeout.
- [ ] **11. camera 통합·opening_test (A, 2-7, 2-10)** — 원본 sync,
      RGB-depth 정합 검증, depth fallback, info topic/QoS, camera_node 삭제.
- [ ] **12. DB 쓰기 계약 (15-5)** — 절대경로와 typed ACK service, 재시작·중복
      요청 테스트. 이후 `/fleet/event`는 관찰 전용으로 전환.
- [ ] **13. 통합/실기기 gate** — §15-6의 다섯 완료 기준과 두 로봇 동시 trap,
      status 단절, STOP, 늦은 결과, rat 대응을 모두 통과한 뒤 배포한다.

## 18. 2026-08-06 개선 설계문서와의 관계

이 설계는 코드 리뷰에서 확인된 경합 때문에 2026-08-06 문서의 상관관계·상태
진실성 제안을 다시 채택한다. 충돌 시 이 문서가 우선한다.

| 2026-08-06 문서 | 이 문서의 처리 |
|---|---|
| §7.6 TrapJob에 robot/operation/attempt ID + TrapResult typed 응답 | **채택** — 상대토픽으로 로봇을 격리하고 operation/attempt로 같은 로봇의 늦은 응답도 격리 |
| §4.3 B 후보 자격(IDLE > DOCKED, 그 외 제외) + A 단독 추적 | **채택·보강** — status TTL·battery 조건과 `pending_role` 상태 진실성 추가 |
| §3.2 UNDOCK 의미 변경(언도킹만, 순찰은 별도 PATROL) | **미채택** — 현행 계약(UNDOCK→자동 순찰) 유지. central·agent 동시 배포 부담 대비 이득 없음 |
| FAULT/UNKNOWN 상태 도입 | **FAULT 최소 채택** — action/센서 결과로 물리 상태를 확정할 수 없을 때 DOCKED/IDLE로 거짓 축약하지 않음 |
| RatObservation typed 토픽 | **채택** — 대응 이벤트와 지속 위치 스트림을 분리하고 TTL/track ID 계약 추가 |
