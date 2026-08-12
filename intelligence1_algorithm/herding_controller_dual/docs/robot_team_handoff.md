# 로봇 파트 전달사항 — 쥐몰이 알고리즘 변경에 따른 연동 수정 요청

**작성** 2026-08-09 · Algorithm 파트 (박선욱)
**대상 패키지** `src/turtle_project` (로봇/중앙 PC), `intelligence1_algorithm/herding_controller_dual` (알고리즘)

---

## 0. 무엇이 바뀌었나 (한 문단)

몰이 알고리즘이 **로봇 한 대만 목표점을 받던 방식(플랜 A)에서, 두 대가 동시에 협공하는 방식(플랜 B + 엔드게임 협공)으로 바뀌었습니다.**
쥐가 덫 0.8m 안에 들어왔는데 3초간 안 잡히면, **두 로봇이 덫 반대편 좌우 ±60°로 갈라져 동시에 압박**합니다. 이 동작은 두 로봇이 **각자 다른 좌표로 동시에** 가야만 성립합니다.

그래서 지금처럼 목표점을 한 대에게만 전달하면 **협공이 성립하지 않고, 성공률이 84% → 39%대로 떨어집니다.** 아래 P0 항목이 그 연결 작업입니다.

---

## P0 — 이게 없으면 시연 자체가 안 됩니다

### P0-1. 통합 어댑터를 목표점 2개로 확장

**파일** `src/turtle_project/turtle_project/rat_herding_node.py`

**현재 상태**
```python
# 54행 — 목표점을 하나만 구독
self.create_subscription(
    PoseStamped, 'herding_controller/robot2_goal', self.goal_cb, 10)

# 66~68행 — relay 발행자도 robot_b 하나만 생성
self.goal_relay_pub = self.create_publisher(
    PoseStamped, f'{robot}/target_pose', 10)

# 77~81행 — robot_b에게만 relay
def goal_cb(self, msg):
    if self.goal_relay_pub is None:
        return
    self.goal_relay_pub.publish(msg)
```

**필요한 변경**
- `herding_controller/robot1_goal`과 `herding_controller/robot2_goal`을 **둘 다** 구독
- relay 발행자를 **두 개** 만든다 — `robot_a/target_pose`, `robot_b/target_pose`
- `robot1_goal` → robot_a, `robot2_goal` → robot_b 로 각각 중계

**주의** — 로봇 A는 TRACK 명령을 받은 로봇이고, 지금은 목표점을 받지 않습니다. 이제부터는 **로봇 A도 우리 알고리즘이 계산한 목표점으로 움직여야 합니다.** 이게 이번 변경의 핵심입니다.

---

### P0-2. launch 파일이 띄우는 패키지 교체

**파일** `src/turtle_project/launch/central_pc.launch.py` (18행, 27행)

```python
# 현재 — 플랜 A 패키지
get_package_share_directory('herding_controller')
Node(package='herding_controller', executable='herding_node', ...)

# 변경 — 플랜 B 패키지
get_package_share_directory('herding_controller_dual')
Node(package='herding_controller_dual', executable='herding_node', ...)
```

실행 파일 이름(`herding_node`)과 파라미터 파일 경로(`config/herding_params.yaml`)는 동일합니다. 패키지 이름만 바뀝니다.

---

### P0-3. 쥐 위치 발행 (Detection 파트)

**파일** `src/turtle_project/turtle_project/detector_node.py` (112행, TODO 상태)

우리 알고리즘의 **유일한 입력**입니다. 이게 없으면 아무것도 동작하지 않습니다.

- 형식: `fleet_msg.event('rat_detected', x, y)`
- 좌표계: **map 프레임, 미터 단위**
- 갱신 주기: 5Hz 이상 권장 (알고리즘은 10Hz로 돌지만 칼만 필터가 보간합니다)

좌표계나 단위가 다르면 알고리즘이 엉뚱한 곳으로 로봇을 보냅니다. 연동 전에 한 번 좌표를 찍어서 맞춰봤으면 합니다.

---

## P1 — 성능·안정성에 직접 영향

### P1-1. Nav2 목표 재전송 폭주 ⚠️ 아직 아무도 안 본 문제

**파일** `src/turtle_project/turtle_project/robot_agent.py` (139~146행)

```python
def target_cb(self, msg):
    if self.patrolling:
        self.stop_patrol()
    self.nav.goToPose(msg)      # ← 메시지 올 때마다 무조건 재전송
```

**문제** — 알고리즘 노드는 **10Hz로 목표점을 발행**합니다. 지금 구조라면 `goToPose`가 **초당 10번** 호출되고, 그때마다 Nav2가 기존 목표를 선점(preempt)하고 경로를 다시 짭니다. 로봇이 제자리에서 떨거나 거의 못 움직일 수 있습니다.

**시뮬레이션에서는 이 문제가 드러나지 않습니다** — 시뮬은 목표점 방향으로 즉시 이동하는 모델이라 Nav2 경로 재계획 비용이 없습니다. 그래서 우리 성공률 숫자에는 반영되어 있지 않습니다.

**제안** — 중계 단계에서 걸러주세요. 둘 중 하나면 됩니다.
- 직전에 보낸 목표와 **0.15~0.20m 이상 차이날 때만** 재전송
- 또는 재전송 주기를 **1~2Hz로 제한**

어느 쪽이 나은지는 실기에서 봐야 알 것 같습니다. 판단은 로봇 파트에 맡기겠습니다.

---

### P1-2. Nav2 inflation radius 정합

우리가 "로봇이 설 수 있는 자리"로 판정하는 기준은 **0.201m** 입니다.

| 항목 | 값 | 근거 |
|---|---|---|
| `robot_radius_m` | 0.171 | TurtleBot 4 실측 342 × 339 mm의 외접 반경 |
| `robot_wall_clearance_m` | 0.03 | 벽에 딱 붙지 않도록 두는 여유 |
| **요구 여유** | **0.201** | 위 둘의 합 |

**Nav2의 `inflation_radius`가 0.201보다 크면**, 우리가 유효하다고 판단해서 보낸 목표점을 Nav2가 도달 불가로 거부합니다. 특히 협공 지점은 벽 가까이 잡히는 경우가 있어서 영향을 받습니다.

이 두 값은 **yaml에서 바꿀 수 있게 외부화해 두었습니다** (`config/herding_params.yaml`). 로봇 파트 값을 알려주시면 거기에 맞추겠습니다. 재빌드 필요 없습니다.

---

### P1-3. 맵 파일 `free_thresh` 불일치 ⚠️

같은 `room_map.yaml`인데 두 파트의 값이 다릅니다.

| 위치 | `free_thresh` |
|---|---|
| `src/turtle_project/resource/room_map.yaml` | **0.25** |
| `intelligence1_algorithm/herding_controller_dual/maps/room_map.yaml` | **0.196** |

**0.196으로 낮춘 이유** — 이 맵의 회색값 205는 occupancy로 환산하면 약 0.196입니다. `free_thresh: 0.25`면 이 회색이 **free(빈 공간)로 해석되어 벽이 뚫립니다.** 0.196으로 낮추면 unknown으로 보존됩니다.

로봇 파트도 **0.196으로 맞춰야** Nav2가 벽을 통과하는 경로를 짜지 않습니다. 실기에서 OccupancyGrid를 덤프해서 확인한 값입니다.

---

### P1-4. 카메라 최소 인식 거리 (Vision 파트)

OAK-D Pro 기본 설정(800P)의 최소 인식 거리는 **약 0.8m**입니다.

그런데 협공이 발동하면 **로봇이 쥐로부터 0.3m 거리에 섭니다.** 800P 설정 그대로면 **가장 중요한 순간에 쥐를 놓칩니다.**

- **400P + extended disparity** 설정 시 최소 약 0.2m
- 해상도가 낮아지므로 인식 정확도와의 트레이드오프가 있습니다

Vision 파트 판단이 필요합니다. 결정되면 알려주세요.

---

## P2 — 확인·합의만 하면 되는 항목

### P2-1. 주행 속도 (safe mode)

- safe mode 유지: 0.31 m/s
- 해제: 0.46 m/s

시뮬레이션은 **로봇이 목표점에 제때 도착한다**고 가정합니다. 실기에서 로봇이 목표점을 못 따라가면 몰이가 끊깁니다. 안전 문제가 없다면 해제 쪽이 유리합니다.

### P2-2. 덫 좌표 합의 (Trap 파트)

현재 알고리즘 기본 덫 좌표는 `capture_zone_x_m: -2.81`, `capture_zone_y_m: -5.36` 이고, 검증은 **top / left / bottom 3개 위치**에서 했습니다.

**실제 덫 설치 좌표와 반드시 일치해야 합니다.** 설치 위치가 정해지면 좌표(map 프레임, 미터)를 알려주세요. yaml에서 바꿉니다.

### P2-3. RC카 조작 매뉴얼 숙지 (시연 담당)

문서: `intelligence1_algorithm/herding_controller_dual/docs/operator_protocol.md`

**규칙 4가 특히 중요합니다** — "막다른 구석으로는 들어가지 않고, 필요하면 로봇 옆을 스쳐서라도 트인 쪽으로 빠져나간다."

이유: **직선으로만 도망가는 표적에서는 로봇 1대와 2대의 성공률이 90.7%로 동일합니다.** 조작자가 단순하게 움직이면 협공이 발동하지 않아, 로봇 B가 아무 일도 안 하는 것처럼 보입니다.

---

## 요약 체크리스트

| # | 항목 | 담당 | 우선순위 |
|---|---|---|---|
| P0-1 | `rat_herding_node`에서 목표점 2개 중계 | 로봇/중앙 | 필수 |
| P0-2 | launch 패키지를 `herding_controller_dual`로 교체 | 로봇/중앙 | 필수 |
| P0-3 | `detector_node`의 쥐 위치 발행 구현 | Detection | 필수 |
| P1-1 | Nav2 목표 재전송 제한 (0.15~0.2m 또는 1~2Hz) | 로봇 | 높음 |
| P1-2 | Nav2 `inflation_radius` 값 회신 (기준 0.201m) | 로봇 | 높음 |
| P1-3 | 로봇 파트 맵 `free_thresh` 0.25 → 0.196 | 로봇 | 높음 |
| P1-4 | OAK-D 최소 인식거리 설정 결정 (400P 여부) | Vision | 높음 |
| P2-1 | safe mode 해제 여부 결정 | 로봇 | 보통 |
| P2-2 | 실제 덫 설치 좌표 회신 | Trap | 보통 |
| P2-3 | RC카 조작 매뉴얼 숙지 | 시연 | 보통 |

---

## 참고

- 알고리즘 상세·검증 결과·예상 Q&A: 코드 리뷰 준비 페이지
- 조작자 매뉴얼: `docs/operator_protocol.md`
- 파라미터와 근거 주석: `config/herding_params.yaml`
- 개발 경위 전체: `intelligence1_algorithm/herding_controller_트러블슈팅_노트.md`

**현재까지의 성공률은 모두 실제 SLAM 맵 기반 시뮬레이션 결과이며, 실물 로봇 검증은 아직 수행하지 않았습니다.** 위 P1 항목들(특히 P1-1 목표 재전송)은 실기에서만 드러나는 문제라, 연동 후 함께 확인했으면 합니다.
