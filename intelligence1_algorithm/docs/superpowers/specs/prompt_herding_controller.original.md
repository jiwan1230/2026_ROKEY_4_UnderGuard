# [작업 지시] UnderGuard AMR — 2대 협업 표적 몰이(Herding) 알고리즘 패키지 개발

## 0. 역할
너는 ROS2 Humble 기반 로봇 소프트웨어 엔지니어다.
아래 명세대로 **독립 실행 가능한 몰이 제어 알고리즘 패키지**를 처음부터 구현하고, 오프라인 시뮬레이션 검증까지 마쳐라.
요구사항을 임의로 축소하지 말고, 애매한 부분은 구현 후 "확인 필요 항목"으로 따로 보고해라.

---

## 1. 프로젝트 배경

- 프로젝트명: **UnderGuard AMR** (실내 순찰형 자율이동로봇)
- 시나리오: 로봇 2대가 협업해 **실시간으로 표적(쥐)을 추적하고, 미리 지정된 포획 구역으로 몰아넣는다.**
- 실제 쥐 대신 **미니카**를 표적 대역으로 사용해 시연·검증한다.
- 나는 이 프로젝트에서 **몰이 알고리즘 파트**를 담당한다. 나중에 팀의 본 프로젝트(perception 노드, mission manager, Nav2 스택)와 합칠 예정이므로 **결합도를 최대한 낮춰야 한다.**
- 기존 환경: TurtleBot4, namespace `/robot6` 계열, `ROS_DOMAIN_ID=6`, Nav2 `NavigateToPose`, OAK-D RGB-D.

### 확정된 설계 결정
1. **목표 지점**: 맵 상에 **고정된 포획 구역 1곳**이 사전 지정되어 있다 (파라미터로 좌표+반경 지정).
2. **로봇 역할**: **Driver / Blocker 역할을 상황에 따라 동적으로 교대**한다. 고정 배정이 아니다.
3. **표적 하드웨어 제약**: 가용 장비는 **미니카 본체와 RC 리모컨뿐**이다. 미니카에 센서·연산장치를 얹을 수 없으므로 **자율 도주 로직 탑재는 불가능**하다.
   - → **검증(validation)은 오프라인 시뮬레이터에서 통계적으로 수행**한다. (100회 시행)
   - → **미니카는 시연(demonstration) 및 소규모 실물 확인용**으로만 사용한다. (10회 시행)
   - → 사람이 조종하되 **엄격한 도주 규칙 프로토콜 + 조종자 눈가림 + 대조군 실험**으로 편향을 통제한다. (4-5절 참조)
   - → 도주 모델은 **교체 가능한 플러그인 구조**로 설계해, 시뮬레이션 모델과 실측 로그 재생을 동일 인터페이스로 다룬다.

---

## 2. 알고리즘 개요 (반드시 이 설계를 따를 것)

### 2-0. 기본 원리 — 양치기 개(Shepherding)
표적은 로봇이 다가오면 **로봇의 반대 방향으로 도망친다.** 이것이 유일한 조종 수단이다.
따라서 표적을 목표 G로 보내려면, 로봇은 **G의 정반대편(표적 뒤쪽)**에 서서 밀어야 한다.

### 2-1. 표적 상태 추정 (Target State Estimator)
- 입력: 표적의 map 좌표 관측 (연속적, 노이즈 있음)
- **상수속도 모델 칼만 필터(Constant Velocity KF)**로 위치 + 속도 벡터를 추정한다.
- 관측이 끊기면(occlusion: 가구 밑 진입 등) 예측만으로 외삽하되, `occlusion_timeout_sec` 초과 시 LOST 상태로 전환.

### 2-2. 도주 방향 예측 (Escape Direction Model)
격자 기반 마르코프 전이확률로 **다음에 어느 방향으로 도망칠지**를 확률로 산출한다.
- 기본 성향: **벽면 선호(thigmotaxis)** — 벽 따라 직진 0.70 / 벽 밀착 0.20 / 개방공간 진출 0.10
- **로봇 반발 항**: 각 로봇으로부터 멀어지는 방향에 가중치를 더한다. 가중치는 로봇-표적 거리에 반비례.
- **관성 항**: 현재 속도 벡터 방향에 `momentum_weight` 가중.
- 정적 장애물 셀로의 전이확률은 0으로 마스킹.
- 출력: 도주 후보 방향별 확률 분포 + 상위 K개 도주로(escape route) 좌표.

### 2-3. Driving Point 계산 (Driver 로봇의 목표)
```
u = normalize(target_pos - goal_pos)          # 목표의 반대 방향 단위벡터
P_drive = target_pos + drive_distance_m * u
```
- `drive_distance_m`은 표적의 **반응거리 r_flee보다 약간 작게** 설정해 지속적 압박을 유지한다.
- **패닉 방지 하한**: 로봇-표적 거리가 `panic_distance_m` 미만이 되면 접근을 중단하고 후퇴한다.
  (너무 가까우면 표적이 무작위 방향으로 튀어 몰이가 실패한다. 반드시 구현할 것.)
- 표적이 이미 목표 방향으로 잘 이동 중이면 압박을 완화한다 (`alignment_threshold` 이상이면 drive_distance 확대).

### 2-4. Blocking Point 계산 (Blocker 로봇의 목표)
1. 2-2의 도주 확률 분포에서 **목표 G 방향의 반구(hemisphere)를 제외**한다.
2. 남은 방향 중 **확률 최댓값 도주로**를 고른다 (= 표적이 목표를 벗어나 도망갈 가장 유력한 경로).
3. 그 경로상의 좌표를 Blocking Point로 삼아 선점 이동한다.
4. 벽이나 장애물이 이미 그 경로를 막고 있으면(자연 깔때기), 차선 확률 경로로 넘어간다.

### 2-5. 역할 동적 배정 (Role Assignment)
매 사이클 다음을 수행한다:
1. 로봇 1, 2 각각에 대해 Driving Point까지의 비용(거리 + 방향 전환 비용)을 계산한다.
2. **비용이 낮은 로봇을 Driver**, 나머지를 Blocker로 배정한다.
3. **히스테리시스 필수**: 역할 교대는 비용 차이가 `role_swap_margin` 이상일 때만, 그리고 마지막 교대 후 `role_swap_cooldown_sec` 이 지난 뒤에만 허용한다. (이거 없으면 두 로봇이 역할을 초당 몇 번씩 바꾸며 제자리에서 진동한다. 반드시 구현할 것.)
4. 두 로봇의 목표 지점이 `min_robot_separation_m` 미만으로 가까워지면 충돌 회피를 위해 Blocker 목표를 밀어낸다.

### 2-6. 상태 기계 (State Machine)
```
IDLE → SEARCH → TRACK → HERD → CORNER → CAPTURED
                  ↑        ↓        ↓
                  └──── LOST ←──────┘
```
| 상태 | 진입 조건 | 동작 |
|---|---|---|
| SEARCH | 표적 미관측 | 순찰/탐색 (본 프로젝트 담당, 여기선 상태만 발행) |
| TRACK | 표적 관측 확보 | KF 수렴 대기, 아직 압박 없음 |
| HERD | KF 수렴 + 목표까지 거리 > capture_radius | Driver/Blocker 배정, 몰이 수행 |
| CORNER | 표적이 포획 구역 근처 + 도주로 확률 분산 낮음 | 양 로봇 모두 압박, 퇴로 차단 |
| CAPTURED | 표적이 포획 구역 반경 내 `capture_hold_sec` 이상 체류 | 정지, 성공 발행 |
| LOST | occlusion_timeout 초과 | 마지막 확률지도 기반 재탐색 |

### 2-7. 보조: 확률 지도 (Occlusion Recovery)
표적이 시야에서 사라졌을 때만 사용하는 축소 버전 베이지안 격자.
- 마지막 추정 위치를 중심으로 확률을 놓고, 시간에 따라 확산(diffusion) + 감쇠(decay)
- LOST 상태에서 확률 최댓값 셀을 재탐색 목표로 발행
- **TRACK/HERD 상태에서는 사용하지 않는다** (실시간 관측이 있으므로 불필요)

---

## 3. 구현 요구사항

### 3-1. 패키지 구조 (반드시 이대로)
```
herding_controller/
├── package.xml
├── setup.py
├── config/
│   └── herding_params.yaml
├── herding_controller/
│   ├── __init__.py
│   ├── grid_map.py              # 격자 <-> map 좌표 변환, 장애물/벽 마스크
│   ├── target_estimator.py      # 칼만 필터 상태 추정
│   ├── escape_model.py          # 마르코프 도주 방향 예측
│   ├── herding_planner.py       # Driving/Blocking Point 계산
│   ├── role_assigner.py         # 역할 동적 배정 + 히스테리시스
│   ├── occlusion_grid.py        # LOST 복구용 베이지안 격자
│   ├── state_machine.py         # 6상태 FSM
│   ├── herding_core.py          # 위 전부를 조합한 파사드
│   └── herding_node.py          # ROS2 rclpy 노드. 여기서만 ROS import
└── test/
    ├── evasion_models/
    │   ├── base.py              # 추상 인터페이스
    │   ├── reactive_flee.py     # 자율 도주 (기본 검증용)
    │   ├── wall_hugger.py       # 벽면 선호 + 도주
    │   ├── line_tracer.py       # 고정 경로 (로봇 무시)
    │   └── human_replay.py      # 사람 조종 로그 재생
    ├── simulator.py             # ROS 없는 2D 몰이 시뮬레이터
    ├── run_validation.py        # ALGO-001~007 자동 검증 + 리포트
    └── test_*.py                # 단위 테스트
```

**최우선 제약**: `herding_core.py`와 그 하위 모듈은 `rclpy`를 절대 import하지 않는다.
입력/출력은 순수 파이썬 자료형(float, tuple, numpy array, dataclass)만 사용한다.
→ 본 프로젝트 통합 시 `herding_node.py`만 교체하면 되고, 오프라인 검증이 가능해야 한다.

### 3-2. 도주 모델 플러그인 인터페이스
표적 행동을 다음 인터페이스로 추상화해, 시뮬레이션 모델과 실측 로그 재생을 동일하게 다룬다.
```python
class EvasionModel(ABC):
    @abstractmethod
    def step(self, target_state, robot_positions, obstacle_map, dt) -> np.ndarray:
        """다음 표적 속도 벡터를 반환한다."""
```
구현할 모델:
| 모델 | 용도 | 설명 |
|---|---|---|
| `reactive_flee` | **주 검증용** | 로봇 반대 방향 도주. 통계 검증의 기준 모델 |
| `wall_hugger` | 검증용 | 벽면 선호 + 도주. 실제 쥐에 가장 근접 |
| `noisy_human` | **실물 근사** | wall_hugger에 반응 지연(0.3~0.8초 랜덤)과 조작 노이즈를 추가. 사람 조종을 모사 |
| `random_walk` | **대조군** | 로봇을 무시하고 무작위 이동. 우연 성공률 baseline 산출용 |
| `log_replay` | 실측 재생 | 미니카 실험에서 기록한 궤적 CSV를 재생 |

**`noisy_human`이 실물 시연 성공률의 예측치**가 되어야 한다. 사람은 반응이 느리고 부정확하므로, 이 모델에서의 성능이 미니카 실험 결과와 가장 가까울 것이다.

### 3-3. ROS2 인터페이스 (herding_node.py)

**Subscribe**
| 토픽 | 타입 | 설명 |
|---|---|---|
| `~/target_pose` | `geometry_msgs/PoseStamped` | 실시간 추적된 표적 map 좌표 |
| `~/robot1_pose`, `~/robot2_pose` | `geometry_msgs/PoseStamped` | 각 로봇 현재 위치 |
| `/map` | `nav_msgs/OccupancyGrid` | 장애물/벽 마스크 (transient_local QoS) |

**Publish**
| 토픽 | 타입 | 설명 |
|---|---|---|
| `~/robot1_goal`, `~/robot2_goal` | `geometry_msgs/PoseStamped` | 각 로봇의 목표 좌표 |
| `~/herding_state` | `std_msgs/String` | JSON: fsm_state, roles, target_pos, target_vel, escape_prob_top3, latency_ms |
| `~/escape_probability` | `nav_msgs/OccupancyGrid` | 도주 확률 시각화 (RViz용) |
| `~/capture_result` | `std_msgs/Bool` | 포획 성공 신호 |

- 제어 주기: `control_rate_hz` (기본 5.0 Hz — 실시간 몰이라 예측 파트보다 빨라야 함)
- **Nav2 액션을 직접 호출하지 않는다.** 목표 좌표만 발행하고 실제 주행은 팀의 mission manager가 담당한다.

### 3-4. config/herding_params.yaml
모든 임계값을 YAML로 노출한다. 코드 내 매직넘버 금지.
```yaml
herding_controller:
  ros__parameters:
    frame_id: "map"
    control_rate_hz: 5.0

    # --- Capture Zone (고정 포획 구역) ---
    capture_zone_x_m: 3.0
    capture_zone_y_m: 3.0
    capture_radius_m: 0.5
    capture_hold_sec: 3.0

    # --- Grid ---
    grid_resolution_m: 0.25
    grid_width_cells: 40
    grid_height_cells: 40

    # --- Target Estimator (KF) ---
    kf_process_noise: 0.1
    kf_measurement_noise: 0.05
    occlusion_timeout_sec: 3.0

    # --- Escape Model (Markov) ---
    markov_wall_follow_p: 0.70
    markov_wall_hug_p: 0.20
    markov_center_p: 0.10
    momentum_weight: 0.4
    robot_repulsion_weight: 1.5
    wall_detect_radius_cells: 1
    escape_route_top_k: 3

    # --- Herding Control ---
    drive_distance_m: 0.8
    flee_reaction_distance_m: 1.0
    panic_distance_m: 0.35
    alignment_threshold: 0.7
    block_lookahead_m: 1.2

    # --- Role Assignment ---
    role_swap_margin: 0.5
    role_swap_cooldown_sec: 2.0
    min_robot_separation_m: 0.6

    # --- Occlusion Grid ---
    diffusion_rate: 0.2
    decay_factor: 0.9
```

---

## 4. 검증 요구사항 (핵심 — 반드시 수행)

### 4-1. 정량 합격 기준
| ID | 요구사항 | 합격 기준 |
|---|---|---|
| ALGO-001 | **몰이 성공률** — 표적을 포획 구역에 넣는 비율 | 100회 시행 **≥ 70%** |
| ALGO-002 | **평균 포획 소요 시간** | **≤ 60 초** (시뮬레이션 시간 기준) |
| ALGO-003 | **패닉 발생률** — 로봇-표적 거리가 panic_distance 미만이 된 비율 | **≤ 10%** |
| ALGO-004 | **역할 진동 없음** — 역할 교대 횟수 | 1회 시행당 **≤ 5회** |
| ALGO-005 | **제어 주기 지연** | 1 사이클 **≤ 100 ms** (40×40 격자) |
| ALGO-006 | **Occlusion 복구** — 시야 차단 후 재포착 | 5초 이내 **≥ 80%** |
| ALGO-007 | **파라미터 외부화** | 코드 내 하드코딩 임계값 **0건** |
| ALGO-008 | **대조군 대비 유의성** — 알고리즘 ON vs 로봇 정지/무작위 조건의 성공률 차이 | **≥ 40 %p 차이**, 카이제곱 검정 p < 0.05 |

**ALGO-008이 가장 중요하다.** 이것이 "표적이 우연히 구역에 들어간 것이 아니라 알고리즘이 몰았다"를 증명하는 유일한 근거다. 반드시 대조군 실험을 자동화해 리포트에 포함시켜라.

### 4-2. 오프라인 시뮬레이터 (`test/simulator.py`)
ROS 없이 실행되는 2D 시뮬레이터. 벽/장애물이 있는 맵에서 로봇 2대와 표적 1개를 물리 시뮬레이션한다.
- 로봇은 최대 속도 제한이 있는 점질량 모델 (TurtleBot4 기준 0.3 m/s 정도)
- 표적은 로봇보다 빠르게 설정 (0.4 m/s) — 몰이가 실제로 어려워야 검증 의미가 있다
- 랜덤 시드로 표적 초기 위치를 바꿔가며 100회 반복

### 4-3. 검증 리포트 (`test/run_validation.py`)
출력 형식 예시:
```
=== Evasion Model: reactive_flee ===
  trials: 100 | success: 78.0% | mean time: 42.3 s | panic rate: 6.0%
  role swaps/trial: 3.2 | mean latency: 18.4 ms
  ALGO-001 (>=70%): PASS
  ...
=== Model Comparison ===
  reactive_flee : 78.0%   wall_hugger : 71.0%
  noisy_human   : 64.0%   <- 실물 시연 예상치
  random_walk   :  8.0%   <- 대조군 (우연 성공률)
=== Control Experiment (ALGO-008) ===
  algorithm ON : 78.0%  |  robots idle : 6.0%  |  robots random : 9.0%
  difference   : +69 %p  |  chi-square p = 0.0001  -> PASS
=== SUMMARY ===
  ALGO-001 PASS / ... / ALGO-008 PASS
```
추가로 matplotlib으로 다음을 `test/output/`에 저장:
- 표적 궤적 + 로봇 2대 궤적 + 포획 구역을 겹친 경로도 (성공/실패 각 1건)
- 도주 확률 히트맵 스냅샷
- 성공률 vs 주요 파라미터 민감도 그래프 (`drive_distance_m`, `robot_repulsion_weight` 최소 2개)

### 4-5. 실물 미니카 시연 프로토콜 (사람 조종 편향 통제)
가용 장비는 미니카 + RC 리모컨뿐이므로, 사람이 조종한다. 조종자 편향을 통제하기 위해 아래를 **문서와 도구로 구현**해라.

**(a) 조종자 눈가림 (Blinding) — 필수**
- 조종자에게 **포획 구역 위치를 알려주지 않는다.**
- 매 시행마다 포획 구역을 사전 정의된 4개 후보 중 **무작위로 선택**한다.
- 실행 스크립트가 구역을 랜덤 선택하고, **콘솔에 출력하지 않고 로그 파일에만 기록**하도록 구현할 것.

**(b) 도주 규칙 카드 — 조종자가 지켜야 할 3원칙**
1. 평상시: 벽면을 따라 일정 속도로 이동한다. 공간 중앙으로 나가지 않는다.
2. 로봇이 약 1 m 안으로 들어오면: 즉시 반대 방향으로 급가속해 도망친다.
3. 3~5초 이동마다 1~2초 정지한다.
→ 이 카드를 `docs/operator_protocol.md` 로 출력물 형태로 작성해라.

**(c) 대조군 실험 — 필수**
동일 조종자·동일 규칙으로 아래 3조건을 각 10회씩 시행한다.
| 조건 | 로봇 동작 | 목적 |
|---|---|---|
| CONTROL-A | 로봇 정지 | 우연 성공률 baseline |
| CONTROL-B | 로봇 무작위 순찰 | 단순 움직임의 효과 분리 |
| TREATMENT | 몰이 알고리즘 ON | 실제 성능 |

**(d) 실험 기록 도구 — `test/field_logger.py`**
ROS bag과 별개로, 시행별 결과를 CSV에 자동 기록하는 스크립트를 구현해라.
기록 항목: `trial_id, condition, capture_zone_id(비공개), start_time, end_time, success(bool), duration_sec, min_robot_target_dist, rule_violation_count, note`
- `rule_violation_count`: 사후에 로그를 분석해 조종자가 규칙 (b)를 어긴 횟수를 자동 판정한다.
  (예: 로봇이 1m 내 접근했는데 표적이 로봇 쪽으로 이동한 경우 = 규칙 2 위반)
- 규칙 위반률이 20%를 넘는 시행은 **분석에서 제외**하고 리포트에 명시한다.

**(e) 실측 궤적 → 시뮬레이터 피드백**
기록한 표적 궤적 CSV를 `log_replay` 모델로 재생해, 시뮬레이션 예측치(`noisy_human`)와 실측이 얼마나 일치하는지 비교표를 낸다. 차이가 크면 `noisy_human`의 반응 지연·노이즈 파라미터를 실측에 맞춰 보정해라.

### 4-4. 단위 테스트
- 격자 <-> map 좌표 변환 왕복 일치성
- Driving Point가 항상 목표의 반대편에 위치하는가
- panic_distance 침범 시 후퇴 명령이 나오는가
- 역할 교대가 cooldown 내에 두 번 일어나지 않는가
- 장애물 셀 전이확률이 0인가
- FSM 상태 전이가 명세와 일치하는가 (모든 전이 경로 커버)

---

## 5. 코드 품질 요구사항
- Python 3.10+, PEP8, 모든 public 함수에 타입 힌트와 docstring
- 상태/설정은 `@dataclass`로 명시적으로 정의
- 예외 처리: 격자 범위 밖 좌표, 관측 소실, `/map` 미수신, 0으로 나누기(정규화)
- numpy 벡터 연산 사용. 격자 순회에 이중 for 루프 금지 (ALGO-005)
- 코어는 표준 `logging`, 노드는 rclpy logger

---

## 6. 진행 방식
1. 전체 구현 계획을 먼저 요약해 보여주고 **내 확인을 받은 뒤** 코딩을 시작해라.
2. 패키지 뼈대 + `herding_params.yaml` 작성
3. 코어를 순서대로 구현하고 각 단계마다 단위 테스트 통과 후 진행:
   `grid_map` → `target_estimator` → `escape_model` → `herding_planner` → `role_assigner` → `state_machine` → `occlusion_grid` → `herding_core`
4. 시뮬레이터 + 도주 모델 5종(`reactive_flee`, `wall_hugger`, `noisy_human`, `random_walk`, `log_replay`) 구현
5. `run_validation.py` 실행 → ALGO-001~008 전부 PASS 확인. 대조군 실험과 카이제곱 검정을 포함할 것.
   **미달 시 파라미터를 튜닝하고 튜닝 근거를 설명해라.** 특히 `drive_distance_m`과 `robot_repulsion_weight`가 성공률에 가장 민감할 것으로 예상된다.
6. 마지막에 `herding_node.py` 구현
7. `docs/operator_protocol.md` (조종자 규칙 카드)와 `test/field_logger.py` (실험 기록 도구)를 작성
8. 완료 후 보고:
   - ALGO-001~008 실제 측정값 및 PASS/FAIL
   - 도주 모델 5종별 성능 비교표 + 대조군 대비 유의성 검정 결과
   - 실물 시연 예상 성공률 (`noisy_human` 모델 기준)
   - 파라미터 튜닝 내역과 근거
   - 본 프로젝트 통합 체크리스트 (어떤 토픽을 누구와 연결해야 하는지, 필요한 인터페이스 계약)
   - 확인 필요 항목 (내가 결정해야 할 것들)
