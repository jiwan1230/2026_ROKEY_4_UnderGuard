# UnderGuard AMR — 2대 협업 표적 몰이(Herding) 알고리즘 패키지 설계

## 0. 출처와 승인
- 원본 요구사항: `prompt_herding_controller.md` (사용자 작성, 사실상 완결된 스펙)
- 승인된 결정: 최상위 프로젝트 폴더명 `Intelligence1_Algorithm`, 그 안에 ROS2 ament_python 패키지 `herding_controller/`
- 환경 확인 완료: ROS2 Humble, colcon, numpy 1.24.4, scipy 1.15.3, matplotlib 3.10.9 설치되어 있음

## 1. 프로젝트 배경
- 프로젝트명: UnderGuard AMR (실내 순찰형 자율이동로봇)
- 시나리오: 로봇 2대가 협업해 실시간으로 표적(쥐, 대역: 미니카)을 추적하고 미리 지정된 포획 구역으로 몰아넣는다.
- 이 저장소는 몰이 알고리즘 파트만 담당한다. 나중에 팀의 perception 노드, mission manager, Nav2 스택과 결합할 예정이므로 결합도를 최대한 낮춘다.
- 기존 환경: TurtleBot4, namespace `/robot6` 계열, `ROS_DOMAIN_ID=6`, Nav2 `NavigateToPose`, OAK-D RGB-D.

### 확정된 설계 결정
1. 목표 지점: 맵 상에 고정된 포획 구역 1곳 (파라미터로 좌표+반경 지정).
2. 로봇 역할: Driver / Blocker를 상황에 따라 동적으로 교대 (고정 배정 아님).
3. 표적 하드웨어 제약: 미니카 + RC 리모컨만 사용 가능, 자율 도주 로직 탑재 불가.
   - 검증은 오프라인 시뮬레이터에서 통계적으로 수행 (100회 시행).
   - 미니카는 시연/소규모 실물 확인용 (10회 시행).
   - 사람이 조종하되 도주 규칙 프로토콜 + 조종자 눈가림 + 대조군 실험으로 편향 통제.
   - 도주 모델은 교체 가능한 플러그인 구조로 설계.

## 2. 알고리즘 설계 (그대로 따름)

### 2-0. 기본 원리 — Shepherding
표적은 로봇이 다가오면 로봇의 반대 방향으로 도망친다. 표적을 목표 G로 보내려면 로봇은 G의 정반대편(표적 뒤쪽)에서 밀어야 한다.

### 2-1. 표적 상태 추정 (target_estimator.py)
- 상수속도 모델 칼만 필터로 위치+속도 벡터 추정.
- 관측 끊김(occlusion) 시 예측 외삽, `occlusion_timeout_sec` 초과 시 LOST 전환.

### 2-2. 도주 방향 예측 (escape_model.py)
- 격자 기반 마르코프 전이확률로 도주 방향 확률 산출.
- 벽면 선호(thigmotaxis): 벽 따라 직진 0.70 / 벽 밀착 0.20 / 개방공간 진출 0.10.
- 로봇 반발 항(거리 반비례 가중), 관성 항(momentum_weight), 정적 장애물 셀 전이확률 0 마스킹.
- 출력: 방향별 확률 분포 + 상위 K개 도주로 좌표.

### 2-3. Driving Point 계산 (herding_planner.py)
```
u = normalize(target_pos - goal_pos)
P_drive = target_pos + drive_distance_m * u
```
- `drive_distance_m`은 `r_flee`보다 약간 작게 설정.
- 패닉 방지 하한: 거리 < `panic_distance_m`이면 접근 중단, 후퇴.
- `alignment_threshold` 이상 정렬 시 drive_distance 확대(압박 완화).

### 2-4. Blocking Point 계산 (herding_planner.py)
1. 도주 확률 분포에서 목표 G 방향 반구 제외.
2. 남은 방향 중 확률 최댓값 도주로 선택.
3. 해당 경로상 좌표를 Blocking Point로 선점.
4. 이미 장애물이 막고 있으면 차선 확률 경로로 대체.

### 2-5. 역할 동적 배정 (role_assigner.py)
1. 각 로봇의 Driving Point까지 비용(거리+방향전환비용) 계산.
2. 비용 낮은 로봇을 Driver로 배정.
3. 히스테리시스 필수: `role_swap_margin` 이상 차이 + `role_swap_cooldown_sec` 경과 시에만 교대.
4. 두 로봇 목표가 `min_robot_separation_m` 미만이면 Blocker 목표를 밀어냄.

### 2-6. 상태 기계 (state_machine.py)
```
IDLE → SEARCH → TRACK → HERD → CORNER → CAPTURED
                  ↑        ↓        ↓
                  └──── LOST ←──────┘
```
SEARCH(미관측)/TRACK(관측 확보, KF 수렴 대기)/HERD(KF 수렴+거리>capture_radius)/
CORNER(포획구역 근처+확률분산 낮음)/CAPTURED(반경 내 capture_hold_sec 이상 체류)/LOST(occlusion_timeout 초과).

### 2-7. 확률 지도 (occlusion_grid.py)
LOST 상태 전용 축소 베이지안 격자. 확산(diffusion)+감쇠(decay). TRACK/HERD에서는 미사용.

## 3. 구현 요구사항

### 3-1. 패키지 구조
```
herding_controller/
├── package.xml
├── setup.py
├── config/herding_params.yaml
├── herding_controller/
│   ├── __init__.py
│   ├── grid_map.py
│   ├── target_estimator.py
│   ├── escape_model.py
│   ├── herding_planner.py
│   ├── role_assigner.py
│   ├── occlusion_grid.py
│   ├── state_machine.py
│   ├── herding_core.py          # 파사드, rclpy import 금지
│   └── herding_node.py          # rclpy는 여기서만 import
└── test/
    ├── evasion_models/{base,reactive_flee,wall_hugger,line_tracer,human_replay}.py
    ├── simulator.py
    ├── run_validation.py
    └── test_*.py
```
최우선 제약: `herding_core.py`와 하위 모듈은 `rclpy` 절대 import 금지. 입출력은 순수 파이썬 자료형만 사용.

### 3-2. 도주 모델 플러그인 인터페이스
```python
class EvasionModel(ABC):
    @abstractmethod
    def step(self, target_state, robot_positions, obstacle_map, dt) -> np.ndarray: ...
```
구현: `reactive_flee`(주 검증용), `wall_hugger`(검증용), `noisy_human`(실물 근사, 반응지연 0.3~0.8s+노이즈),
`random_walk`(대조군), `log_replay`(실측 CSV 재생).

### 3-3. ROS2 인터페이스 (herding_node.py)
Subscribe: `~/target_pose`, `~/robot1_pose`, `~/robot2_pose` (PoseStamped), `/map` (OccupancyGrid, transient_local).
Publish: `~/robot1_goal`, `~/robot2_goal` (PoseStamped), `~/herding_state` (String/JSON),
`~/escape_probability` (OccupancyGrid), `~/capture_result` (Bool).
제어 주기 `control_rate_hz` 기본 5.0Hz. Nav2 액션 직접 호출 금지 — 목표 좌표만 발행.

### 3-4. config/herding_params.yaml
문서 3-4에 명시된 전체 파라미터를 그대로 사용 (매직넘버 금지).

## 4. 검증 요구사항

### 4-1. 정량 합격 기준 (ALGO-001~008)
| ID | 요구사항 | 합격 기준 |
|---|---|---|
| ALGO-001 | 몰이 성공률 | 100회 시행 ≥ 70% |
| ALGO-002 | 평균 포획 소요 시간 | ≤ 60초(sim time) |
| ALGO-003 | 패닉 발생률 | ≤ 10% |
| ALGO-004 | 역할 진동 없음 | 시행당 ≤ 5회 |
| ALGO-005 | 제어 주기 지연 | 1사이클 ≤ 100ms (40×40 격자) |
| ALGO-006 | Occlusion 복구 | 5초 이내 ≥ 80% |
| ALGO-007 | 파라미터 외부화 | 하드코딩 임계값 0건 |
| ALGO-008 | 대조군 대비 유의성 | ≥40%p 차이, 카이제곱 p<0.05 (최우선) |

### 4-2. 오프라인 시뮬레이터 (test/simulator.py)
ROS 없는 2D 시뮬레이터. 로봇 최대속도 0.3m/s(TurtleBot4 기준), 표적 0.4m/s(로봇보다 빠름). 랜덤 시드 100회.

### 4-3. 검증 리포트 (test/run_validation.py)
도주 모델별 결과 + 모델 비교표 + 대조군 실험(ALGO-008, 카이제곱 검정) + SUMMARY 출력.
matplotlib으로 `test/output/`에 저장: 궤적도(성공/실패 각 1건), 도주확률 히트맵, 파라미터 민감도 그래프
(`drive_distance_m`, `robot_repulsion_weight` 최소 2개).

### 4-4. 단위 테스트
격자<->map 좌표 왕복 일치, Driving Point가 항상 목표 반대편, panic_distance 침범 시 후퇴,
role cooldown 내 재교대 없음, 장애물 셀 전이확률 0, FSM 전이 명세 일치(모든 경로 커버).

### 4-5. 실물 미니카 시연 프로토콜
- 조종자 눈가림: 포획구역 4개 후보 중 랜덤 선택, 콘솔 미출력·로그 파일에만 기록.
- `docs/operator_protocol.md`: 도주 규칙 카드 3원칙.
- 대조군 실험: CONTROL-A(정지)/CONTROL-B(무작위 순찰)/TREATMENT(알고리즘 ON) 각 10회.
- `test/field_logger.py`: 시행별 CSV 기록(`trial_id, condition, capture_zone_id, start_time, end_time,
  success, duration_sec, min_robot_target_dist, rule_violation_count, note`), 규칙위반 자동판정,
  위반률 20% 초과 시행은 분석 제외.
- `log_replay`로 실측 궤적 재생 → `noisy_human` 예측치와 비교표, 필요 시 파라미터 보정.

## 5. 코드 품질 요구사항
Python 3.10+, PEP8, 모든 public 함수 타입힌트+docstring. 상태/설정은 `@dataclass`.
예외 처리: 격자 범위 밖 좌표, 관측 소실, `/map` 미수신, 0으로 나누기. numpy 벡터 연산(이중 for 루프 금지).
코어는 표준 `logging`, 노드는 rclpy logger.

## 6. 빌드 순서 (합의됨)
1. 패키지 뼈대 + `herding_params.yaml`
2. `grid_map` → `target_estimator` → `escape_model` → `herding_planner` → `role_assigner` →
   `state_machine` → `occlusion_grid` → `herding_core` (단계별 단위 테스트 통과 후 진행)
3. `simulator.py` + 도주 모델 5종
4. `run_validation.py` 실행, ALGO-001~008 PASS 확인 (미달 시 파라미터 튜닝, 근거 기록)
5. `herding_node.py`
6. `docs/operator_protocol.md`, `test/field_logger.py`
7. 완료 보고: ALGO 실측값/PASS-FAIL, 모델별 성능 비교표, 실물 예상 성공률(noisy_human 기준),
   파라미터 튜닝 근거, 통합 체크리스트, 확인 필요 항목

## 7. 확인 필요 항목 (구현 중 발견 시 최종 보고에 포함)
문서 자체가 "애매한 부분은 구현 후 확인 필요 항목으로 보고"를 명시했으므로,
사전 질문 없이 합리적 기본값으로 진행하고 발견된 모호성은 최종 보고서에 정리한다.
