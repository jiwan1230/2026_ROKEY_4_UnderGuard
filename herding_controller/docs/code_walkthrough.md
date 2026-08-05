# 코드 리뷰용 워크스루 — 파일별 알고리즘 기능 정리

이 문서는 `herding_controller` 패키지의 모든 소스 파일을 훑으면서, **어떤 파일의 어떤
클래스/함수가 몰이(Herding) 알고리즘의 어떤 기능을 담당하는지**를 정리한 것입니다.
코드 리뷰 때 "이 부분은 뭘 하는 코드냐"는 질문에 바로 찾아가기 위한 지도용 문서이며,
각 파일 자체에는 이미 WHY 중심의 상세 주석이 달려 있으니 이 문서는 그 주석들을
찾아가는 색인이자 전체 그림 요약으로 봐 주세요.

관련 문서: [설계 스펙](../../docs/superpowers/specs/2026-08-04-herding-controller-design.md) ·
[구현 플랜](../../docs/superpowers/plans/2026-08-04-herding-controller.md) ·
[최종 검증 리포트](../../docs/superpowers/plans/2026-08-04-herding-controller-final-report.md) ·
[작업 요약](../../herding_controller_작업요약.md)

---

## 1. 아키텍처 원칙

```
herding_controller/herding_controller/   <- 알고리즘 코어. ROS 의존성 0.
    grid_map.py          격자 좌표 변환 + 장애물 마스크
    target_estimator.py  칼만 필터 (표적 위치/속도 추정)
    escape_model.py       마르코프 도주 방향 예측
    herding_planner.py    Driving/Blocking Point 계산
    role_assigner.py      Driver/Blocker 동적 배정 (이력현상)
    state_machine.py      6+1 상태 FSM
    occlusion_grid.py     LOST 상태 재탐색용 belief 그리드
    herding_core.py       위 전부를 조합하는 파사드 (핵심 오케스트레이션)
    herding_node.py       <- 패키지에서 유일하게 rclpy를 import하는 파일

test/                     <- 오프라인 검증 하네스 (역시 ROS 의존성 0, herding_node.py 제외)
    evasion_models/*.py    표적(쥐) 역할 대리 모델 5종
    simulator.py           헤드리스 2D 물리 시뮬레이터
    run_validation.py      ALGO-001~008 통계 검증 + 대조군 실험
    field_logger.py        실물 시연 CSV 로깅 + 블라인드 포획구역 선택
```

**왜 이렇게 나뉘는가:** `herding_core.py`와 그 아래 7개 모듈은 미션 매니저/Nav2 등
팀의 다른 ROS 컴포넌트와 통합되기 전에 순수 파이썬으로 오프라인 검증이 가능해야
한다는 게 이 프로젝트의 핵심 제약입니다. `herding_node.py`는 그 위에 얹힌 얇은
rclpy 어댑터 한 장뿐입니다 — 여기서 로직을 추가하면 오프라인 검증이 깨집니다.

---

## 2. 한 제어 주기의 데이터 흐름 (`HerdingCore.step()` 기준)

`herding_core.py:238` `step()`이 매 제어 주기(기본 5Hz)마다 실행하는 순서:

1. **장애물 맵 갱신** — `Observation.occupancy`가 있으면 `grid_map.py`의
   `set_obstacle_mask_from_occupancy()`로 반영.
2. **표적 위치/속도 추정** — `target_estimator.py`의 칼만 필터에 관측값을
   `update()`(관측 있음) 또는 `predict()`만(관측 없음) 호출.
3. **도주 방향 예측** — KF가 수렴했으면 `escape_model.py`의
   `EscapeModel.compute()`로 8방향 확률 분포 계산.
4. **FSM 전이** — `state_machine.py`의 `HerdingStateMachine.step()`으로
   IDLE→SEARCH→TRACK→HERD→CORNER→CAPTURED(+LOST) 중 다음 상태 결정.
5. **상태별 분기**:
   - `LOST`: `occlusion_grid.py`의 belief 그리드를 확산/감쇠시키고, 최고
     확률 셀을 재탐색 목표로 두 로봇에게 준다.
   - `HERD`/`CORNER`: `role_assigner.py`로 Driver/Blocker 배정 →
     `herding_planner.py`의 `compute_driving_point()`/`compute_blocking_point()`로
     각 로봇의 목표 좌표 계산 → `resolve_separation()`으로 두 목표점이
     너무 가깝지 않게 밀어냄.
   - 그 외(`IDLE`/`SEARCH`/`TRACK`/`CAPTURED`): 로봇은 현재 위치를 유지.
6. **결과 반환** — `HerdingOutput`(로봇 목표 좌표 2개, FSM 상태, driver/blocker id,
   도주 확률, 패닉/역할교체 플래그, 지연시간)을 만들어 반환.

`herding_node.py`는 이 `step()`을 ROS 타이머 콜백에서 호출하고, 결과를
`~/robot1_goal`, `~/robot2_goal`, `~/herding_state`, `~/escape_probability`,
`~/capture_result` 토픽으로 발행하는 게 전부입니다.

---

## 3. 파일별 상세

### `grid_map.py` — 격자 좌표계

| 클래스/함수 | 담당 기능 |
|---|---|
| `GridConfig` | 격자 해상도·크기·원점 |
| `GridMap.world_to_cell` / `cell_to_world` | 미터 좌표 ↔ (row, col) 셀 인덱스 변환. 다른 모든 모듈이 "장애물인가"를 물을 때 거치는 공통 좌표계 |
| `GridMap.set_obstacle_mask_from_occupancy` | ROS `OccupancyGrid` 메시지(0~100 점유율)를 불리언 장애물 마스크로 변환 |
| `GridMap.is_obstacle` / `in_bounds` | 셀 하나가 장애물인지/격자 범위 안인지 |

알고리즘상 역할: 다른 모든 모듈(도주 모델, 플래너, occlusion grid)이 "이 좌표가
벽인가"를 판단할 때 공유하는 단일 진실 공급원.

### `target_estimator.py` — 칼만 필터 (표적 추적)

| 클래스/함수 | 담당 기능 |
|---|---|
| `TargetEstimator` | 등속도(constant-velocity) 모델 4상태([x,y,vx,vy]) 칼만 필터 |
| `.predict(dt)` | 관측 없이 상태를 dt만큼 전진 (LOST 상태에서도 계속 호출됨) |
| `.update(measurement)` | 새 (x,y) 관측을 칼만 이득으로 융합 |
| `.get_state()` | 현재 위치/속도 추정치 + `is_lost` 플래그(occlusion_timeout_sec 초과 여부) |

알고리즘상 역할: 원시 센서 좌표의 잡음을 걸러내고, 위치뿐 아니라 **속도**까지
추정해서 escape model의 관성(momentum) 항과 planner의 "이미 목표 쪽으로
가고 있는지" 판단에 넘겨줌.

### `escape_model.py` — 마르코프 도주 방향 예측

| 클래스/함수 | 담당 기능 |
|---|---|
| `_DIRECTIONS` | 8방위(N/NE/E/SE/S/SW/W/NW) 단위벡터 |
| `EscapeModel.compute()` | 표적이 각 방향으로 도주할 확률 분포 계산 |
| `._base_weights()` | 벽과의 관계(따라가기/붙기/중앙으로) 기본 확률 — 향촉성(thigmotaxis) |
| `._robot_repulsion()` | 가까운 로봇일수록 그 반대 방향 확률을 높임 (거리 반비례) |
| `._momentum()` | 현재 속도 방향으로 계속 가려는 관성 가중치 |
| `._mask_obstacles()` | 장애물 쪽 방향 확률을 0으로 — 완전히 막히면 균등분포로 대체(디제너러시 방지) |
| `._top_k_routes()` | 확률 상위 K개 방향의 짧은 미리보기 경로 (RViz 시각화용) |

알고리즘상 역할: "표적이 다음에 어디로 도망칠 것 같은가"를 확률분포로 내놓는
예측 모델. Blocker의 목표 지점(`compute_blocking_point`)이 바로 이 분포를 읽고
가장 유력한 도주로를 선점한다.

### `herding_planner.py` — 로봇 목표점 계산

| 클래스/함수 | 담당 기능 |
|---|---|
| `compute_driving_point()` | Driver 목표점 = 표적 뒤쪽(포획구역 반대편). 표적이 이미 목표 쪽으로 가고 있으면 `drive_distance_ease_factor`로 느슨하게, 로봇이 `panic_distance_m` 안으로 너무 붙으면 후퇴(retreat) 지점 반환 |
| `compute_blocking_point()` | Blocker 목표점 = escape model이 예측한 최고확률 도주 경로 중 "목표 반구 밖"에 있는 것을 선점. 막혀 있으면 차선책으로 완화 |

알고리즘상 역할: 예측(escape_model)과 상태(FSM)를 실제 로봇이 가야 할 (x, y)
좌표로 변환하는 곳. **이 두 함수의 반환값이 곧 로봇에게 발행되는 목표 좌표**다.

### `role_assigner.py` — Driver/Blocker 동적 배정

| 클래스/함수 | 담당 기능 |
|---|---|
| `RoleAssigner.assign()` | 두 로봇 중 어느 쪽이 Driver인지 결정. `role_swap_margin`(비용 차이 임계값) + `role_swap_cooldown_sec`(최소 유지 시간) 둘 다 넘어야 교체 — 역할 진동(chattering) 방지 |
| `._cost()` | 후보 로봇이 목표점까지 가는 "수고" = 직선거리 + 회전각 가중치 |
| `resolve_separation()` | Driver/Blocker 목표점이 `min_robot_separation_m`보다 가까우면 Blocker 쪽을 밀어냄 (로봇 간 충돌 방지) |

알고리즘상 역할: "누가 밀고 누가 막을지"를 매 주기 재계산하되, 히스테리시스로
불필요한 역할 교체를 억제. `herding_core.py`의 `_nominal_driving_point()`가
이 함수에 넘길 로봇 중립적인 후보점을 별도로 계산한다는 점이 포인트(안 그러면
우연히 가까운 로봇 쪽으로 배정이 편향됨).

### `state_machine.py` — 몰이 진행 상태 FSM

| 클래스/함수 | 담당 기능 |
|---|---|
| `FSMState` | IDLE / SEARCH / TRACK / HERD / CORNER / CAPTURED / LOST |
| `HerdingStateMachine.step()` | 이번 주기 신호(`FSMInputs`)로 다음 상태 계산. 상태별 "전진" 전이 + 상태 무관 감시 2개(①관측 끊김→LOST, ②포획 반경 체류 타이머→CAPTURED)로 구성 |

알고리즘상 역할: 전체 미션의 진행 단계를 관리하는 중앙 상태기계. HERD↔CORNER
전이는 위치 조건(포획반경 안)과 확률 조건(escape 분포가 한쪽으로 집중,
`escape_concentration_threshold`)을 **둘 다** 요구해서, 반경 안에 있어도
여전히 도망갈 구석이 많으면 성급하게 "구석에 몰렸다"고 판정하지 않는다.

### `occlusion_grid.py` — LOST 상태 재탐색

| 클래스/함수 | 담당 기능 |
|---|---|
| `OcclusionGrid.seed()` | 마지막으로 알려진 표적 셀에 belief를 1.0으로 집중 |
| `.step(dt)` | 4방향 확산(질량 보존, 안정성 상한 0.25로 클램프) + 장애물 마스킹 + 전역 감쇠 |
| `.best_guess_cell()` | belief가 가장 높은 셀 = 재탐색 목표 |

알고리즘상 역할: 표적이 시야에서 사라졌을 때(LOST) "어디 있을 가능성이
가장 높은가"를 시간에 따라 흐려지는 확률 지도로 추적. (검증 결과 ALGO-006
기준 미달 — 이동 방향을 반영하지 못하는 구조적 한계로 기록됨, 작업요약 참고)

### `herding_core.py` — 오케스트레이션 파사드

| 클래스/함수 | 담당 기능 |
|---|---|
| `HerdingConfig` | 모든 튜닝 파라미터를 담는 flat dataclass + `__post_init__`에서 `drive_distance_m * drive_distance_ease_factor < flee_reaction_distance_m` 불변식 검증 |
| `Observation` / `HerdingOutput` | 코어의 입력/출력 데이터 타입 (numpy만 사용, ROS 메시지 타입 없음) |
| `HerdingCore.step()` | 위 6개 모듈을 순서대로 호출하는 메인 루프 (§2 참고) |
| `._nominal_driving_point()` | role assigner에 넘길 로봇 중립적 후보점 |
| `._search_point()` | LOST 상태에서 belief 최고점 또는 마지막 관측 위치로 폴백 |

알고리즘상 역할: 이 프로젝트의 핵심 아키텍처 제약(ROS 의존성 0)을 지키는
경계선이자, 개별 모듈들을 올바른 순서로 엮는 곳.

### `herding_node.py` — ROS2 어댑터

| 클래스/함수 | 담당 기능 |
|---|---|
| `_load_config()` | ROS 파라미터 → `HerdingConfig` |
| `HerdingNode` | `~/target_pose`, `~/robot{1,2}_pose`, `/map` 구독 → `HerdingCore.step()` 호출 → `~/robot{1,2}_goal`, `~/herding_state`, `~/escape_probability`, `~/capture_result` 발행 |
| `_rasterize_escape_probabilities()` | 8방향 확률을 RViz `OccupancyGrid`로 그리기 위한 래스터화 |
| `_quaternion_to_heading()` | 로봇 orientation 쿼터니언 → 2D heading 벡터 |

알고리즘상 역할: 순수 로직(`HerdingCore`)과 ROS 세계를 잇는 유일한 지점.
여기엔 알고리즘 로직이 없어야 하며, 실제로 없다 — 전부 위임(delegate)뿐.

---

## 4. 검증 하네스 (`test/`)

### `evasion_models/*.py` — 표적(쥐) 역할 대리 모델 5종

| 파일 | 역할 |
|---|---|
| `base.py` | 모든 모델의 추상 인터페이스 (`step()` → 다음 속도 벡터) |
| `reactive_flee.py` | 주 검증 모델. 반경 안 로봇으로부터 곧장 도망 |
| `wall_hugger.py` | 위협 없을 때 벽을 따라가고, 위협 시 반발(위 모델 재사용) |
| `noisy_human.py` | `wall_hugger`를 감싸 반응 지연 + 노이즈 추가 — **사람이 조종하는 실물 미니카에 가장 가까운 근사**, 실물 시연 예상치로 사용 |
| `random_walk.py` | 로봇을 완전히 무시 — ALGO-008 대조군(우연 성공률 baseline) |
| `log_replay.py` | 실제 기록된 궤적 CSV 재생 — 시뮬레이션 대 현장 실험 비교용 |

### `simulator.py` — 헤드리스 2D 물리 시뮬레이터

| 함수 | 역할 |
|---|---|
| `run_trial()` | 로봇 2대+표적을 점질량으로 물리 시뮬레이션 하며 매 스텝 `HerdingCore.step()` 호출. `control_mode`로 `algorithm`/`idle`/`random` 전환 가능(ALGO-008용) |
| `_step_body()` | 위치를 제안 지점으로 이동시키되 벽 충돌 시 정지 |
| `_advance_target()` | 도주 모델이 명령한 속도를 적분해 표적을 이동 |

### `run_validation.py` — 통계 검증

| 함수 | 역할 |
|---|---|
| `run_algo_suite()` | 4개 도주모델 × N회 시행 → ALGO-001~005/007 판정, `_run_occlusion_recovery_check`(ALGO-006), `_run_control_experiment`(ALGO-008 카이제곱 검정), `_run_sensitivity_sweep`(파라미터 민감도) |
| `_write_report()` / `_write_plots()` | `test/output/`에 텍스트 리포트 + 궤적/민감도 그래프 저장 |

### `field_logger.py` — 실물 시연 지원

| 함수 | 역할 |
|---|---|
| `select_capture_zone()` | 4개 후보 중 포획구역을 무작위 선택 (조종자에게 절대 노출 안 함 — 블라인딩) |
| `FieldLogger.log_trial()` | 시행별 CSV 기록 |
| `detect_rule_violations()` | 조종자가 `operator_protocol.md`의 규칙 2(로봇 1m 이내 접근 시 도망)를 어겼는지 사후 자동 판정 |

---

## 5. 이 워크스루에 포함되지 않은 것

최근 대화에서 만든 **로봇 1대 몰이 실험(허더/트래커 분리, 마르코프+가치함수
결합 정책)**은 `/tmp` 스크래치 디렉터리의 별도 스크립트로만 존재하며, 위 정식
코드베이스(`herding_controller/`, `test/`)에는 포함되어 있지 않습니다. 검증되면
정식 통합하기로 한 상태이니, 코드 리뷰 범위에는 들어가지 않는 게 맞습니다.
