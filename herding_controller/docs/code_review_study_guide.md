# 코드 리뷰 스터디 가이드 — `herding_controller`

> **이 문서의 목적**: 팀 코드 리뷰에서 발표자가 "이 알고리즘이 왜 이렇게 동작하는가"를
> 자기 말로 설명할 수 있도록 준비하는 자료입니다. PPT 대신 이 문서 + 실제 코드 +
> `code_walkthrough.md`(파일별 색인)를 함께 펼쳐 놓고 리뷰를 진행하면 됩니다.
>
> 읽는 순서 추천: **1 → 2 → 3(설계도) → 4(코드) → 5(테스트) → 6(Nav2) → 7(Q&A)**.
> 이 순서는 Code Review 안내의 "설계도 → 코드 → 테스트결과 → Q&A" 발표 순서와 동일합니다.

---

## 1. 한 줄 요약

**"로봇이 표적을 직접 붙잡는 게 아니라, 표적이 로봇을 피해 도망치는 반응 자체를
역이용해서 원하는 방향(포획존)으로 유도한다."**

- 로봇 A(**Driver**)는 표적 뒤쪽, 포획존과 정반대 지점에 자리를 잡는다. 표적은
  다가오는 로봇에게서 도망치므로, Driver가 그 자리에 있으면 표적은 자연히
  포획존 쪽으로 밀려난다.
- 로봇 B(**Blocker**)는 표적이 포획존이 아닌 다른 방향으로 새어나갈 것 같은
  경로를 예측해서 미리 막아선다.
- 이 둘을 언제 바꿔 맡을지(**역할 배정**), 지금이 "추적 중"인지 "구석에
  몰렸는지"(**상태기계**), 표적을 놓쳤을 때 어디를 찾아봐야 할지(**재탐색**)를
  각각 별도 모듈이 담당하고, 이 전부를 매 제어 주기(5Hz)마다 순서대로
  실행하는 것이 `herding_core.py`의 `HerdingCore.step()`이다.

---

## 2. 전체 아키텍처 — 메시지 흐름

```mermaid
flowchart LR
    subgraph SENSORS["센서 / 상위 시스템"]
        TP["/robot6/herding_controller/target_pose\n(PoseStamped, OAK-D 검출 결과)"]
        R1P["~/robot1_pose (PoseStamped)"]
        R2P["~/robot2_pose (PoseStamped)"]
        MAP["/map (OccupancyGrid, transient_local)"]
    end

    subgraph NODE["herding_node.py — 이 패키지에서 유일하게 rclpy를 쓰는 파일"]
        TIMER["5Hz 타이머 콜백\n_on_timer()"]
    end

    subgraph CORE["herding_core.py — ROS 의존성 0, 순수 Python"]
        STEP["HerdingCore.step()"]
    end

    subgraph DOWNSTREAM["다운스트림 (미션 매니저 / Nav2)"]
        NAV["Nav2 NavigateToPose\n(이 패키지가 아니라\n다운스트림이 호출)"]
    end

    TP -- "구독" --> TIMER
    R1P -- "구독" --> TIMER
    R2P -- "구독" --> TIMER
    MAP -- "구독" --> TIMER
    TIMER -- "Observation" --> STEP
    STEP -- "HerdingOutput" --> TIMER
    TIMER -- "~/robot1_goal, ~/robot2_goal\n(PoseStamped, 목표 좌표만)" --> NAV
    TIMER -- "~/herding_state (String/JSON)" --> DOWNSTREAM
    TIMER -- "~/escape_probability (OccupancyGrid)" --> DOWNSTREAM
    TIMER -- "~/capture_result (Bool)" --> DOWNSTREAM
```

**핵심 경계선**: 이 패키지는 `~/robot1_goal`/`~/robot2_goal`로 **좌표만 발행**하고,
Nav2의 `NavigateToPose` 액션은 직접 호출하지 않습니다(설계 스펙 §3-3
"Nav2 액션 직접 호출 금지"). 실제 주행 경로 계획과 장애물 회피는 그 목표
좌표를 받는 다운스트림(Nav2 스택)의 책임입니다 — §6 참고.

---

## 3. 한 제어 주기의 데이터 흐름 (시퀀스)

```mermaid
sequenceDiagram
    participant Node as herding_node.py
    participant Core as HerdingCore.step()
    participant KF as target_estimator
    participant Esc as escape_model
    participant FSM as state_machine
    participant Occ as occlusion_grid
    participant Role as role_assigner
    participant Plan as herding_planner

    Node->>Core: Observation(관측값, 로봇 위치, dt)
    Core->>KF: predict(dt) / update(measurement)
    KF-->>Core: TargetState(위치, 속도, is_lost)
    alt KF 수렴 & 그리드 내부
        Core->>Esc: compute(target_pos, target_vel, robot_positions)
        Esc-->>Core: EscapeEstimate(8방향 확률분포)
    end
    Core->>FSM: step(FSMInputs)
    FSM-->>Core: 다음 상태 (IDLE/SEARCH/TRACK/HERD/CORNER/CAPTURED/LOST)

    alt 상태 == LOST
        Core->>Occ: step(dt)  (확산+감쇠)
        Occ-->>Core: best_guess_cell()
    else 상태 in (HERD, CORNER)
        Core->>Role: assign(robot 위치/헤딩, 후보점)
        Role-->>Core: (driver_id, blocker_id)
        Core->>Plan: compute_driving_point(...)
        Plan-->>Core: Driver 목표점
        Core->>Plan: compute_blocking_point(...)
        Plan-->>Core: Blocker 목표점
    end

    Core-->>Node: HerdingOutput(robot1_goal, robot2_goal, fsm_state, ...)
    Node->>Node: ~/robot1_goal, ~/robot2_goal,\n~/herding_state, ~/escape_probability,\n~/capture_result 발행
```

이 순서가 `herding_core.py`의 `HerdingCore.step()` 함수 본문 순서와 정확히
일치합니다 (코드 리뷰에서 "코드 설명" 단계에 이 다이어그램과 `step()`을
나란히 두고 한 줄씩 매칭하면 됩니다).

---

## 4. 상태기계(FSM) — 몰이 진행 단계

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> SEARCH: 부트스트랩 직후 (대기 이유 없음)
    SEARCH --> TRACK: target_observed
    TRACK --> HERD: target_observed AND kf_converged
    HERD --> CORNER: 포획반경 안 AND 도주확률 집중
    CORNER --> HERD: 위 조건 중 하나라도 깨짐
    HERD --> CAPTURED: 포획반경 안 체류가 capture_hold_sec 이상 지속
    CORNER --> CAPTURED: 포획반경 안 체류가 capture_hold_sec 이상 지속
    TRACK --> LOST: occlusion_elapsed_sec > occlusion_timeout_sec
    HERD --> LOST: occlusion_elapsed_sec > occlusion_timeout_sec
    CORNER --> LOST: occlusion_elapsed_sec > occlusion_timeout_sec
    LOST --> TRACK: target_observed (재관측, HERD로 바로 안 가고\nKF 재수렴 기다림)
    CAPTURED --> [*]
```

**왜 HERD→CORNER 전이가 위치 조건 + 확률 조건을 둘 다 요구하는가**: 포획반경
안에 있어도 사방으로 도망칠 여지가 있으면 아직 "구석에 몰렸다"고 볼 수
없습니다. `escape_model`이 예측한 확률분포의 최댓값이
`escape_concentration_threshold`(기본 0.5) 이상 — 즉 도주로가 사실상 한
방향으로 좁혀졌을 때만 CORNER로 인정합니다 (`state_machine.py:66-73`).

**왜 재관측 시 HERD가 아니라 TRACK으로 돌아가는가**: LOST 동안 칼만 필터는
관측 없이 `predict()`만 계속 호출되어 불확실성(P)이 계속 커진 상태입니다.
재관측 즉시 HERD로 점프하면 아직 신뢰할 수 없는 속도 추정치로 Driver를
움직이게 되므로, TRACK의 `kf_converged` 게이트를 다시 통과시켜 KF가
재수렴할 시간을 줍니다.

---

## 5. 메시지 인터페이스

| 방향 | 토픽 | 타입 | 발행/구독 주체 | 비고 |
|---|---|---|---|---|
| 구독 | `~/target_pose` | `geometry_msgs/PoseStamped` | Detection 파트 (표적 검출) | 관측 없는 주기에는 발행되지 않음 → `predict()`만 실행됨 |
| 구독 | `~/robot1_pose` | `geometry_msgs/PoseStamped` | AMR 로컬라이제이션 | Driver/Blocker 역할과 무관하게 "로봇 1"의 물리적 위치 |
| 구독 | `~/robot2_pose` | `geometry_msgs/PoseStamped` | AMR 로컬라이제이션 | |
| 구독 | `/map` | `nav_msgs/OccupancyGrid` | SLAM/맵 서버 | QoS `TRANSIENT_LOCAL` (맵은 한 번 발행되고 유지되는 정적 데이터) |
| 발행 | `~/robot1_goal` | `geometry_msgs/PoseStamped` | 이 노드 → Nav2 | **좌표만 발행. Nav2 액션 직접 호출 안 함** |
| 발행 | `~/robot2_goal` | `geometry_msgs/PoseStamped` | 이 노드 → Nav2 | |
| 발행 | `~/herding_state` | `std_msgs/String` (JSON) | 이 노드 → SysMon/로깅 | FSM 상태, driver/blocker id, 표적 위치/속도, panic/role_swap 플래그 |
| 발행 | `~/escape_probability` | `nav_msgs/OccupancyGrid` | 이 노드 → RViz 시각화 | 8방향 도주 확률을 그리드에 래스터화 (0~100) |
| 발행 | `~/capture_result` | `std_msgs/Bool` | 이 노드 → SysMon | FSM 상태 == CAPTURED |

제어 주기: **5.0 Hz** (`control_rate_hz` 파라미터).

---

## 6. Nav2 연동 — 이 패키지가 "하지 않는" 일

**AMR 파트는 발표자가 Nav2 API 사용법까지 설명할 수 있어야 한다는 요구사항이
있으므로, 이 경계선을 명확히 이해하고 있어야 합니다.**

- 이 패키지(`herding_controller`)는 Nav2의 `NavigateToPose` 액션 클라이언트가
  **아닙니다**. `~/robot1_goal`, `~/robot2_goal`로 목표 좌표(`PoseStamped`)만
  발행하고 끝입니다.
- 실제로 그 좌표를 받아 Nav2 액션을 호출하고, 전역/지역 경로 계획, 장애물
  회피, 로컬라이제이션까지 수행하는 것은 **다운스트림의 미션 매니저 노드**의
  책임입니다.
- 이렇게 분리한 이유(설계 제약): `herding_core.py`와 그 하위 7개 모듈이 ROS
  의존성 없이 순수 Python으로 오프라인 검증(`test/run_validation.py`의
  ALGO-001~008)이 가능해야 했기 때문입니다. Nav2 액션 호출까지 이 안에
  넣으면 ROS 환경 없이는 알고리즘 자체를 테스트할 수 없게 됩니다.
- 그래서 이 패키지의 "AMR API 사용"은 **좁습니다**: 어떤 Nav2 API를 이 패키지
  자체가 호출하는 것이 아니라, 이 패키지가 발행하는 좌표를 다운스트림이
  Nav2에 어떻게 넘기는지(NavigateToPose goal pose)를 설명하면 됩니다. Q&A에서
  "왜 Nav2를 직접 호출하지 않았나"라는 질문이 나오면 위 이유를 그대로
  답하면 됩니다.

---

## 7. 설계 ↔ 코드 ↔ 테스트 매칭

| 설계 요소 (`docs/superpowers/specs/2026-08-04-herding-controller-design.md`) | 코드 | 검증 테스트 |
|---|---|---|
| §2-1 Driving Point 공식 | `herding_planner.py: compute_driving_point()` | `test/test_herding_planner.py`, ALGO-002 |
| §2-2 Blocking Point 4단계 알고리즘 | `herding_planner.py: compute_blocking_point()` | `test/test_herding_planner.py`, ALGO-003 |
| §2-3 역할 배정 알고리즘 | `role_assigner.py: RoleAssigner.assign()` | `test/test_role_assigner.py`, ALGO-004 |
| §2-4 FSM 다이어그램 | `state_machine.py: HerdingStateMachine.step()` | `test/test_state_machine.py`, ALGO-001 |
| §3-3 메시지 인터페이스 | `herding_node.py` 구독/발행부 | `test/test_herding_node.py` (mock rclpy) |
| (칼만 필터, 스펙 §2 배경) | `target_estimator.py: TargetEstimator` | `test/test_target_estimator.py` |
| (마르코프 도주 모델, 스펙 §2 배경) | `escape_model.py: EscapeModel.compute()` | `test/test_escape_model.py`, ALGO-005 |
| (LOST 재탐색, 스펙 §2 배경) | `occlusion_grid.py: OcclusionGrid` | `test/test_occlusion_grid.py`, **ALGO-006 (79.3%, 기준 80% 미달로 기록됨)** |
| 전체 통합 | `herding_core.py: HerdingCore.step()` | `test/simulator.py` + `test/run_validation.py` (ALGO-001~008 전체) |

> 정식 코드에는 없지만 참고: `herding_controller/experiments/`의 실험 스크립트들은
> 위 정식 모듈(`compute_driving_point`/`compute_blocking_point`)을 그대로
> 재사용해 실제 SLAM 맵 위에서 2로봇 시나리오를 시각화하는 용도입니다.
> 코드 리뷰의 "코드" 항목(프로덕션 코드만) 범위에는 포함되지 않지만, "테스트
> 자료"의 시각 자료(GIF)로는 활용할 수 있습니다.

---

## 8. 모듈별 발표 대본 (구두 설명용 핵심 문장)

리뷰에서 "이 부분은 뭘 하는 코드냐"는 질문에 아래 한두 문장으로 먼저 답하고,
필요하면 코드의 상세 주석(각 파일에 직접 추가됨)을 짚어 보여주는 방식을 권장합니다.

1. **`grid_map.py`** — "모든 모듈이 '이 좌표가 벽인가'를 판단할 때 거치는
   단일 진실 공급원입니다. 미터 좌표와 격자 셀 인덱스를 서로 변환합니다."
2. **`target_estimator.py`** — "표적의 위치를 등속도 칼만 필터로 추적합니다.
   센서는 위치만 주지만, 필터는 위치 변화로부터 속도까지 함께 추정해서
   도주 방향 예측과 '이미 목표 쪽으로 가고 있는지' 판단에 씁니다."
3. **`escape_model.py`** — "표적이 다음에 어느 방향으로 도망칠지 8방향 확률로
   예측합니다. 벽을 따라 도망치는 습성, 로봇으로부터의 반발, 관성 세 요인을
   더해서 계산합니다."
4. **`herding_planner.py`** — "이 예측을 실제 로봇의 목표 좌표로 바꾸는
   곳입니다. Driver는 표적 뒤쪽에, Blocker는 예측된 도주로 중 가장
   유력한 곳을 선점합니다."
5. **`role_assigner.py`** — "두 로봇 중 누가 Driver를 맡을지 매 주기 재계산하되,
   비용 차이와 최소 유지시간 조건을 둘 다 넘어야 교체해서 역할이
   진동하지 않게 합니다."
6. **`state_machine.py`** — "몰이의 진행 단계(탐색→추적→몰이→구석몰이→포획)를
   관리합니다. 관측이 끊기면 어떤 상태에서도 즉시 LOST로 빠지는 안전
   조건이 별도로 걸려 있습니다."
7. **`occlusion_grid.py`** — "표적을 놓쳤을 때(LOST) '어디 있을 가능성이
   가장 높은가'를 시간에 따라 흐려지는 확률 지도로 추적해서 재탐색
   목표를 줍니다."
8. **`herding_core.py`** — "위 7개 모듈을 매 제어 주기마다 올바른 순서로
   호출하는 오케스트레이션 파사드입니다. ROS 의존성이 전혀 없어서 이
   전체를 ROS 없이도 통계적으로 검증할 수 있었습니다."
9. **`herding_node.py`** — "순수 로직과 ROS 세계를 잇는 유일한 지점입니다.
   여기엔 알고리즘 로직이 없고, 전부 `HerdingCore`에 위임만 합니다."

---

## 9. 예상 Q&A

**Q. 왜 로봇이 표적을 직접 미느냐, 왜 도주 반응을 이용하는 방식을 택했나?**
A. 실제 로봇이 물리적으로 표적을 접촉해서 미는 것은 안전/제어상 훨씬
어렵습니다. 표적의 자연스러운 회피 행동을 전제로, 로봇은 "표적이 도망칠
방향"만 계산해서 그 반대편에 서는 것으로 충분히 몰이가 가능합니다.

**Q. Driver와 Blocker는 고정 역할인가?**
A. 아니요, 매 제어 주기 `role_assigner.py`가 두 로봇의 거리+회전비용을
비교해 동적으로 재배정합니다. 다만 비용 차이(`role_swap_margin`)와 최소
유지시간(`role_swap_cooldown_sec`) 조건을 둘 다 만족해야 실제로 교체되어,
근소한 차이로 매 주기 역할이 바뀌는 진동을 막습니다.

**Q. 표적을 놓치면(카메라 시야 이탈 등) 어떻게 되나?**
A. `occlusion_timeout_sec` 이상 관측이 끊기면 FSM이 LOST로 전이하고,
`occlusion_grid.py`의 belief 그리드가 마지막 위치에서 확산·감쇠하며
"있을 법한 위치"를 추적합니다. 두 로봇은 그 최고 확률 지점으로 재탐색을
갑니다. (검증 결과 이 부분이 목표 성공률 80%에 못 미치는 79.3%로 나와
알려진 한계로 기록되어 있습니다 — 이동 방향을 반영하지 못하는 확산 모델의
구조적 한계.)

**Q. Nav2와는 어떻게 연동되나?**
A. 이 패키지는 목표 좌표(`~/robot1_goal`, `~/robot2_goal`)만 발행하고,
Nav2의 `NavigateToPose` 액션 호출과 실제 경로 계획/장애물 회피는
다운스트림 미션 매니저가 담당합니다. §6 참고.

**Q. 왜 ROS 의존성을 이렇게 철저히 분리했나?**
A. 실제 로봇/Nav2 스택 없이도 알고리즘 자체의 성공률을 통계적으로
검증(`test/run_validation.py`의 ALGO-001~008, 다회 시행 + 카이제곱
대조군 실험)해야 했기 때문입니다. `herding_node.py`만 rclpy를 쓰고,
그 아래 로직은 순수 numpy/Python이라 pytest만으로 수백 회 시행을 빠르게
돌릴 수 있었습니다.

**Q. 포획 판정은 어떻게 하나? 표적이 반경 근처만 스쳐도 성공으로 잡히지 않나?**
A. 단순히 반경 안에 들어온 순간이 아니라, `capture_hold_sec`(연속 체류
시간) 이상 반경 안에 머물러야 CAPTURED로 판정합니다(`state_machine.py`의
포획 유지 타이머). 다만 반경 자체의 크기(`capture_radius_m`)가 시나리오
난이도를 좌우하는 파라미터이며, 이 값과 로봇 B의 실질 기여도에 대한 검토는
현재 진행 중입니다(트러블슈팅 노트 참고).

---

## 10. 참고 문서

- [설계 스펙](../../docs/superpowers/specs/2026-08-04-herding-controller-design.md) — 원본 함수 흐름/메시지 인터페이스 텍스트
- [코드 워크스루](code_walkthrough.md) — 파일별 클래스/함수 색인
- [최종 검증 리포트](../../docs/superpowers/plans/2026-08-04-herding-controller-final-report.md) — ALGO-001~008 수치 결과
- [트러블슈팅 노트](../../herding_controller_트러블슈팅_노트.md) — 실패 사례, 파라미터 튜닝 히스토리, GIF 도구 개발기
