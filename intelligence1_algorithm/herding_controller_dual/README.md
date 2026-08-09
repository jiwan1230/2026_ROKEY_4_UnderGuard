# herding_controller_dual (플랜 B — 동적 Driver/Blocker 역할 배정)

2대의 로봇이 협업해서 표적을 지정된 포획존으로 몰아가는(Herding) ROS2
컨트롤러 패키지. **`herding_controller`(플랜 A)와의 차이**: 플랜 A는 Driver
역할이 상위 시스템에 의해 에피소드 내내 고정되고, 이 패키지는 로봇 2(Blocker)의
목표점만 계산·발행한다. 플랜 B는 두 로봇 중 어느 쪽이 미는 역할(Driver)을
할지 매 제어 주기 `RoleAssigner`가 거리+회전비용으로 재판단하고
(margin+cooldown 이력현상으로 진동 방지), **두 로봇 모두**의 목표점을
계산·발행한다.

원본은 `algorithm/dual-robot-herding` 브랜치의 `herding_controller_dual`에
있으며(2026-08-07, 4abf472 — git 히스토리 `b35eb68`에 있다가 `6c04c5b`에서
"로봇 A는 조종 안 함" 정정과 함께 제거됐던 `RoleAssigner`를 복원한 것), 그
당시 코드베이스(트러블슈팅 노트 10-4까지) 위에 만들어졌다. 이 패키지는 그
로직을 **최신 플랜 A**(10-6/11-8/11-9/12/13번 항목의 모든 수정: 벽 반발
샘플링 반경, deadlock 감지·해제, escape_model 반발항 게이팅 포함) 위에
다시 이식한 버전이다(2026-08-08).

## 검증 결과 (2026-08-09, 재-SLAM 맵 기준)

`python3 -m test.run_validation` 한 줄로 전체가 돈다. 트랩당 100회 x 3트랩.

| 표적 모델 | 성공률 | 평균 소요 | panic |
|---|---|---|---|
| `noisy_human` (실물 시연 예상치) | **95.7%** (n=300) | 22.8s | 19.0% |
| `reactive_flee` (기본 검증) | 85.3% (n=300) | 19.9s | 6.3% |

ALGO-001/002/003/005(실제맵) 4/4 PASS, 추상 아레나 ALGO-001~008 8/8 PASS.

**소요 시간은 "표적을 발견한 이후"부터 잰다** (2026-08-09 변경). 로봇 파트의
실제 순찰 경로(36점)를 쓰면서 발견까지 중앙값 46.8초가 걸리는데, 그건 순찰
경로 길이지 몰이 알고리즘 성능이 아니다. 탐색에는 별도 예산
(`max_search_time_sec`)을 준다 -- 자세한 근거는 `test/simulator.py`의
`SimulatorConfig.max_sim_time_sec` 주석 참고.

옛 맵(106x147) 대비: `noisy_human` 95.3% -> 95.7%로 동등하고,
`reactive_flee`만 93.7% -> 85.3%로 낮다. 후자는 로봇이 0.42m 안에 들어와야만
움직이는 모델이라 L자 방 구조에 더 민감하다.

## 디렉토리 구조

```
herding_controller_dual/
    herding_controller_dual/   <- 알고리즘 코어 (ROS 의존성 0, herding_node.py 제외)
        grid_map.py             격자 좌표 변환 + 장애물 마스크
        target_estimator.py     칼만 필터 (표적 위치/속도 추정)
        escape_model.py         마르코프 도주 방향 예측 (8방위), blocker_index 파라미터로 역할-인식 게이팅
        herding_planner.py      Driving Point / Blocking Point 계산
        geodesic_field.py        벽을 고려한 목표 방향 필드 (Dijkstra 기반)
        role_assigner.py        RoleAssigner(동적 역할 배정) + 최소 이격 거리 유지
        state_machine.py        6+1 상태 FSM
        occlusion_grid.py       LOST 상태 재탐색용 belief 그리드
        herding_core.py         위 전부를 조합하는 파사드 (HerdingCore.step())
        herding_node.py         <- 이 패키지에서 유일하게 rclpy를 import하는 파일
    config/herding_params.yaml  ROS 파라미터 기본값 (role_swap_* 포함)
    maps/room_map.pgm(.yaml)    실제 SLAM 맵 (정식 검증이 이 맵 기준으로 이루어짐)
    test/                       오프라인 검증 하네스 (ALGO-001~008 통계 검증: 추상 아레나 회귀 + 실제 맵 정식)
    docs/                       플랜 A에서 그대로 가져온 코드 리뷰/스터디 자료(플랜 B 반영 안 됨, 참고용)
    experiments/                실험용 프로토타입 (플랜 A 전용 어블레이션 스크립트는 제외했음)
```

## 빌드/실행

```bash
# 단위 테스트
python3 -m pytest test/ -q -p no:anyio

# 통계 검증 (추상 아레나 회귀 + 실제 맵 정식)
python3 test/run_validation.py 100

# ROS2 노드 실행 (colcon 빌드 후)
ros2 run herding_controller_dual herding_node --ros-args --params-file config/herding_params.yaml
```

## 메시지 인터페이스 (요약)

| 구독 | 발행 |
|---|---|
| `~/target_pose`, `~/robot1_pose`, `~/robot2_pose` (PoseStamped) | `~/robot1_goal`, `~/robot2_goal` (PoseStamped, **좌표만** — Nav2 액션은 직접 호출하지 않음) |
| `/map` (OccupancyGrid) | `~/herding_state` (String/JSON, `driver`/`blocker` 필드로 현재 역할 배정 포함), `~/escape_probability` (OccupancyGrid), `~/capture_result` (Bool) |

플랜 A와 달리 `~/robot1_goal`도 발행한다 — 두 로봇 다 이 노드가 조종한다.

## 문서

- [트러블슈팅 노트](../herding_controller_트러블슈팅_노트.md) 14번 항목 — 이 패키지를 만든 배경, 최신 플랜 A 위로 이식한 과정, 검증 결과
- `docs/` 아래 문서는 플랜 A 기준으로 작성된 것을 그대로 복사해왔다 — 아키텍처 설명 중 "역할 고정"/"로봇 A는 조종 안 함" 부분은 이 패키지에는 적용되지 않으니 주의.
