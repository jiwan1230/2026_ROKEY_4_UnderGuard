# herding_controller

2대의 로봇(Driver/Blocker)이 협업해서 표적을 지정된 포획존으로 몰아가는(Herding)
ROS2 컨트롤러 패키지. Driver는 표적 뒤쪽에 자리를 잡아 밀고, Blocker는 표적이
예측한 도주로 중 가장 유력한 경로를 미리 막아선다.

**이 패키지는 Blocker(로봇 2)의 목표점만 계산·발행한다.** Driver(로봇 1) 역할은
배터리 상태에 따라 번갈아 순찰/충전하는 두 로봇 중 순찰 중 표적을 발견한
쪽에게 상위 시스템이 그 순간 배정하며(포획이 끝날 때까지 고정), 그 로봇의
움직임 자체는 이 패키지가 조종하지 않는다.

## 디렉토리 구조

```
herding_controller/
    herding_controller/        <- 알고리즘 코어 (ROS 의존성 0, herding_node.py 제외)
        grid_map.py             격자 좌표 변환 + 장애물 마스크
        target_estimator.py     칼만 필터 (표적 위치/속도 추정)
        escape_model.py         마르코프 도주 방향 예측 (8방위)
        herding_planner.py      Driving Point / Blocking Point 계산
        geodesic_field.py        벽을 고려한 목표 방향 필드 (Dijkstra 기반)
        role_assigner.py        최소 이격 거리 유지 (역할 자체는 상위 시스템이 배정, 고정)
        state_machine.py        6+1 상태 FSM
        occlusion_grid.py       LOST 상태 재탐색용 belief 그리드
        herding_core.py         위 전부를 조합하는 파사드 (HerdingCore.step())
        herding_node.py         <- 이 패키지에서 유일하게 rclpy를 import하는 파일
    config/herding_params.yaml  ROS 파라미터 기본값 (실제 배포 맵 기준으로 튜닝됨)
    maps/room_map.pgm(.yaml)    실제 SLAM 맵 (정식 검증이 이 맵 기준으로 이루어짐)
    test/                       오프라인 검증 하네스 (ALGO-001~008 통계 검증: 추상 아레나 회귀 + 실제 맵 정식)
    docs/                       코드 리뷰/스터디 자료 (아래 참고)
    experiments/                실험용 프로토타입 (프로덕션 코드 아님, 별도 README)
```

## 빌드/실행

```bash
# 단위 테스트
python3 -m pytest test/ -q -p no:anyio

# 통계 검증 (추상 아레나 회귀 + 실제 맵 정식, 각각 N=100 시행 기준 수 분 소요)
python3 test/run_validation.py 100

# ROS2 노드 실행 (colcon 빌드 후)
ros2 run herding_controller herding_node --ros-args --params-file config/herding_params.yaml
```

## 검증 결과 (2026-08-06, 실제 맵 기준 — 정식)

| 도주 모델 | 성공률 (N=100/트랩 × 3트랩) | 주 트랩("top") 단독 |
|---|---|---|
| `reactive_flee` (주 검증 모델) | 65.0% | 75.0% |
| `noisy_human` (실물 시연 예상치) | 87.0% | 94.0% |

ALGO-001/002/003/005 실제 맵 기준 4개 게이트 전부 PASS, 추상 아레나 회귀
검증(ALGO-001~008) 8개도 전부 PASS. 여기 도달하기까지의 과정(버그 수정이
오히려 성공률을 낮췄던 사례 포함)은 [트러블슈팅 노트](../herding_controller_트러블슈팅_노트.md)
10번 항목 참고.

## 메시지 인터페이스 (요약)

| 구독 | 발행 |
|---|---|
| `~/target_pose`, `~/robot1_pose`(Driver, 읽기 전용), `~/robot2_pose`(Blocker) (PoseStamped) | `~/robot2_goal` (PoseStamped, Blocker만, **좌표만** — Nav2 액션은 직접 호출하지 않음) |
| `/map` (OccupancyGrid) | `~/herding_state` (String/JSON), `~/escape_probability` (OccupancyGrid), `~/capture_result` (Bool) |

`~/robot1_goal`은 발행하지 않는다 — 로봇 1(Driver)은 이 패키지가 조종하지 않는다.

전체 표는 최종본 패키지의 [코드 리뷰 발표 대본](../herding_controller_dual/docs/code_review_script.md) 참고.

## 문서

- [코드 리뷰 발표 대본](../herding_controller_dual/docs/code_review_script.md) — **최신**. 설계 근거, 임계값 근거, 이론 적용, 예상 Q&A
- [설계 스펙](../docs/superpowers/specs/2026-08-04-herding-controller-design.md)
- [최종 검증 리포트](../docs/superpowers/plans/2026-08-04-herding-controller-final-report.md) — 2026-08-05 시점 스냅샷(역할 동적 배정 가정 하의 결과, 지금은 역할이 고정으로 바뀌어 구조가 다름). 현재 수치는 위 "검증 결과" 절 참고
- [트러블슈팅 노트](../herding_controller_트러블슈팅_노트.md) — 실패 사례, 파라미터 튜닝 히스토리, 아키텍처 정정 및 실제 맵 재검증 전체 과정(10번 항목)

## 아키텍처 제약

`herding_core.py`와 그 하위 8개 모듈은 ROS 의존성이 전혀 없는 순수 Python이다.
`herding_node.py`만 `rclpy`를 가져오는 얇은 어댑터이며, 여기엔 알고리즘 로직이
없다 — 전부 `HerdingCore.step()`에 위임한다. 이 경계 덕분에 실제 로봇/ROS 환경
없이도 `test/run_validation.py`로 성공률을 통계적으로 검증할 수 있었다 (그것도
추상 아레나가 아니라 실제 배포 맵 위에서).
