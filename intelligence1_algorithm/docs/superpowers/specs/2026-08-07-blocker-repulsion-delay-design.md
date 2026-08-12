# 로봇 B(Blocker) 기여도 검증 — 7차 시도: escape_model 반발항 지연 + 강화 시나리오

## 0. 배경

`herding_controller_트러블슈팅_노트.md` 11번 항목에서, 로봇 B(Blocker)가 성공률에
실제로 기여하는지 여섯 갈래(커밋 주기 튜닝, N=450 정식 어블레이션, 회피 모델 4종,
`block_lookahead_m` 양방향 재스윕, `compute_blocking_point` geodesic 재설계, "B
준비까지 A 대기")로 검증했지만 전부 유의미한 개선을 만들지 못했다. 11-9의 페어드
rescue/regression 분석(N=450, seed 페어링)으로 문제가 **left 트랩에 집중**돼 있고
(rescue=2, regression=27), 그 원인 대부분(26건 중 24건)이 "실시간으로 막혀서"가
아니라 **`EscapeModel._robot_repulsion()`이 거리 무관하게 항상 로봇 B의 존재를
반영해, 몰이 초반부터 표적의 도주 경로를 미묘하게 바꿔놓고 그 나비효과로 한참
뒤 로봇 A 혼자 막다른 길에 갇히는** 간접·경로의존적 메커니즘이라는 결론에
도달했다. 11-9는 다음 시도로 "로봇 B가 표적과 실제로 가까워지기 전까지는
`_robot_repulsion()` 계산에서 제외"를 제안했고, 11-10에서 이를 포함한 12개 후보를
정리했다. 이 문서는 사용자와 합의한 조합(후보 6번을 주 실험, 8번을 보조 진단으로)의
구현 설계다.

## 1. 목표 및 성공 기준

**주 실험**: `EscapeModel._robot_repulsion()`에서 로봇 B(Blocker)의 기여에 거리
기반 활성화 임계값을 추가한다. Driver는 건드리지 않는다.

**보조 진단**: 기존 `JukingFlee` 모델의 파라미터를 조정해 "Driver 단독으로는
명백히 부족한" 강화 시나리오를 만들고, 그 조건에서 주 실험의 효과가 표준
시나리오의 천장효과 때문에 가려졌을 가능성을 배제한다.

**성공 기준**: 11-9와 동일한 페어드 rescue/regression 지표(N=150×3트랩=450,
`HerdingCore._stabilize_blocking_point` 몽키패치로 능동/고정 비교, 동일 seed로
페어링)를 재사용한다. 현재 기준값은 rescue=2, regression=27이다. 주 실험 적용
후 regression이 통계적으로 유의미하게 줄면(예: Fisher's exact test) 성공.
표준 시나리오와 강화 시나리오 둘 다에서 유의미한 개선이 없으면 "구조적 한계"로
결론짓고, 11-7의 "원점 재검토" 후보(2로봇 구조 자체가 Driver 단독과 차이가
없다는 결론)로 넘겨 다음 세션에 사용자와 논의한다.

## 2. 아키텍처 / 데이터 흐름

`EscapeModel.compute(target_pos, target_vel, robot_positions)`은
`robot_positions = [observation.robot1_pos, observation.robot2_pos]` 순서로
호출된다(`herding_core.py:399`, 10번 항목에서 확정된 "로봇 1=Driver 고정/로봇
2=Blocker 고정" 아키텍처에 근거). 즉 `robot_positions[1]`이 항상 Blocker다. 이
고정된 인덱스 관계를 이용해, `_robot_repulsion()` 내부에서 인덱스 1에 한해 거리
게이팅을 적용한다 — 함수 시그니처나 호출부는 바꾸지 않는다.

```
EscapeModel.compute()
  └─ _robot_repulsion(target_pos, robot_positions)
       for i, robot_pos in enumerate(robot_positions):
           dist = ||target_pos - robot_pos||
           if i == 1 and dist > config.robot_repulsion_activation_distance_m:
               continue  # Blocker가 아직 멀면 기여 0
           (기존 로직 그대로)
```

새 설정값 `HerdingConfig.robot_repulsion_activation_distance_m: float = float("inf")`.
기본값이 무한대이므로 게이팅 조건이 항상 거짓 — 기존 동작과 100% 동일하게
유지된다(하위호환, 기존 142개 테스트 회귀 없어야 함). 실험에서만 유한값으로
오버라이드한다.

Hard cutoff(거리 밖이면 기여 완전히 0) 방식을 채택한다. 10-6/11-6에서 반복
확인된 "좁은 공간에서 급격한 전환은 진동을 부른다"는 교훈이 있지만, 이번
게이팅 대상은 물리적 이동 명령이 아니라 확률 추정 입력이라 즉각적인 진동
리스크가 낮다고 판단한다. 실험 중 진동이나 불안정성이 관측되면 그때
스무딩(예: 임계값 근처 선형 램프)을 추가한다 — 처음부터 넣지 않는다(YAGNI).

## 3. 구성요소

### 3-1. `herding_controller/herding_controller/escape_model.py`
- `EscapeModelConfig`(또는 `HerdingConfig`에서 전달되는 대응 필드)에
  `robot_repulsion_activation_distance_m` 추가
- `_robot_repulsion()`에 인덱스 1 게이팅 로직 추가

### 3-2. `herding_controller/herding_controller/herding_core.py`
- `HerdingConfig`에 `robot_repulsion_activation_distance_m: float = float("inf")` 필드 추가
  (기존 `blocking_point_commit_sec`/`deadlock_*` 필드와 같은 위치, 기본값 있는
  필드 그룹에 배치)
- `__post_init__` 불변식 영향 없음 확인(기존 `drive_distance_m * drive_distance_ease_factor
  < flee_reaction_distance_m` 제약과 무관한 독립 필드)

### 3-3. `config/herding_params.yaml`
- 신규 필드는 기본값(무한대 상당 표현, 예: 필드 생략 시 dataclass 기본값 사용)
  그대로 두어 프로덕션 동작 불변 유지. YAML에 명시적으로 추가할지는 스윕 결과가
  나온 뒤 "채택할 값이 있다면" 그때 반영한다(11-6/10-6 패턴과 동일 — 실험
  단계에서는 코드 옵션 인자로만 존재).

### 3-4. 페어드 rescue/regression 스크립트 (신규, 커밋 대상)
- 위치: `herding_controller/experiments/blocker_contribution_ablation.py`
- 11-9에서 쓰였지만 저장되지 않았던 몽키패치 방식(`HerdingCore._stabilize_blocking_point`를
  "능동" vs "스폰 위치 고정" 두 조건으로 교체)을 재구현하고, 이번엔 반드시
  커밋해 재사용 가능하게 만든다.
- CLI 인자: `--robot-repulsion-activation-distance`(float, 기본 inf),
  `--trials-per-trap`(기본 150), `--model`(기본 reactive_flee),
  `--juke-probability`/`--juke-duration`/`--juke-angle-range`(JukingFlee 강화
  시나리오용, 지정 시 `reactive_flee` 대신 튜닝된 `JukingFlee` 사용)
- 출력: 트랩별 rescue/regression/양쪽성공/양쪽실패 표 (11-9 표 형식과 동일),
  Fisher's exact test p-value 포함

### 3-5. 실행 순서 (스크립트 실행으로 진행, 별도 자동화 오케스트레이터 없음)
1. `robot_repulsion_activation_distance_m` 스윕: `inf`(대조군), 2.0, 1.5, 1.0,
   0.5m — 표준 `reactive_flee`, N=150×3트랩
2. 가장 나은 값으로 전체 회귀 스위트(`run_real_map_algo_suite`,
   `test/run_validation.py`) 재검증 — 다른 트랩/모델(`noisy_human` 등)에 회귀
   없는지 확인
3. `JukingFlee` 파라미터 스크리닝(N=30, Blocker 고정 조건만 우선): 후보
   2~3조합에서 frozen(Driver 단독) 성공률이 뚜렷이 낮아지는(예: 40% 이하) 설정
   탐색. 시작 후보: `juke_probability_per_sec` 상향(0.5→0.8~1.0), `juke_duration_sec`
   상향(0.4→0.8~1.2), 각도 범위 확대
4. 확정된 강화 시나리오에서 1번의 최적 활성화 거리로 다시 active-vs-frozen
   페어드 비교 (N=150×3)

### 3-6. 문서화
- `herding_controller_트러블슈팅_노트.md`에 `## 12. 로봇 B 기여도 — 7차 시도
  (escape_model 반발항 지연)` 섹션 추가. 실험 과정 전체(성공이든 실패든, 원인
  추적 포함)를 기존 절과 같은 상세도로 기록한다 — 이 노트는 결과가 좋을 때만
  쓰는 문서가 아니라 시행착오 자체가 산출물이다.
- 결론에 따라 11-7 "다음 단계 후보" 목록도 갱신(성공 시 완료 표시, 실패 시
  "원점 재검토"로 안내)

## 4. 에러 처리 / 엣지 케이스
- `robot_positions`에 로봇이 2개 미만 들어오는 경우는 없음(현재 아키텍처상
  항상 Driver+Blocker 2개 고정) — 방어 코드 불필요
- `robot_repulsion_activation_distance_m`이 0 이하로 설정되면 Blocker 기여가
  항상 0(완전 무시)이 되는데, 이는 유효한 극단값이므로 별도 검증 불필요(자연스러운
  스윕 범위의 일부)

## 5. 테스트 계획
- 기존 142+α개 유닛테스트 전부 통과 확인(신규 필드 기본값이 무한대이므로
  회귀 없어야 함)
- 신규 유닛테스트: `_robot_repulsion()` 게이팅 — (a) 기본값(inf)일 때 기존
  동작과 동일, (b) 유한 임계값 설정 시 Blocker가 임계값 밖이면 기여 0, 임계값
  안이면 기존과 동일 기여
- 통계 검증은 유닛테스트가 아니라 3-4의 스크립트 실행 결과로 판단(기존 프로젝트
  관행과 동일 — `success_rate_check.py`, `run_validation.py` 등도 유닛테스트가
  아니라 별도 실행 스크립트)

## 6. 브랜치
현재 브랜치 `algorithm/intelligence1-herding-blocker-redesign`을 그대로 사용
(11-6 geodesic 재설계도 이 브랜치에서 진행됨).
