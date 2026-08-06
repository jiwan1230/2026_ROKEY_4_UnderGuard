# 실험용 프로토타입 (프로덕션 코드 아님)

이 폴더는 `herding_controller/herding_controller/`(정식 알고리즘)와 `herding_controller/test/`(정식 검증
하네스)와는 별개로, 로봇 2대를 동시에 제어하기 전에 단계적으로 검증해보기 위해 만든 실험용
스크립트들이다. 프로덕션 코드는 건드리지 않았고, 여기서 검증된 아이디어가 있으면 나중에 정식으로
통합하면 된다.

## 파일 구성

- `single_robot_sim.py` / `single_robot_template.html` / `single_robot_replay.html`
  손그림 월드맵(벽 1개 + 쥐구멍 3곳) 위에서, 로봇 1대만 능동적으로 몰이하는 실험. 최종적으로는
  로봇 A=Driver(미는 역할), 로봇 B=Blocker(경로 차단)로 정리됨 — 기존 검증된
  `herding_planner.py`의 `compute_driving_point`/`compute_blocking_point`를 그대로 재사용한다.
- `real_map_sim.py` / `real_map_template.html` / `real_map_replay.html`
  실제 SLAM 맵(`../maps/room_map.pgm`) 위에서 같은 실험. 로봇 A가 경유점을 순회하며 순찰하다가
  센서 반경 안에 쥐가 들어오면 그 순간 Driver로 전환되고, 로봇 B가 출발해 Blocker를 시작하는
  순찰→발견 흐름과, 벽에 가까워지면 목표 방향에 반발 방향을 섞는 임시 벽 회피(potential field)가
  들어 있다. **2026-08-06부로 이 스크립트도 `HerdingCore` 전체를 그대로 사용한다**
  (더는 개별 함수를 손으로 조합하지 않음) — GIF 시각화 전용이지만 알고리즘 자체는
  정식 검증(`run_real_map_algo_suite()`)과 동일하다. 다만 목표 트랩을 "발견 위치에서
  가장 가까운 곳"으로 매 시행 다르게 고르기 때문에(정식 검증은 트랩 1곳을 미리 고정),
  체감 성공률이 정식 통계보다 높게 나온다 — 시각 자료용 데모 시나리오라는 점 참고.
- `../maps/room_map.pgm`, `../maps/room_map.yaml`
  실제 room_map 원본 (해상도 0.05m, 원점 [-3.19, -9.03]). **2026-08-06 패키지 레벨로 이동**
  (`herding_controller/maps/`) — 정식 검증 하네스와 프로덕션 코드가 공유하는 자산이 됐기 때문.
- `*_replay.html`
  각 스크립트를 실행한 결과를 이미 끼워 넣어 만든 완성된 페이지. 브라우저로 바로 열면 재생된다
  (재생성 없이 바로 확인하고 싶을 때 사용).
- `record_trial.py`
  화면 녹화 없이, `*_frames.json`에 저장된 좌표 데이터를 직접 matplotlib으로 렌더링해서 GIF로
  저장하는 스크립트. `media/`에 트러블슈팅 노트용으로 미리 뽑아둔 예시가 들어 있다.
- `success_rate_check.py`
  `real_map_sim.py`의 `run_trial()`을 GIF/JSON 생성 없이 반복 실행해서, 로봇 B(Blocker)를
  껐을 때와 켰을 때 성공률 차이(소거 실험)만 빠르게 재는 보조 스크립트.
  `python3 success_rate_check.py 25`처럼 실행. **N=100 규모의 공식 성공률 통계는
  이제 `python3 test/run_validation.py 100`(정확히는 그 안의
  `run_real_map_algo_suite()`)이 담당한다** — 트러블슈팅 노트 10번 항목 참고.
- (`geodesic_field.py`는 2026-08-06부로 `herding_controller/herding_controller/geodesic_field.py`로
  승격되어 정식 알고리즘의 일부가 됐다 — Driver/Blocker의 목표 방향 계산에 벽 정보를 반영하는
  모듈. `HerdingCore`가 내부적으로 항상 사용하므로(끄고 켜는 옵션 아님) `real_map_sim.py`도
  자동으로 이걸 쓴다. 자세한 경위는 트러블슈팅 노트 9/10번 항목 참고.)

## 재실행 방법

```bash
cd herding_controller/experiments
python3 single_robot_sim.py   # single_robot_frames.json 생성
python3 real_map_sim.py       # real_map_frames.json 생성 (최대 몇 분 소요)
```

시뮬레이션 데이터를 다시 만든 뒤, 아래처럼 템플릿에 끼워 넣으면 `*_replay.html`을 갱신할 수 있다:

```bash
python3 -c "
for prefix in ['single_robot', 'real_map']:
    data = open(f'{prefix}_frames.json').read()
    template = open(f'{prefix}_template.html').read()
    open(f'{prefix}_replay.html', 'w').write(template.replace('/*__TRIAL_DATA__*/', data))
"
```

## GIF로 시행 하나 뽑기

**GIF는 자동으로 생성되지 않는다.** `real_map_sim.py`/`single_robot_sim.py`를 실행하면
그 결과가 `*_frames.json`에 저장될 뿐이고, 그 안의 원하는 시행 번호를 골라
`record_trial.py`를 **수동으로 한 번 더 실행**해야 GIF가 만들어진다.

```bash
# python3 record_trial.py <데이터.json> <시행 번호(0부터)> <출력.gif> [--max-seconds N] [--fps N] [--subsample N] [--number]
python3 record_trial.py real_map_frames.json 0 media/my_clip.gif --fps 15
```

`--max-seconds`로 앞부분만 잘라낼 수 있고(정지해버린 실패 시행을 끝까지 렌더링할 필요는 없으니),
`--subsample`로 프레임을 건너뛰어 재생 속도를 조절한다 (기본 3 = 원본 대비 3배속 근처).

### 파일이 덮어써지는 문제 (`--number`)

같은 `out_gif` 경로로 다시 실행하면 이전 GIF가 **조용히 덮어써진다** — "자꾸
초기화된다"고 느꼈던 원인이 이것이다. `--number` 플래그를 붙이면 대상 파일이
이미 있을 때 덮어쓰는 대신 자동으로 `_001`, `_002`... 번호를 붙여 새 파일로
저장하므로 이전 산출물이 항상 남는다:

```bash
python3 record_trial.py real_map_frames.json 0 media/trial.gif --number
# 처음 실행 -> media/trial.gif
# 두 번째 실행(같은 명령 그대로) -> media/trial_001.gif
# 세 번째 실행 -> media/trial_002.gif ...
```

트러블슈팅 노트나 코드 리뷰에 "before/after"를 남기고 싶을 때는, 파일명
자체에 의미를 담아 번호 대신 라벨을 붙이는 것도 방법이다 (예:
`media/real_map_v1_before_wall_avoid.gif`, `media/real_map_v2_after_wall_avoid.gif`).
`--number`는 라벨을 매번 생각해내기 귀찮을 때 쓰는 안전장치로 보면 된다.

## 알려진 한계 (다음에 이어서 할 사람을 위해)

- **경로 계획 없음**: 로봇 이동이 "목표까지 직선/경유점 + 막히면 국소 우회 + 벽 반발력" 조합일
  뿐, A* 같은 진짜 경로 계획은 없다. 실제 로봇 연동 전 최소한의 완충용 (다만 실제 검증
  하네스에는 `geodesic_field.py`의 경유점 유도가 추가되어 상당 부분 보완됨, 아래 참고).
- **(해결됨, 2026-08-06) 실측 성공률이 목표(90%)에 크게 못 미쳤던 문제**: 이 폴더의 실험 결과
  (46.2%, `success_rate_check.py`)로 한동안 "목표 방향 정확도가 병목이 아니다"까지만 확인됐었는데,
  이후 정식 검증 하네스(`test/real_map_arena.py`)로 옮겨서 `HerdingCore` 전체로 재현·추적한
  결과 진짜 원인 3가지를 찾아 전부 해결했다: ①표적이 벽 앞에서 완전히 얼어붙던 버그(고치니
  일시적으로 더 나빠져서 "버그가 성공률을 부풀리고 있었다"는 걸 알게 됨), ②`panic_distance_m`
  등 거리 파라미터가 이 방 규모(5.3×7.35m)에 안 맞았던 것, ③`capture_hold_sec`(3.0초)이 좁은
  방에서 표적이 반경 경계를 들락거리는 진동 때문에 너무 길었던 것(1.5초로 조정). 최종 실제 맵
  검증 성공률: `reactive_flee` 65.0%, `noisy_human` 87.0% (N=100/트랩 × 3트랩). 전체 여정은
  트러블슈팅 노트 10-4 항목에 기록.
- `SENSOR_RANGE_M`(1.5m), `WALL_AVOID_RADIUS_M`(0.4m) 등은 전부 스펙에 없는 실험적 가정값이다.

자세한 트러블슈팅 히스토리는 저장소 루트의 `herding_controller_트러블슈팅_노트.md` 참고.
