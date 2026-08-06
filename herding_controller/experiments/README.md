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
  실제 SLAM 맵(`maps/room_map.pgm`) 위에서 같은 실험. 로봇 A가 경유점을 순회하며 순찰하다가
  센서 반경 안에 쥐가 들어오면 그 순간 Driver로 전환되고, 로봇 B가 출발해 Blocker를 시작하는
  순찰→발견 흐름과, 벽에 가까워지면 목표 방향에 반발 방향을 섞는 임시 벽 회피(potential field)가
  들어 있다.
- `maps/room_map.pgm`, `maps/room_map.yaml`
  실제 room_map 원본 (해상도 0.05m, 원점 [-3.19, -9.03]).
- `*_replay.html`
  각 스크립트를 실행한 결과를 이미 끼워 넣어 만든 완성된 페이지. 브라우저로 바로 열면 재생된다
  (재생성 없이 바로 확인하고 싶을 때 사용).
- `record_trial.py`
  화면 녹화 없이, `*_frames.json`에 저장된 좌표 데이터를 직접 matplotlib으로 렌더링해서 GIF로
  저장하는 스크립트. `media/`에 트러블슈팅 노트용으로 미리 뽑아둔 예시가 들어 있다.
- `success_rate_check.py`
  `real_map_sim.py`의 `run_trial()`을 GIF/JSON 생성 없이 다회 반복 실행해서 성공률과 로봇
  B(Blocker) 실질 기여도(소거 실험 포함)를 통계로 뽑는 스크립트. `python3 success_rate_check.py 25
  --ablation`처럼 실행 (트러블슈팅 노트 8번 항목 참고). `--compare-geodesic`을 추가하면 아래
  `geodesic_field.py` 적용 전/후 성공률도 같이 비교한다.
- `geodesic_field.py`
  Driver/Blocker의 목표 방향 계산에 벽 정보를 반영하기 위한 모듈. 트랩(목표)으로부터 장애물을
  피해서 가는 실제 최단거리 필드(Dijkstra)를 만들고, 그 기울기를 "벽을 고려한 목표 방향"으로
  제공한다. `real_map_sim.py`는 기본적으로 이걸 사용한다(`run_trial(..., use_geodesic=True)`).
  트러블슈팅 노트 9번 항목에 효과 측정(+3.8%p, 성공률의 지배적 병목은 아님)이 정리돼 있다.

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

- **경로 계획 없음**: 로봇 이동이 "목표까지 직선 + 막히면 국소 우회 + 벽 반발력" 조합일 뿐,
  A* 같은 진짜 경로 계획은 없다. 실제 로봇 연동 전 최소한의 완충용.
- **목표 쥐구멍 0.3~0.4m 앞에서 멈추는 미해결 실패 케이스**가 있다 (손그림 맵의 "좌측 쥐구멍",
  실제 맵의 "상단 쥐구멍" 시나리오에서 재현). 로봇이 위협 위치로 이동하는 경로가 표적의 반응
  반경을 스치면서 표적이 엉뚱한 방향으로 먼저 도망치는 게 원인으로 보임.
- **실측 성공률이 목표(90%)에 크게 못 미친다 (46.2%, n=80, `success_rate_check.py`).** 포획 판정에
  "도주확률 집중" 조건이 빠져 있던 버그를 고친 뒤 드러난 수치(42.5%, 트러블슈팅 노트 8번 항목)에,
  벽을 고려한 목표 방향(`geodesic_field.py`, 9번 항목)을 적용해 소폭 개선한 값이다. 로봇
  B(Blocker)의 실질 기여도는 소거 실험 결과 거의 0으로 측정됐고, geodesic 방향 적용도 +3.8%p에
  그쳐 **목표 방향의 정확도가 이 문제의 지배적인 병목은 아니라는 것**이 실험으로 확인됐다. 남은
  격차는 로봇 속도(0.3m/s vs 표적 0.4m/s), escape model의 8방향 이산화, Blocker의 선점 타이밍
  중 무엇이 진짜 병목인지 개별 소거 실험이 필요 — 트러블슈팅 노트 9-4에 다음 가설들 정리해 둠.
- `SENSOR_RANGE_M`(1.5m), `WALL_AVOID_RADIUS_M`(0.4m) 등은 전부 스펙에 없는 실험적 가정값이다.

자세한 트러블슈팅 히스토리는 저장소 루트의 `herding_controller_트러블슈팅_노트.md` 참고.
