# 실행 안내 — 다른 컴퓨터에서 이 작업 이어하기

**대상** 알고리즘 파트(`intelligence1_algorithm/`) 작업을 다른 노트북에서 그대로 돌리는 경우
**갱신** 2026-08-09

---

## 0. 요약 — 세 줄

```bash
git clone https://github.com/jiwan1230/2026_ROKEY_4_UnderGuard.git
cd 2026_ROKEY_4_UnderGuard && git checkout algorithm/intelligence1-herding-blocker-redesign
pip install numpy scipy matplotlib pyyaml pytest pillow
```

**시뮬레이션·검증·GIF는 ROS 없이 전부 돌아갑니다.** ROS2는 실기 연동에만 필요합니다.

---

## 1. 저장소 받기

### 처음 받는 경우

```bash
git clone https://github.com/jiwan1230/2026_ROKEY_4_UnderGuard.git
cd 2026_ROKEY_4_UnderGuard
git checkout algorithm/intelligence1-herding-blocker-redesign
```

### 이미 받아둔 경우

```bash
cd ~/2026_ROKEY_4_UnderGuard
git fetch origin
git checkout algorithm/intelligence1-herding-blocker-redesign
git pull
```

브랜치 이름이 길어서 탭 자동완성을 쓰거나, 아래처럼 짧은 별칭을 만들어두면 편합니다.

```bash
git config alias.algo 'checkout algorithm/intelligence1-herding-blocker-redesign'
# 이후로는  git algo  만 치면 됩니다
```

---

## 2. 파이썬 패키지 (ROS 불필요)

```bash
pip install numpy scipy matplotlib pyyaml pytest pillow
```

| 패키지 | 쓰는 곳 |
|---|---|
| `numpy` | 알고리즘 전반 |
| `scipy` | 거리장(EDT), 측지거리, 통계 |
| `matplotlib` | GIF·그래프 렌더링 |
| `pillow` | GIF 저장 |
| `pyyaml` | 파라미터·맵 파일 읽기 |
| `pytest` | 단위 테스트 |

**한글이 깨지면** 폰트를 설치하세요. 렌더링 스크립트가 `Noto Sans CJK`를 씁니다.

```bash
sudo apt install fonts-noto-cjk
```

---

## 3. 자주 쓰는 명령 (전부 ROS 불필요)

아래 명령은 모두 이 디렉토리에서 실행합니다.

```bash
cd ~/2026_ROKEY_4_UnderGuard/intelligence1_algorithm/herding_controller_dual
```

### 3.1 단위 테스트 — 30초

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q
```

`177 passed`가 나와야 합니다. `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`은 이 환경의
`anyio` 플러그인 충돌을 피하려고 붙이는 것으로, 없으면 수집 단계에서 죽습니다.

### 3.2 정식 검증 스위트 — 10~20분

```bash
python3 -m test.run_validation
```

추상 아레나 회귀(ALGO-001~008)와 실제 맵 검증(트랩당 100회 × 3트랩)을 전부 돕니다.
현재 기준값:

```
reactive_flee: 성공 96.0% (n=300)
noisy_human  : 성공 98.0% (n=300)
ALGO-001/002/003/005 (실제맵) 4/4 PASS
```

시간이 오래 걸리므로 백그라운드로 돌리고 로그를 보는 편이 낫습니다.

```bash
python3 -m test.run_validation > /tmp/validation.log 2>&1 &
tail -f /tmp/validation.log
```

### 3.3 협공 시연 GIF 뽑기 — 시행 수에 따라 3~15분

```bash
# 36번 돌려서 협공이 잘 나온 순으로 보기 (렌더링 없이 빠름)
python3 -m experiments.pincer_demo --trials 36

# 상위 2개를 GIF로 저장 (기본 저장 위치 ~/Downloads)
python3 -m experiments.pincer_demo --trials 36 --render 2

# 매번 다른 결과를 보고 싶을 때
python3 -m experiments.pincer_demo --trials 30 --seed-base random --render 1

# 마음에 든 시드만 다시 뽑기
python3 -m experiments.pincer_demo --seed 2000038 --trap bottom --render 1
```

출력 표의 **협공지속**과 **최대사잇각**(이론값 120°)을 보고 시드를 고르면 됩니다.
`--outdir`로 저장 위치를 바꿀 수 있습니다.

### 3.4 로봇 B 기여도 어블레이션 (플랜 A 패키지)

```bash
cd ../herding_controller
python3 -m experiments.blocker_contribution_ablation
```

---

## 4. ROS2로 실기 돌리기

여기서부터는 ROS2 Humble과 TurtleBot 4 환경이 필요합니다.

### 4.1 빌드

```bash
cd ~/2026_ROKEY_4_UnderGuard
colcon build --symlink-install
source install/setup.bash
```

`colcon`은 저장소 최상위에서 네 패키지를 전부 찾습니다.

```
herding_controller        intelligence1_algorithm/herding_controller       (플랜 A)
herding_controller_dual   intelligence1_algorithm/herding_controller_dual  (플랜 B = 최종)
turtle_interfaces         src/turtle_interfaces
turtle_project            src/turtle_project
```

확인:

```bash
colcon list
```

### 4.2 노드 실행

```bash
# 플랜 B (최종본)
ros2 run herding_controller_dual herding_node --ros-args \
    -r __node:=herding_controller \
    --params-file install/herding_controller_dual/share/herding_controller_dual/config/herding_params.yaml

# 플랜 A (구버전, 비교용)
ros2 run herding_controller herding_node --ros-args \
    -r __node:=herding_controller \
    --params-file install/herding_controller/share/herding_controller/config/herding_params.yaml
```

**노드 이름을 `herding_controller`로 맞추는 것이 중요합니다.** 배관 노드
(`rat_herding_node`)가 `/herding_controller/...` 토픽 경로를 기대하므로,
두 패키지 어느 쪽을 띄우든 이름이 같아야 그대로 붙습니다.

### 4.3 토픽

| 방향 | 토픽 | 타입 |
|---|---|---|
| 구독 | `/herding_controller/target_pose` | PoseStamped (쥐 위치) |
| 구독 | `/herding_controller/robot1_pose` · `robot2_pose` | PoseStamped |
| 구독 | `/map` | OccupancyGrid |
| 발행 | `/herding_controller/robot1_goal` · `robot2_goal` | PoseStamped |
| 발행 | `/herding_controller/herding_state` | String |
| 발행 | `/herding_controller/escape_probability` | OccupancyGrid |
| 발행 | `/herding_controller/capture_result` | Bool |

플랜 A는 `robot2_goal`만 발행합니다.

### 4.4 파라미터를 실기에서 바꾸기 (재빌드 불필요)

```bash
ros2 param list /herding_controller
ros2 param get  /herding_controller endgame_half_angle_deg
ros2 param set  /herding_controller endgame_half_angle_deg 70.0
```

**주의:** `drive_distance_m × drive_distance_ease_factor`는 반드시
`flee_reaction_distance_m`보다 작아야 합니다. 어기면 노드가 시작 시 죽습니다.

---

## 5. 파일 위치 지도

```
2026_ROKEY_4_UnderGuard/
├── intelligence1_algorithm/              ← 알고리즘 파트 (내 담당)
│   ├── herding_controller_dual/          ★ 최종본 (플랜 B)
│   │   ├── herding_controller_dual/      실제 솔루션 코드 10개 파일
│   │   │   ├── herding_planner.py        목표점 계산 ★ 엔드게임 협공
│   │   │   ├── herding_core.py           오케스트레이션
│   │   │   └── herding_node.py           ROS2 어댑터 (유일한 노드)
│   │   ├── config/herding_params.yaml    운용 파라미터 + 근거 주석
│   │   ├── maps/room_map.yaml/.pgm       검증용 맵 사본
│   │   ├── test/                         검증 하네스 + 단위 테스트 177개
│   │   │   ├── real_map_arena.py         ★ 덫/스폰 좌표는 여기
│   │   │   ├── simulator.py              헤드리스 2D 시뮬레이터
│   │   │   └── run_validation.py         정식 검증 스위트
│   │   ├── experiments/pincer_demo.py    협공 GIF 뽑는 도구
│   │   └── docs/                         이 문서 포함
│   ├── herding_controller/               플랜 A (구버전, 비교 기준선)
│   └── herding_controller_트러블슈팅_노트.md   개발 경위 전체 기록
│
└── src/                                  ← 로봇 파트 (팀원 담당)
    ├── turtle_project/
    │   ├── turtle_project/rat_herding_node.py   ★ 알고리즘↔fleet 배관 (내 담당)
    │   ├── turtle_project/detector_node.py      YOLO 쥐 감지
    │   ├── turtle_project/robot_agent.py        로봇별 Nav2 주행
    │   ├── config/nav2.yaml                     robot_radius 0.175 / inflation 0.25
    │   └── resource/room_map.yaml/.pgm          실제 맵 원본
    └── turtle_interfaces/
```

---

## 6. 현재 확정된 좌표 (재-SLAM 맵 기준)

```
그리드 : 109 x 149, origin (-2.68, -6.08), resolution 0.05

덫 top    : (-2.20, -2.41)
덫 left   : (-1.56, +0.74)
덫 bottom : (+2.35, -3.41)

로봇 A 시작 : (-2.00, +0.74)   순찰 경로 첫 점
로봇 B 대기 : (+0.62, -5.58)   ★순찰 경로 밖이어야 함
```

옛 맵(106×149) 좌표를 쓰는 자료가 있으면 `새 = 옛 + (+0.61, +2.95) m`로
변환하면 됩니다. 단 로봇 B 대기 지점은 변환하면 순찰 경로를 막습니다
(`test/real_map_arena.py`의 `ROBOT_B_SPAWN` 주석 참고).

---

## 7. 자주 겪는 문제

| 증상 | 원인 / 해결 |
|---|---|
| `pytest` 수집 단계에서 죽음 | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`을 앞에 붙이세요 |
| 그래프·GIF 한글이 네모로 깨짐 | `sudo apt install fonts-noto-cjk` |
| `ModuleNotFoundError: test` | `herding_controller_dual` 디렉토리 안에서 실행해야 합니다 |
| `colcon`이 알고리즘 패키지를 못 찾음 | 저장소 최상위에서 빌드하세요 (`src/`가 아니라) |
| 노드가 시작하자마자 죽음 | `drive_distance_m × 1.15 < flee_reaction_distance_m` 위반 |
| 검증 숫자가 문서와 다름 | 맵/좌표가 갱신됐는지 확인 — `git pull` 후 다시 |
