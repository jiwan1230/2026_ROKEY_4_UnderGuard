# rosbag2를 쥐몰이 Replay JSON으로 변환하는 방법

이 문서는 실제 쥐몰이 시험을 Sysmon PC에서 rosbag2로 기록하고, System Monitor의
`기록 조회 → 쥐몰이 기록` 화면에서 재생할 JSON으로 바꾸는 순서를 설명한다.

## 1. 먼저 알아둘 점

Replay 화면은 카메라 동영상을 재생하는 기능이 아니다. 여러 ROS 토픽에서 받은
쥐·로봇·목표 좌표와 알고리즘 상태를 시간순으로 합쳐 지도 위에 다시 그린다.

현재 `main`의 `detector_node.py`와 `rat_herding_node.py`에는 쥐 좌표 계산과 몰이
알고리즘 부분이 아직 `TODO`로 남아 있다. 따라서 실제 시험 전에 최소한
`rat_detected:x:y` 사건과 두 로봇 odom이 발행되는지 반드시 확인해야 한다.
이 값이 없으면 변환기는 좌표를 추측하지 않고 어떤 필수 값이 없는지 알려준다.

## 2. 변환기가 읽는 토픽

### 현재 main에서 사용하는 기본 토픽

| 의미 | 기본 토픽 | 메시지 타입 | 필수 여부 |
|---|---|---|---|
| 쥐 위치 | `/fleet/event`의 `rat_detected:x:y` | `std_msgs/String` | 필수 |
| Driver 실제 위치 | `/robot4/odom` | `nav_msgs/Odometry` | 필수 |
| Blocker 실제 위치 | `/robot6/odom` | `nav_msgs/Odometry` | 필수 |
| Driver 목표 | `/robot4/target_pose` | `geometry_msgs/PoseStamped` | 선택 |
| Blocker 목표 | `/robot6/target_pose` | `geometry_msgs/PoseStamped` | 선택 |
| Driver 상태 | `/fleet/status`의 `robot4:state:battery` | `std_msgs/String` | 선택 |

목표 토픽이 없으면 해당 로봇의 실제 위치를 목표 위치로 사용한다. 상태 토픽이
없으면 `SEARCH`, 포획 진행률 토픽이 없으면 0%를 사용한다. 이 대체값은 JSON을
깨뜨리지 않기 위한 값이므로 정밀한 알고리즘 분석에는 전용 토픽을 기록하는 것이
좋다.

### 정밀 기록을 위해 권장하는 전용 토픽

| 의미 | 기본 토픽 | 권장 메시지 타입 | 값 |
|---|---|---|---|
| FSM 상태 | `/herding/state` | `std_msgs/String` | `SEARCH`, `TRACK`, `HERD`, `CORNER`, `LOST`, `CAPTURED` |
| 포획 진행률 | `/herding/capture_progress` | `std_msgs/Float32` | 0.0~1.0 |
| 성공 여부 | `/herding/success` | `std_msgs/Bool` | 성공 시 `true` |

현재 main에는 이 전용 토픽 발행이 아직 구현되지 않았다. 알고리즘 팀과 위 이름과
타입을 합의하거나, 실제 발행 이름이 다르면 변환 명령의 옵션으로 바꾸면 된다.

## 3. 시험 전에 토픽 확인

ROS 2와 프로젝트 워크스페이스를 먼저 불러온다.

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot4_ws/install/setup.bash
```

현재 발행 중인 토픽과 타입을 확인한다.

```bash
ros2 topic list -t
ros2 topic echo /fleet/event
ros2 topic echo /robot4/odom --once
ros2 topic echo /robot6/odom --once
```

odom의 `header.frame_id`도 확인해야 한다. Replay 지도는 `map` 좌표를 기대한다.
`odom`처럼 다른 frame이면 TF를 적용한 map 위치 토픽을 기록하거나, 변환 후 경고를
확인해 좌표가 같은 지도 기준인지 검증해야 한다.

## 4. Sysmon PC에서 rosbag2 기록

현재 main 토픽만 기록하는 기본 명령은 다음과 같다.

```bash
mkdir -p ~/under_guard_bags

ros2 bag record -o ~/under_guard_bags/trial_001 \
  /fleet/event \
  /fleet/status \
  /robot4/odom \
  /robot6/odom \
  /robot4/target_pose \
  /robot6/target_pose
```

정밀 상태 토픽까지 발행되는 경우에는 다음 세 토픽도 뒤에 추가한다.

```text
/herding/state
/herding/capture_progress
/herding/success
```

시험을 시작하기 전에 위 기록 명령을 실행하고, 시험이 끝난 뒤 기록 터미널에서
`Ctrl+C`를 누른다. 결과는 단일 파일이 아니라 다음과 같은 디렉터리다.

```text
~/under_guard_bags/trial_001/
├── metadata.yaml
└── trial_001_0.db3
```

기록 내용을 확인한다.

```bash
ros2 bag info ~/under_guard_bags/trial_001
```

## 5. Replay JSON으로 변환

저장소의 backend 디렉터리로 이동한다.

```bash
cd ~/Desktop/Sysmon_inte/Sysmon/backend
```

가장 기본적인 변환 명령은 다음과 같다.

```bash
python3 convert_rosbag_to_replay.py \
  ~/under_guard_bags/trial_001 \
  data/trial_001_replay.json \
  --model field_algorithm_v1 \
  --goal-name bottom
```

기존 예제의 배경 지도와 포획 지점을 복사하려면 `--base-replay`를 추가한다.

```bash
python3 convert_rosbag_to_replay.py \
  ~/under_guard_bags/trial_001 \
  data/trial_001_replay.json \
  --model field_algorithm_v1 \
  --goal-name bottom \
  --base-replay system_monitor/replay_data/real_map_frames.json
```

기본 출력 간격은 0.1초다. 20Hz로 만들고 싶다면 `--sample-period 0.05`를 사용한다.

## 6. 실제 토픽 이름이 다를 때

예를 들어 로봇 위치가 `odometry/filtered`에 있고 쥐 위치가 별도 PoseStamped로
발행된다면 다음처럼 지정한다. 사용하지 않는 `/fleet/event`는 빈 문자열로 끈다.

```bash
python3 convert_rosbag_to_replay.py BAG_DIR OUTPUT.json \
  --driver-odom /robot4/odometry/filtered \
  --blocker-odom /robot6/odometry/filtered \
  --target-event "" \
  --target-pose /herding/target_pose \
  --driver-goal /herding/driver_goal \
  --blocker-goal /herding/blocker_goal \
  --state-topic /herding/state \
  --progress-topic /herding/capture_progress \
  --success-topic /herding/success
```

모든 옵션은 다음 명령으로 확인할 수 있다.

```bash
python3 convert_rosbag_to_replay.py --help
```

## 7. 같은 JSON에 다음 시험 추가

첫 시험을 변환한 뒤 두 번째 시험을 같은 파일에 추가하려면 기존 출력 파일을
`--base-replay`로 다시 지정한다. 쓰기 도중 문제가 생겨도 기존 파일이 반쯤
잘리지 않도록 임시 파일을 완성한 뒤 원자적으로 교체한다.

```bash
python3 convert_rosbag_to_replay.py \
  ~/under_guard_bags/trial_002 \
  data/field_trials.json \
  --model field_algorithm_v1 \
  --goal-name top \
  --base-replay data/field_trials.json \
  --append-existing-trials \
  --force
```

`--force`가 없으면 기존 출력 파일을 덮어쓰지 않는다. 원본 rosbag은 어떤 경우에도
수정하거나 삭제하지 않는다.

## 8. 변환 결과를 System Monitor에서 확인

생성한 파일을 Replay 모드에서 지정한다.

```bash
cd ~/Desktop/Sysmon_inte/Sysmon/backend
export REPLAY_FRAMES_PATH="$PWD/data/trial_001_replay.json"
export ROBOT_NAMESPACES=robot4,robot6
./run_replay.sh
```

브라우저에서 `http://localhost:5000`에 접속한 뒤 다음 순서로 확인한다.

```text
기록 조회 → 쥐몰이 기록 → 시험 선택 → 재생
```

확인할 항목은 다음과 같다.

- 쥐·Driver·Blocker 경로가 배경 지도와 같은 위치에 있는가?
- Driver와 Blocker 목표 점선이 실제 경로와 구분되는가?
- FSM 상태가 시험 진행과 맞게 바뀌는가?
- 마지막 상태와 성공 여부가 실제 시험 결과와 같은가?
- 변환 명령이 출력한 `주의:` 메시지가 있는가?

## 9. 자주 발생하는 오류

### `ROS 2 Python 모듈을 찾을 수 없습니다`

ROS 환경을 불러오지 않은 터미널이다. 다음 두 줄을 다시 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot4_ws/install/setup.bash
```

### `rosbag에 필수 토픽이 없습니다`

`ros2 bag info BAG_DIR`로 실제 토픽 이름을 확인하고 `--driver-odom`,
`--blocker-odom`, `--target-event` 또는 `--target-pose`를 맞춘다.

### `프레임 생성에 필요한 좌표가 없습니다: target`

bag에 `/fleet/event`가 있어도 `rat_detected:x:y` 메시지가 실제로 발행되지 않은
경우다. 현재 main의 쥐 감지 TODO 구현 여부를 확인해야 한다.

### map과 다른 frame 경고

좌표를 수치만 보고 억지로 합치면 경로가 틀린 위치에 그려진다. 알고리즘 또는
로봇 쪽에서 TF를 적용한 map 좌표 토픽을 제공하는 것이 가장 안전하다.

## 10. 실제 실행 중 확인된 오류 사례

작업일: 2026-08-10

다음과 같은 오류가 발생했다.

```text
bash: cd: Sysmon/backend: 그런 파일이나 디렉터리가 없습니다
오류: ROS 2 Python 모듈을 찾을 수 없습니다.
```

### 원인 1 — 이미 backend 폴더에 있음

터미널 프롬프트가 다음 경로를 표시하고 있었다.

```text
~/Desktop/Sysmon_inte/Sysmon/backend
```

이 상태에서 `cd Sysmon/backend`를 다시 실행하면 현재 폴더 아래의
`Sysmon/backend`를 찾는다. 그런 하위 폴더는 없으므로 오류가 발생한다.

현재 위치는 다음 명령으로 확인한다.

```bash
pwd
```

출력이 아래와 같다면 추가 `cd` 없이 바로 변환 명령을 실행하면 된다.

```text
/home/rokey/Desktop/Sysmon_inte/Sysmon/backend
```

### 원인 2 — ROS 2 환경을 현재 터미널에 불러오지 않음

이 PC에는 ROS 2 Humble이 `/opt/ros/humble`에 설치되어 있지만 당시 터미널의
`ROS_DISTRO`와 `AMENT_PREFIX_PATH`가 비어 있었다. 설치되어 있는 것과 현재
터미널에 환경을 불러오는 것은 별개의 과정이다.

같은 터미널에서 다음 명령을 먼저 실행한다.

```bash
source /opt/ros/humble/setup.bash

python3 -c "import rosbag2_py, rclpy; print('ROS 환경 정상')"
```

`ROS 환경 정상`이 출력되면 변환기가 rosbag2를 읽을 수 있는 상태다. bag에
프로젝트 전용 커스텀 메시지가 있다면 그 메시지를 빌드한 워크스페이스의
`install/setup.bash`도 추가로 source해야 한다. 현재 변환기의 기본 입력은
`String`, `Odometry`, `PoseStamped` 같은 ROS 표준 메시지라 Humble 환경만으로도
읽을 수 있다.

### 원인 3 — 예제 rosbag 경로가 실제로 존재하지 않음

문서의 다음 경로는 사용 방법을 보여주기 위한 예시다.

```text
~/under_guard_bags/trial_001
```

확인 당시 이 PC에는 해당 디렉터리와 rosbag의 `metadata.yaml`이 없었다. ROS
환경 오류를 해결한 다음에는 bag 경로 오류가 이어서 발생할 수 있으므로 실제
기록 위치를 확인해야 한다.

```bash
find ~ -name metadata.yaml -type f
```

아직 bag이 없다면 먼저 4절의 기록 명령으로 시험을 저장한다. bag 기록이 끝난
뒤에는 다음 명령이 성공해야 한다.

```bash
ros2 bag info ~/under_guard_bags/trial_001
```

### 이 PC에서 다시 실행할 정확한 순서

현재 위치가 이미 backend일 때는 다음 명령만 순서대로 실행한다. 명령의
`python3 convert_rosbag_to_replay.py` 줄을 두 번 입력하지 않도록 주의한다.

```bash
source /opt/ros/humble/setup.bash

python3 -c "import rosbag2_py, rclpy; print('ROS 환경 정상')"

ros2 bag info ~/under_guard_bags/trial_001

python3 convert_rosbag_to_replay.py \
  ~/under_guard_bags/trial_001 \
  data/trial_001_replay.json \
  --model field_algorithm_v1 \
  --goal-name bottom
```

`ros2 bag info`가 실패하면 변환부터 실행하지 말고 실제 bag 경로를 먼저 찾아야
한다. 또한 기록 전에 `/fleet/event`에서 `rat_detected:x:y`가 실제로 나오는지
확인해야 한다. 현재 main의 쥐 감지 부분이 TODO라서 이 메시지가 없으면 bag은
있어도 쥐 경로 프레임을 만들 수 없다.
