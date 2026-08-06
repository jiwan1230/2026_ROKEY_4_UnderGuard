# Flask System Monitor Starter

[`rokey_4_mini`](https://github.com/12tmdwo/rokey_4_mini/tree/main/mini_turtle4)의 ROS 2 인터페이스를 참고해 만든 Flask 기반 다중 로봇 관제 스타터입니다.

원본 프로젝트는 웹캠 Detection의 `map` 좌표로 대상에 접근하고, OAK-D가 같은 대상을 탐지하면 OAK-D 좌표 기반 추적으로 전환합니다. OAK-D가 대상을 1.5초 동안 놓치면 Nav2 Goal을 취소하고 정지합니다. `dommy` 탐지는 `dummy_cloud` PointCloud2로 변환되어 Nav2 Costmap 장애물로 사용됩니다.

이 스타터는 원본 구조를 분석했지만 모든 ROS 기능을 연결한 완성본은 아닙니다. 현재 Mock에서 확인된 기능과 실제 ROS에서 추가 검증할 기능을 아래와 같이 구분합니다.

## Mock 모드에서 구현 및 확인된 기능

- Flask 로그인·로그아웃
- 터틀봇 2대 Mock 상태 모니터링
- 두 로봇의 공동 탐색과 최초 쥐 탐지 기반 역할 자동 배정
  - 최초 탐지 로봇: `RAT_TRACKER` · 쥐 추적
  - 나머지 로봇: `SURVEY_TRAP` · 쥐구멍 탐색·트랩 설치
- Mock 자동 시나리오와 수동 테스트 이벤트
- 실제 `my_map.pgm/yaml` 배경을 사용하는 Canvas 관제 지도
- 로봇 위치·방향, 현재 대상, 과거 탐지 위치 표시
- 두 로봇의 카메라·탐지 상태 UI
  - 현재 화면은 `MOCK VIDEO` 자리 표시 화면이며 실제 영상 스트리밍이 아님
- 브라우저에서 1초마다 `/api/snapshot`을 요청하는 준실시간 상태·이벤트 갱신
- 배터리 부족·대상 유실·탐지·트랩 설치 위치 저장 테스트
- SQLite3 탐지 결과와 이벤트 저장
- 객체·로봇·날짜·검토 상태별 탐지 검색
- 탐지 결과 검토 상태와 메모 변경
- 관리자 전용 운영 데이터 초기화
  - 사용자·스키마·맵 설정은 유지하고 Mock/ROS 탐지·사건·트랩을 초기화
  - Mock은 임무를 대기시키고 ROS는 수집 노드와 현재 로봇 상태를 유지
- Mock/ROS 실행 모드 분리와 공통 RuntimeService 계약
- 앱·맵 변환·상태 관리자·Mock 시나리오·ROS/Fleet 변환·DB Unit Test 44개

## Mock/ROS 공통 동작 계약

Flask 라우트는 실행 모드를 직접 분기하지 않고 선택된 `RuntimeService`를 사용합니다.
두 모드는 `/api/health`, `/api/snapshot`, `/api/commands`에서 같은 응답 필드를
반환하며, 실제 지원 여부만 `runtime` capability 값으로 구분합니다.

| 항목 | Mock | ROS |
|---|---|---|
| 상태·탐지·사건 응답 구조 | 동일 | 동일 |
| 위험신호 저장 이름 | `LIVE_RODENT`, `ENTRY_POINT`, `DROPPINGS` | 동일 |
| 최초 쥐 탐지 역할 배정 | 구현 | 구현 |
| 대상 유실과 마지막 위치 유지 | 수동/시나리오 | OAK-D 1.5초 timeout |
| 저전압 사건 중복 방지 | 수동/시나리오 | BatteryState 임계값 감시 |
| 웹 명령 응답 구조 | `accepted: true` | main Fleet 지원 명령만 `accepted: true` |
| 임무 진행률 | 시뮬레이션 값 | 실제 Feedback 연동 전 `—` |

ROS 모드는 main의 `/fleet/command` String 계약에 있는 명령만 송신합니다. `PAUSE`,
`INSTALL_TRAP`처럼 main에 없는 명령은 capability에서 제외해 화면에서 비활성화하며,
Mock 전용 사건 버튼은 ROS 데이터에 가짜 사건이 섞이지 않도록 숨깁니다.

## ROS 모드 기본 골격

현재 `system_monitor/ros_bridge.py`에 구현된 구독은 다음과 같습니다.

| 토픽 | 구현 상태 |
|---|---|
| `/fleet/status` | main의 `robot:state:battery` 구독 및 상태 변환 구현 |
| `/fleet/event` | main의 `event:x:y` 구독 및 탐지·트랩 저장 구현 |
| `/fleet/command` | main이 지원하는 명령 발행 구현 |
| `/<namespace>/webcam/detections` | `vision_msgs/Detection3DArray` 구독 구현 |
| `/<namespace>/oakd/detections` | `vision_msgs/Detection3DArray` 구독 구현 |
| `/<namespace>/odom` | `nav_msgs/Odometry` 구독 코드는 있으나 실제 장비 토픽 확인 필요 |
| `/<namespace>/battery_state` | `sensor_msgs/BatteryState` 구독 코드는 있으나 실제 장비 토픽 확인 필요 |
| `/<namespace>/dummy_cloud` | 미구현 · 후속 연결 필요 |

위 토픽명은 코드에 고정하지 않고 환경변수로 주입합니다. main Fleet String 계약을
1순위로 사용하고, Detection3DArray·odom·BatteryState는 실제 메시지 패키지와 토픽이
존재할 때 사용하는 보조 입력입니다.

Detection 결과의 상태 변환, DB 저장, 최초 탐지 기반 역할 배정은 구현되어 있습니다.
OAK-D에서 살아있는 설치류가 1.5초 동안 다시 탐지되지 않으면 `TARGET_LOST` 사건을
한 번 생성하고 마지막 위치를 유지합니다. 저전압 역시 임계값 진입 시 한 번만 사건을
생성하고 정상 범위로 회복한 뒤 재진입했을 때 다시 생성합니다.

## 아직 실제 연동이 필요한 기능

- 실제 장비의 `robot4`, `robot6` namespace 및 토픽 발행 확인
- 실시간 SLAM `/map` 토픽과 OccupancyGrid 갱신
- TF 기반 `map → base_link` 로봇 위치
- Nav2 계획 경로 표시
- 실제 OAK-D·웹캠 영상 및 Bounding Box 스트리밍
- `dummy_cloud` 구독과 상태 표시
- `NavigateToPose` Feedback·Result
- 역할 자동 배정 결과와 main 중앙 조율 상태의 단일 소유권 확정
- main Fleet에 없는 일시정지·트랩 설치 명령 계약 추가
- WebSocket 또는 Flask-SocketIO 기반 실시간 Push 통신

## Mock 모드 실행

가상환경을 만들지 않고 사용자 Python 패키지 경로에 설치합니다.

```bash
cd rokey_4_mini_sysmon_starter

python3 -m pip install --user -r requirements.txt
./run_mock.sh
```

기본 맵 경로는 현재 작업 환경에서 확인된 다음 파일입니다.

```text
../minipjt/mini_turtle4/resource/my_map.yaml
```

다른 맵을 사용할 때는 YAML 경로만 지정하면 같은 폴더의 PGM을 자동으로 읽습니다.

```bash
export MAP_YAML_PATH=/path/to/my_map.yaml
./run_mock.sh
```

브라우저가 PGM을 직접 지원하지 않아도 서버가 Pillow로 PNG로 변환합니다.
YAML의 `resolution`, `origin`으로 로봇·이동 경로·탐지·배설물·설치된
덫의 실제 좌표를 Canvas 위치로 변환합니다. 맵 파일이 없거나 잘못된 경우에는 기존
좌표 격자로 자동 대체합니다.

브라우저에서 다음 주소로 접속합니다.

```text
http://localhost:5000
```

초기 계정:

```text
ID: admin
PW: admin123
```

## 테스트

```bash
python3 -m unittest discover -s tests -v
```

현재 앱 생명주기, 모드 공통 API 계약, PGM/YAML 변환, 표준 위험신호,
Mock 역할 배정·트랩 위치, ROS 탐지 좌표계·대상 유실·저전압 처리,
DB 초기화 안전조건과 main Fleet 상태·사건·명령 변환을 포함한 총 44개 테스트가
통과합니다.

## Mock/ROS 운영 데이터 초기화

관리자 계정으로 `탐지 DB` 화면을 열고 `운영 데이터 초기화`를 선택합니다.
두 모드 모두 탐지·사건·트랩 기록을 한 트랜잭션으로 삭제하며 관리자 계정,
DB 테이블 구조, 맵과 환경설정은 유지합니다. Mock은 현재 임무도 초기화해
`임무 시작` 전까지 자동 시나리오를 대기시킵니다. ROS는 수집 노드와 현재 로봇의
연결·위치·임무 상태를 유지하고, 초기화 이후 수신되는 새 토픽부터 다시 저장합니다.

## ROS 모드 실행

ROS 2 Humble과 프로젝트 워크스페이스를 먼저 로드합니다. 아래
`turtlebot4_ws` 경로는 실제 장비 환경에 맞게 변경해야 합니다.

```bash
cd rokey_4_mini_sysmon_starter

python3 -m pip install --user -r requirements.txt

source /opt/ros/humble/setup.bash
source ~/turtlebot4_ws/install/setup.bash

export ROBOT_NAMESPACES=robot4,robot6
export ROBOT_ROLES=SCOUT,SCOUT

./run_ros.sh
```

실행 전 다음 명령으로 실제 토픽 이름과 타입을 확인해야 합니다.

```bash
ros2 topic list -t
```

특히 아래 후보 토픽의 존재 여부를 확인합니다.

```text
/robot4/odom
/robot4/battery_state
/robot6/odom
/robot6/battery_state
```

## 프로젝트 구조

```text
rokey_4_mini_sysmon_starter/
├── system_monitor/        # Flask 앱, 상태 모델, Mock·ROS Bridge
├── system_monitor/static/ # JavaScript, CSS, 이미지
├── system_monitor/templates/
├── docs/                  # 인터페이스·상태·개선 기록
├── tests/                 # Unit Test
├── requirements.txt
├── run_mock.sh
└── run_ros.sh
```

기존 `rokey_4_mini` 저장소에 통합할 때는 저장소 루트에 `system_monitor/`, `docs/`, `tests/`, `requirements.txt`, `run_mock.sh`, `run_ros.sh`를 추가하는 구성을 권장합니다.

사업 목표와 BR-01~BR-15, Pilot KPI, 현재 스타터 구현의 추적 관계는 [`docs/UNDERGUARD_BUSINESS_REQUIREMENTS_V4_1.md`](docs/UNDERGUARD_BUSINESS_REQUIREMENTS_V4_1.md)를 기준으로 확인합니다.

## 운영 전 확인 사항

- 초기 관리자 비밀번호와 `SECRET_KEY` 변경
- 실제 robot namespace와 상태 토픽 확인
- ROS Action·Service 명세와 안전 정책 확정
- 실제 카메라·SLAM·TF 연동 시험
- 개발용 Flask 서버를 운영용 WSGI 서버로 교체
