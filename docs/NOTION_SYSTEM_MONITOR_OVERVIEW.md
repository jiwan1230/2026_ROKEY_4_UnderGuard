# Flask System Monitor Starter

`rokey_4_mini`의 ROS 2 인터페이스를 기준으로 제작한 Flask 기반 다중 로봇 시스템 모니터링 스타터 프로젝트입니다.

원본 프로젝트는 외부 웹캠에서 탐지한 `map` 좌표로 대상에 접근한 뒤, 로봇의 OAK-D가 같은 대상을 탐지하면 OAK-D 좌표 기반 추적으로 전환합니다. OAK-D가 대상을 1.5초 동안 놓치면 Nav2 Goal을 취소하고 로봇을 정지합니다.

또한 웹캠에서 탐지한 `dommy` 객체를 `dummy_cloud` PointCloud2로 변환하여 Nav2 Costmap의 가상 장애물로 사용합니다.

> 현재 프로젝트는 Mock 환경에서 웹 관제 기능을 검증할 수 있는 스타터 프로젝트입니다. ROS 2 탐지·상태 구독 골격은 구현되어 있지만, 실제 로봇 제어 Action·Service 및 일부 센서 연동은 후속 작업이 필요합니다.

원본 프로젝트: <https://github.com/12tmdwo/rokey_4_mini/tree/main/mini_turtle4>

---

## 1. 시스템 동작 개요

### 원본 `rokey_4_mini` 추적 흐름

```text
외부 웹캠 탐지
    ↓
webcam/detections의 map 좌표로 접근
    ↓
OAK-D에서 같은 대상 탐지
    ↓
oakd/detections 기반 정밀 추적으로 전환
    ↓
1.5초 이상 대상 유실
    ↓
Nav2 Goal 취소 및 정지
```

### 가상 장애물 처리 흐름

```text
웹캠에서 dommy 객체 탐지
    ↓
webcam/detections
    ↓
dummy_obstacle_node
    ↓
dummy_cloud PointCloud2 발행
    ↓
Nav2 Costmap 장애물로 사용
```

### 시스템 모니터 역할

```text
ROS 2 또는 Mock 데이터
    ↓
ROS Bridge / Mock Manager
    ↓
State Manager
    ├── 로봇 상태 관리
    ├── 임무 및 역할 관리
    ├── 탐지 상태 관리
    └── 이벤트 관리
    ↓
Flask API
    ↓
웹 관제 화면
```

---

## 2. 구현된 기능

### Mock 모드에서 구현 및 확인된 기능

- Flask 세션 기반 로그인·로그아웃
- 기본 2대의 터틀봇 Mock 상태 모니터링
- 두 로봇의 공동 탐색 시나리오
- 최초 쥐 탐지 기반 역할 자동 배정
  - 최초 탐지 로봇: `RAT_TRACKER`
  - 나머지 로봇: `SURVEY_TRAP`
- Mock 자동 시나리오
- 브라우저에서 실행 가능한 수동 테스트 이벤트
- 실제 `my_map.pgm/yaml` 기반 로봇·탐지·트랩 좌표 관제 지도
- 로봇 위치·방향·이동 상태 표시
- 현재 대상 및 과거 탐지 위치 표시
- 로봇별 배터리·속도·연결·임무 상태 표시
- 카메라 연결 및 탐지 상태 영역
- 1초 Polling 기반 준실시간 상태 갱신
- 준실시간 이벤트 타임라인
- SQLite3 탐지 결과 및 시스템 이벤트 저장
- 탐지 결과 검색 및 검토
- Mock/ROS 실행 모드 분리와 공통 RuntimeService 계약
- 표준 위험신호·ROS 대상 유실·저전압 경고 처리
- 앱·맵 변환·상태 관리자·Mock 시나리오·ROS 탐지 변환·DB Unit Test 33개

> 카메라 영역은 현재 `MOCK VIDEO · 실제 영상 아님` 자리 표시 화면입니다. 실제 카메라 영상 스트리밍과 Bounding Box 표시는 아직 연결되지 않았습니다.

### Mock 테스트 이벤트

- 쥐 탐지
- 쥐구멍 탐지
- 배설물 탐지
- 대상 유실
- 배터리 부족
- 쥐덫 설치 완료

---

## 3. 탐지 결과 관리

탐지 결과는 SQLite3 데이터베이스에 저장됩니다.

### 저장 정보

- 탐지 로봇
- 객체 유형
- 탐지 신뢰도
- 대상까지의 거리
- `map` 좌표
- 탐지 소스: `WEBCAM`, `OAK-D`
- 검토 상태
- 검토 메모
- 탐지 시각

### 검색 조건

- 객체 유형
- 로봇
- 날짜 범위
- 검토 상태

### 검토 상태

| 상태 | 의미 |
|---|---|
| `UNREVIEWED` | 미검토 |
| `REVIEWED` | 검토 완료 |
| `ACTIONED` | 조치 완료 |
| `FALSE_POSITIVE` | 오탐 |

---

## 4. ROS 2 Bridge 구현 상태

현재 `system_monitor/ros_bridge.py`에는 다음 구독 구조가 구현되어 있습니다.

| 토픽 | 메시지 타입 | 구현 상태 | 용도 |
|---|---|---|---|
| `/<namespace>/webcam/detections` | `vision_msgs/Detection3DArray` | 구현 | 외부 웹캠 탐지 결과 |
| `/<namespace>/oakd/detections` | `vision_msgs/Detection3DArray` | 구현 | OAK-D 탐지 결과 |
| `/<namespace>/odom` | `nav_msgs/Odometry` | 후보 구현 | 로봇 위치·방향·속도 |
| `/<namespace>/battery_state` | `sensor_msgs/BatteryState` | 후보 구현 | 배터리 상태 |
| `/<namespace>/dummy_cloud` | `sensor_msgs/PointCloud2` | 미구현 | 가상 장애물 상태 |

`odom`과 `battery_state`는 TurtleBot 배포 환경에 따라 실제 토픽 이름이 다를 수 있으므로 현장에서 확인해야 합니다.

```bash
ros2 topic list -t
```

특히 다음 후보 토픽의 존재 여부를 확인합니다.

```text
/robot4/odom
/robot4/battery_state
/robot5/odom
/robot5/battery_state
```

### ROS Bridge에서 처리하는 내용

- Detection 메시지를 웹 관제 상태로 변환
- `header.frame_id`가 `map`인 탐지만 지도 좌표로 기록
- 탐지 결과를 SQLite3에 저장
- 탐지 이벤트 생성
- 최초 쥐 탐지 기반 역할 자동 배정
- Odometry 기반 로봇 위치·방향·속도 갱신
- BatteryState 기반 배터리 잔량 갱신
- 일정 시간 데이터가 없으면 로봇을 `OFFLINE`으로 처리

### 실제 로봇 제어 상태

현재 실제 로봇 제어용 Action·Service는 연결되어 있지 않습니다. ROS 모드에서 웹 UI가 제어 명령을 요청하면 `accepted: false`를 반환합니다.

> 원본 시스템의 “1.5초 대상 유실 시 Nav2 Goal 취소”는 기존 `goal_manager_node`의 동작입니다. 현재 Flask 시스템 모니터는 이 Goal 취소를 직접 실행하지 않습니다.

---

## 5. 관제 상태 정의

| 상태 | 의미 |
|---|---|
| `OFFLINE` | 제한 시간 동안 로봇 데이터가 수신되지 않음 |
| `IDLE` | 로봇이 연결되어 있지만 임무 대기 중 |
| `SEARCHING` | 쥐 또는 쥐구멍 탐색 중 |
| `APPROACHING` | 웹캠 좌표 기반으로 대상에 접근 중 |
| `TRACKING` | OAK-D 기반으로 대상을 추적 중 |
| `TARGET_LOST` | OAK-D가 추적 대상을 놓침 |
| `NAVIGATING` | Nav2를 이용해 이동 중 |
| `INSTALLING_TRAP` | 쥐덫 설치 작업 중 |
| `RETURNING` | 시작 위치 또는 Dock으로 복귀 중 |
| `PAUSED` | 임무 일시정지 |
| `COMPLETED` | 현재 임무 완료 |
| `ERROR` | 복구 판단이 필요한 오류 발생 |

---

## 6. 로봇 역할 정의

| 역할 | 의미 |
|---|---|
| `SCOUT` | 최초 탐지 전 공동 탐색 |
| `RAT_TRACKER` | 최초로 쥐를 탐지한 로봇, 쥐 추적 담당 |
| `SURVEY_TRAP` | 나머지 로봇, 쥐구멍 탐색 및 쥐덫 설치 담당 |
| `UNASSIGNED` | 역할이 배정되지 않은 상태 |

두 로봇은 기본적으로 `SCOUT` 역할로 시작합니다.

```text
robot4: SCOUT
robot5: SCOUT
```

최초 쥐 탐지가 발생하면 역할이 자동으로 변경됩니다.

```text
최초 탐지 로봇 → RAT_TRACKER
나머지 로봇   → SURVEY_TRAP
```

---

## 7. 프로젝트 구조

```text
rokey_4_mini_sysmon_starter/
├── system_monitor/
│   ├── __init__.py
│   ├── app.py
│   ├── config.py
│   ├── ros_bridge.py
│   ├── state_manager.py
│   ├── mock_manager.py
│   ├── database.py
│   ├── security.py
│   ├── templates/
│   │   ├── login.html
│   │   ├── dashboard.html
│   │   └── detections.html
│   └── static/
│       ├── css/
│       ├── js/
│       └── favicon.svg
├── docs/
│   ├── ROS_INTERFACE_SPEC.md
│   ├── STATUS_ENUM.md
│   ├── SYSTEM_MONITOR_SCOPE.md
│   ├── BEFORE_AFTER_IMPROVEMENTS.md
│   └── NOTION_SYSTEM_MONITOR_OVERVIEW.md
├── tests/
│   ├── test_state_manager.py
│   ├── test_mock_manager.py
│   └── test_database.py
├── .env.example
├── requirements.txt
├── run_mock.sh
└── README.md
```

> `system_monitor.db`는 프로그램 실행 시 생성되는 Runtime 데이터베이스 파일이므로 프로젝트 구조에서는 제외합니다.

---

## 8. 주요 파일 설명

| 파일 및 폴더 | 역할 |
|---|---|
| `system_monitor/` | Flask 시스템 모니터의 핵심 소스 코드 |
| `app.py` | Flask 서버 시작점, 로그인 세션, 화면 및 API 라우팅 관리 |
| `config.py` | 실행 모드, DB 경로, 로봇 namespace, 초기 역할 등 환경 설정 |
| `ros_bridge.py` | ROS 2 토픽 데이터를 웹 관제 상태로 변환 |
| `state_manager.py` | 로봇 위치·배터리·역할·임무·탐지·이벤트 상태 관리 |
| `mock_manager.py` | 실제 로봇 없이 자동 시나리오와 테스트 데이터를 생성 |
| `database.py` | 사용자·탐지 결과·시스템 이벤트의 SQLite3 저장 및 조회 |
| `security.py` | PBKDF2 기반 비밀번호 해시 생성 및 검증 |
| `templates/` | 로그인·대시보드·탐지 결과 HTML 화면 |
| `static/` | CSS, JavaScript, favicon 등 웹 화면 리소스 |
| `docs/` | ROS 인터페이스, 상태 규칙, 책임 범위 문서 |
| `tests/` | 상태 관리자·Mock Manager·DB Unit Test |
| `.env.example` | 환경변수 설정 예시 |
| `requirements.txt` | Flask 등 Python 의존성 목록 |
| `run_mock.sh` | Mock 모드 실행 스크립트 |
| `README.md` | 설치·실행·테스트·연동 방법 안내 |

---

## 9. Mock 모드 실행

### 패키지 설치

```bash
cd rokey_4_mini_sysmon_starter
python3 -m pip install --user -r requirements.txt
```

### 서버 실행

```bash
./run_mock.sh
```

또는 다음 명령으로 직접 실행할 수 있습니다.

```bash
export MONITOR_MODE=mock
python3 -m system_monitor.app
```

### 접속 정보

```text
URL: http://localhost:5000
ID: admin
PW: admin123
```

> 초기 계정은 개발·시연용입니다. 실제 운영 전 반드시 비밀번호와 `SECRET_KEY`를 변경해야 합니다.

---

## 10. 환경변수

```bash
MONITOR_MODE=mock
SECRET_KEY=change-this-before-demo
DATABASE_PATH=system_monitor.db
ROBOT_NAMESPACES=robot4,robot5
ROBOT_ROLES=SCOUT,SCOUT
OFFLINE_TIMEOUT_SEC=3.0
POLL_INTERVAL_MS=1000
```

| 환경변수 | 설명 |
|---|---|
| `MONITOR_MODE` | `mock` 또는 `ros` 실행 모드 |
| `SECRET_KEY` | Flask 세션 암호화 키 |
| `DATABASE_PATH` | SQLite3 데이터베이스 경로 |
| `ROBOT_NAMESPACES` | 쉼표로 구분한 로봇 namespace 목록 |
| `ROBOT_ROLES` | 로봇별 초기 역할 |
| `OFFLINE_TIMEOUT_SEC` | 로봇을 Offline으로 판단하는 시간 |
| `POLL_INTERVAL_MS` | 브라우저 상태 조회 주기 |

---

## 11. 실제 ROS 모드 실행

```bash
cd rokey_4_mini_sysmon_starter

python3 -m pip install --user -r requirements.txt

source /opt/ros/humble/setup.bash
source ~/turtlebot4_ws/install/setup.bash

export MONITOR_MODE=ros
export ROBOT_NAMESPACES=robot4,robot5
export ROBOT_ROLES=SCOUT,SCOUT

python3 -m system_monitor.app
```

다음 값은 실제 프로젝트 환경에 맞게 변경해야 합니다.

- `~/turtlebot4_ws` 워크스페이스 경로
- `robot4`, `robot5` namespace
- `odom`, `battery_state` 토픽 이름
- 두 로봇의 TF 및 Nav2 구성

---

## 12. Unit Test

```bash
python3 -m unittest discover -s tests -v
```

현재 총 33개 테스트가 통과합니다.

### 테스트 범위

- 기본 관리자 계정 생성 및 비밀번호 검증
- 탐지 결과 등록·조회·수정
- 날짜 범위 검색
- 잘못된 검토 상태 거부
- 로봇 상태 갱신
- Offline 상태 판단
- 지원하지 않는 상태값 검증
- Mock 탐지 좌표 생성
- 최초 탐지 기반 역할 자동 배정
- 역할별 명령 검증
- 오류 및 경고 상태 집계
- 앱 백그라운드 서비스의 명시적·중복 방지 시작
- ROS Detection3D의 map/센서 좌표계 구분
- Mock/ROS 런타임 및 명령 응답 계약 일치
- 세 가지 표준 위험신호 별칭 변환
- ROS OAK-D 대상 유실과 마지막 위치 유지
- ROS 저전압 사건 중복 방지
- ROS map YAML/PGM 로드와 PNG 변환
- 실제 좌표의 이미지 좌표 변환과 Y축 반전
- 트랩 설치 위치 DB 저장 및 지도 표시

---

## 13. 다음 작업: 미구현 및 현장 연동 항목

### 지도 및 위치

- [x] `my_map.pgm`, `my_map.yaml` 정적 지도 로드
- [x] YAML 해상도·원점 기반 좌표 변환
- [x] 로봇·경로·탐지·배설물·설치 트랩 마커
- [ ] 실제 SLAM `/map` 토픽 구독
- [ ] `nav_msgs/OccupancyGrid` 기반 지도 렌더링
- [ ] TF 기반 `map → base_link` 로봇 위치 계산
- [ ] Nav2 계획 경로 표시

### 카메라 및 탐지

- [ ] 실제 웹캠 영상 스트리밍
- [ ] 실제 OAK-D 영상 스트리밍
- [ ] Bounding Box 오버레이
- [ ] 영상과 Detection 메시지 시간 동기화

### ROS 상태 및 제어

- [ ] `dummy_cloud` 구독 및 장애물 상태 표시
- [ ] `NavigateToPose` Goal·Feedback·Result 연동
- [ ] 대상 유실 및 Goal 취소 상태 수신
- [ ] 임무 시작·중단·복귀 명령
- [ ] 쥐덫 설치 명령
- [ ] 비상정지 인터페이스
- [ ] 역할 자동 배정 결과를 실제 로봇에 전달

### 통신 및 운영

- [ ] WebSocket 또는 Flask-SocketIO 기반 Push 통신
- [ ] 운영용 WSGI 서버 적용
- [ ] HTTPS 구성
- [ ] 사용자 역할별 API 접근 권한
- [ ] 로그인 실패 제한 및 CSRF 보호
- [ ] 이벤트 이력 조회 API
- [ ] 로그 보존 및 데이터 백업 정책

---

## 14. 다른 PC에서 작업 재개하기

저장소를 다른 PC로 옮긴 뒤 다음 순서로 현재 상태를 확인합니다.

```bash
cd rokey_4_mini_sysmon_starter
python3 -m pip install --user -r requirements.txt
python3 -m unittest discover -s tests -v
./run_mock.sh
```

ROS 연동 전에는 실제 장비에서 다음 정보를 먼저 수집합니다.

```bash
ros2 topic list -t
ros2 node list
ros2 action list -t
ros2 service list -t
```

확인이 필요한 핵심 정보:

- [ ] 두 번째 로봇의 실제 namespace
- [ ] 로봇별 odometry 토픽과 타입
- [ ] 로봇별 배터리 토픽과 타입
- [ ] `/map`, `/tf`, `/tf_static` namespace 구성
- [ ] Nav2 `navigate_to_pose` Action 이름
- [ ] 웹캠 및 OAK-D 영상 토픽
- [ ] `dummy_cloud` 실제 토픽 이름과 발행 상태
- [ ] 임무 제어용 Action·Service 명세

---

## 15. 현재 프로젝트 범위 요약

| 구분 | 현재 상태 |
|---|---|
| Flask 로그인 및 접근 제어 | 구현 |
| 2대 로봇 Mock 관제 | 구현 |
| Mock 자동 시나리오 | 구현 |
| 역할 자동 배정 | 구현 |
| 실제 PGM/YAML 기반 Canvas 지도 | 구현 |
| 실시간 SLAM OccupancyGrid | 미구현 |
| 카메라 상태 UI | 구현 |
| 실제 영상 스트리밍 | 미구현 |
| 탐지 결과 DB 저장·검색·검토 | 구현 |
| ROS Detection 구독 | 구현 |
| ROS odom·배터리 구독 | 후보 구현, 현장 검증 필요 |
| `dummy_cloud` 구독 | 미구현 |
| Nav2 Action 연동 | 미구현 |
| 실제 로봇 제어 | 미구현 |
| Unit Test | 33개 통과 |

> 이 프로젝트는 Mock 환경에서 관제 화면과 데이터 처리 흐름을 검증하고, 이후 실제 TurtleBot4 ROS 2 시스템과 연결할 수 있도록 구성한 Flask 시스템 모니터 스타터입니다.
