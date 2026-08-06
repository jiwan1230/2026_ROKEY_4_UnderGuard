# UnderGuard System Monitor

두 대의 AMR 상태와 위치, 카메라 상태, 탐지 이벤트를 한 화면에서 확인하는 Flask 기반 실시간 관제 UI입니다. `main`의 Fleet 인터페이스를 우선 사용하며 Mock과 ROS 모드는 같은 상태 모델과 화면을 공유합니다.

현재 System Monitor는 데이터를 소유하지 않습니다. 로그인, 로컬 SQLite, 탐지 기록 조회·검색·검토, 데이터 초기화 기능은 범위에서 제외했습니다. 수신한 탐지와 이벤트는 화면 표시를 위해 프로세스 메모리에만 잠시 유지되며 서버를 재시작하면 사라집니다.

## 현재 구현 범위

- 두 로봇의 연결, 상태, 배터리, 속도와 현재 작업 표시
- 정적 `my_map.pgm/yaml` 배경과 실제 좌표 변환
- 로봇 위치·방향·이동 경로 표시
- 현재 실행 중 수신한 탐지·덫 마커와 최근 이벤트 표시
- `LIVE_RODENT`, `ENTRY_POINT`, `DROPPINGS` 표준 위험신호 변환
- 최초 쥐 탐지 기반 추적/지원 역할 표시
- 대상 유실 시 마지막 표적 위치 유지
- Offline 및 저전압 경고 표시
- Mock 자동 시나리오와 수동 테스트 이벤트
- main Fleet 상태·사건의 읽기 전용 수신

카메라 카드는 현재 상태 UI와 Mock 자리 표시 화면입니다. 실제 영상 스트리밍은 아직 연결되지 않았습니다.

## 데이터 경계

```text
Mock 또는 ROS 메시지
        ↓
RosBridge / MockManager
        ↓
StateManager (현재 프로세스 메모리)
        ↓
/api/snapshot
        ↓
실시간 관제 화면
```

- System Monitor가 영구 저장하거나 조회하는 DB는 없습니다.
- `/fleet/event`는 현재 화면의 마커와 이벤트로만 변환합니다.
- 로봇 측 DB가 구현되면 별도 계약을 정한 뒤 필요한 조회 기능만 연결합니다.
- 과거 이력, 검색, 검토, 보고서는 현재 개발 범위가 아닙니다.

## Mock/ROS 공통 계약

| 항목 | Mock | ROS |
|---|---|---|
| 화면·스냅샷 구조 | 공통 | 공통 |
| 위험신호 이름 | 표준값 사용 | 표준값 사용 |
| 사건 보관 | 프로세스 메모리 | 프로세스 메모리 |
| 테스트 사건 버튼 | 표시 | 숨김 |

ROS 입력과 제약은 [ROS 인터페이스 명세](docs/ROS_INTERFACE_SPEC.md), 현재 개발 순서는 [System Monitor 책임 범위](docs/SYSTEM_MONITOR_SCOPE.md)를 기준으로 확인합니다.

## 실행

이 문서 아래 `backend/`에 실제 실행 대상(Flask 패키지·스크립트)이 있습니다.

```bash
cd backend
python3 -m pip install --user -r requirements.txt
./run_mock.sh
```

접속 주소는 `http://localhost:5000`입니다. 로그인 단계는 없습니다.

ROS 모드는 ROS 2와 프로젝트 워크스페이스를 먼저 로드합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot4_ws/install/setup.bash
export ROBOT_NAMESPACES=robot4,robot6
export ROBOT_ROLES=SCOUT,SCOUT
cd backend
./run_ros.sh
```

실제 연결 전 `ros2 topic list -t`와 `ros2 topic echo /fleet/status`로 토픽 이름, 타입, 값이 main 계약과 맞는지 확인해야 합니다.

## 주요 ROS 인터페이스

| 토픽 | 처리 |
|---|---|
| `/fleet/status` | `robot:state:battery`를 연결·상태·배터리로 변환 |
| `/fleet/event` | `event:x:y`를 현재 세션의 탐지·덫 마커로 변환 |
| `/<namespace>/odom` | 위치·방향·속도 보조 입력, 실제 토픽 확인 필요 |
| `/<namespace>/battery_state` | 배터리 보조 입력, 실제 토픽 확인 필요 |
| `/<namespace>/*/detections` | 상세 탐지 보조 입력, 메시지 패키지 확인 필요 |

## 테스트

```bash
cd backend
python3 -m unittest discover -s tests -v
```

현재 앱 라우트, Mock/ROS 계약, 맵 변환, 상태 관리, Mock 시나리오, Fleet/ROS 메시지 변환을 포함한 35개 테스트가 통과합니다.

## 프로젝트 구조

저장소 루트에서 `src/`(main의 `turtle_project`/`turtle_interfaces`가 있는 곳)와
같은 층에 있으며, 화면(frontend)과 서버 로직(backend)을 물리적으로 분리합니다.

```text
Sysmon/
├── README.md                  # 이 문서
├── docs/
├── backend/
│   ├── requirements.txt
│   ├── .env.example
│   ├── run_mock.sh
│   ├── run_ros.sh
│   ├── tests/
│   └── system_monitor/
│       ├── app.py               # Flask 화면/API와 실행 수집기 조립 (frontend/ 폴더를 명시적으로 연결)
│       ├── config.py            # 실행 모드·로봇·맵·토픽 설정
│       ├── state_manager.py     # 현재 실행 중 상태의 메모리 집계
│       ├── detection_service.py # 탐지·경고를 공통 상태로 변환
│       ├── mock_manager.py      # 장비 없는 시연 데이터 생성
│       ├── ros_bridge.py        # ROS/Fleet 메시지의 읽기 전용 변환
│       ├── map_service.py       # PGM/YAML 로딩과 PNG 변환
│       └── runtime_service.py   # Mock/ROS 공통 인터페이스
└── frontend/
    ├── templates/dashboard.html
    └── static/
```

## 다음 통합 우선순위

1. 실제 `/fleet/status`로 두 로봇 상태·배터리 검증
2. 실제 TF/odom과 정적 맵의 좌표계 일치
3. `/fleet/event`의 로봇 ID 및 탐지 필드 계약 확정
4. Offline·재연결·대상 유실 현장 시험
5. 실제 카메라 영상 전달 방식 확정과 연결
6. 로봇 제어는 `central_node` 소유로 유지하고 관제 입력 계약 문서화

사업 요구사항 문서는 참고 자료이며 현재 구현 순서의 기준이 아닙니다.
