# 시스템 모니터링 담당 파일 및 폴더 구조

이 문서는 UnderGuard 프로젝트에서 **시스템 모니터링(System Monitor)** 영역이
직접 관리하는 파일과 각 파일의 책임을 정리한다. 코드 리뷰와 기존 로봇 프로젝트
통합 시 이 문서를 기준으로 시스템 모니터링 코드의 경계를 확인한다.

## 1. 담당 범위

시스템 모니터링은 다음 기능을 담당한다.

- Flask 서버, 로그인 및 웹 API
- 두 로봇의 상태를 하나의 관제 스냅샷으로 집계
- Mock 모드와 ROS 모드의 공통 실행 계약
- ROS 메시지를 웹 화면용 상태로 변환
- 세 가지 위험신호(`LIVE_RODENT`, `ENTRY_POINT`, `DROPPINGS`) 정규화
- 탐지, 사건, 트랩 위치의 SQLite 저장과 검색
- 실제 ROS 맵 로드 및 좌표 변환
- 대시보드, 탐지 목록 및 검토 화면
- Mock 전체 사건 시나리오
- System Monitor 단위 테스트와 연동 문서

ROS 로봇 주행 노드, 센서 드라이버, Nav2 설정, 실제 카메라 노드는 이 폴더의
직접 담당 범위가 아니다. 해당 코드와의 접점은 `config.py`, `ros_bridge.py`,
`.env.example`, `docs/ROS_INTERFACE_SPEC.md`에 모아 둔다.

## 2. 전체 폴더 구조

저장소 루트에서 `src/`(main의 `turtle_project`/`turtle_interfaces`가 있는 곳)와
같은 층에 있는 `Sysmon/`이며, 화면(frontend)과 서버 로직(backend)을 물리적으로
분리한다.

```text
Sysmon/
├── README.md                          # 설치·실행·현재 구현 상태
├── docs/                              # 설계·범위·검증 문서
│   ├── MODE_PARITY.md                 # Mock/ROS 모드 정합성
│   ├── ROS_INTERFACE_SPEC.md          # ROS 토픽·좌표계·명령 접점 초안
│   ├── STATUS_ENUM.md                 # 로봇·임무·통신 상태 정의
│   ├── SYSTEM_MONITOR_FILE_STRUCTURE.md # 현재 문서
│   └── UNDERGUARD_BUSINESS_REQUIREMENTS_V4_1.md
│                                        # 사업 요구사항과 구현 추적
├── backend/
│   ├── .env.example                   # 실행 환경변수 예시
│   ├── requirements.txt               # Python 의존 패키지
│   ├── run_mock.sh                    # Mock 모드 실행 진입점
│   ├── run_ros.sh                     # ROS 모드 실행 진입점
│   ├── system_monitor/                # 시스템 모니터링 Flask 패키지
│   │   ├── __init__.py                # Python 패키지 선언
│   │   ├── app.py                     # Flask 앱 조립(../../frontend를 template/static으로 연결), 화면·API 라우트
│   │   ├── config.py                  # 환경변수와 ROS 인터페이스 설정
│   │   ├── detection_service.py       # Mock/ROS 공통 탐지·사건 처리
│   │   ├── map_service.py             # ROS 맵 로드와 좌표 변환
│   │   ├── mock_manager.py            # Mock 상태 갱신과 데모 시나리오
│   │   ├── risk_signals.py            # 위험신호 이름 표준화
│   │   ├── ros_bridge.py              # ROS 구독과 관제 상태 변환
│   │   ├── runtime_service.py         # Mock/ROS 공통 인터페이스
│   │   └── state_manager.py           # 로봇·사건·탐지 통합 상태
│   └── tests/                         # System Monitor 단위 테스트
│       ├── test_app.py                # Flask 및 모드 공통 API 계약
│       ├── test_map_service.py        # 맵 로드와 좌표 변환
│       ├── test_mock_manager.py       # Mock 시나리오와 역할 배정
│       ├── test_ros_bridge.py         # ROS 메시지 변환과 자동 경고
│       └── test_state_manager.py      # 통합 상태와 명령 상태 전이
└── frontend/
    ├── templates/
    │   └── dashboard.html             # 메인 관제 화면 구조
    └── static/
        ├── css/
        │   └── dashboard.css          # 전체 웹 화면 스타일
        ├── js/
        │   └── dashboard.js           # 관제 화면 갱신·지도·명령 처리
        └── favicon.svg                # 웹 브라우저 아이콘
```

로그인, SQLite 저장, 탐지 목록 검토 화면은 이전 시안에 있었으나 "팀원의
read-only 전환 결정"에 따라 삭제되어 현재 소스에는 없다(`database.py`,
`security.py`, `templates/login.html`, `templates/detections.html`,
`static/js/detections.js`, `tests/test_database.py` 등). 현재 담당 범위는
`docs/SYSTEM_MONITOR_SCOPE.md`를 기준으로 확인한다.

## 3. 실행 및 설정 파일

| 파일 | 역할 | 주요 입력 | 주요 출력·효과 |
|---|---|---|---|
| `.env.example` | 서버, DB, 로봇 namespace, ROS 토픽, 맵 경로의 설정 예시 | 운영 환경별 값 | `Settings`가 읽을 환경변수 기준 |
| `.gitignore` | DB, 캐시, 가상환경 등 커밋하지 않을 파일 지정 | 파일 경로 패턴 | Git 추적 대상 정리 |
| `README.md` | 설치, Mock/ROS 실행, 구현·미구현 상태 설명 | 프로젝트 현재 상태 | 개발자 실행 안내 |
| `requirements.txt` | Flask, Pillow 등 Python 의존성 고정 | `pip` 설치 명령 | 실행 가능한 Python 환경 |
| `run_mock.sh` | 실행 모드를 `mock`으로 고정해 서버 시작 | 환경변수, Python | Mock Flask 서버 |
| `run_ros.sh` | `rclpy` 및 ROS 환경을 확인하고 `ros` 모드로 시작 | source된 ROS 환경 | ROS Bridge가 포함된 Flask 서버 |

## 4. 백엔드 핵심 파일

| 파일 | 책임 | 코드 리뷰 핵심 |
|---|---|---|
| `system_monitor/__init__.py` | 패키지 선언 | 애플리케이션 코드가 `system_monitor` 패키지로 import됨 |
| `system_monitor/app.py` | Flask 앱 생성, 로그인·로그아웃, 화면과 API 라우트, 런타임 시작 | 라우트가 Mock/ROS를 직접 분기하지 않고 공통 `RuntimeService`를 사용 |
| `system_monitor/config.py` | 환경변수 검증, 로봇별 namespace와 토픽 이름 생성 | 로봇 저장소 변경 시 우선 수정할 통신 경계 |
| `system_monitor/detection_service.py` | 탐지 저장, 상태 갱신, 사건 생성, 대상 유실·저전압·트랩 처리 | Mock과 ROS가 같은 처리 순서를 사용하게 만드는 공통 계층 |
| `system_monitor/map_service.py` | `my_map.yaml`/PGM 읽기, PNG 변환, map 좌표를 이미지 픽셀로 변환 | `resolution`, `origin`, Y축 반전을 사용한 좌표 변환 |
| `system_monitor/mock_manager.py` | 두 로봇 이동, 예약 탐지, 역할 배정, 테스트 사건을 재현 | ROS 장비 없이 화면·DB·사건 전체 흐름을 검증 |
| `system_monitor/risk_signals.py` | 센서별 객체 라벨을 표준 위험신호로 변환 | DB와 UI가 세 가지 표준 이름만 사용하도록 정규화 |
| `system_monitor/ros_bridge.py` | Detection3DArray, Odometry, BatteryState 구독 및 공통 상태 변환 | 실제 토픽·메시지·좌표계가 연결되는 핵심 ROS 통신 파일 |
| `system_monitor/runtime_service.py` | `start`, `stop`, `snapshot`, `command` 공통 계약 정의 | Flask와 실행 모드의 결합도를 낮추는 경계 |
| `system_monitor/state_manager.py` | 로봇, 위치, 대상, 사건, 탐지, 트랩을 단일 스냅샷으로 집계 | 최초 쥐 탐지 기반 역할 배정, 상태 검증, Offline 판정의 중심 |

## 5. 웹 화면 파일 (`frontend/`)

| 파일 | 역할 |
|---|---|
| `frontend/templates/dashboard.html` | KPI, 로봇 카드, 지도, 카메라, 타임라인 UI의 HTML 구조 |
| `frontend/static/css/dashboard.css` | 관제 화면의 공통 색상, 크기, 반응형 배치 스타일 |
| `frontend/static/js/dashboard.js` | `/api/snapshot` 폴링, 상태 카드·카메라·지도 렌더링, 지도 마커 상세 처리 |
| `frontend/static/favicon.svg` | 웹 브라우저 아이콘 |

## 6. 테스트 파일 (`backend/tests/`)

| 파일 | 검증 내용 |
|---|---|
| `tests/test_app.py` | 백그라운드 서비스 시작, Mock/ROS 응답 구조와 공통 화면 계약 |
| `tests/test_map_service.py` | YAML/PGM 로드, PNG 변환, 실제 좌표 변환, 맵 누락 시 fallback |
| `tests/test_mock_manager.py` | 탐지 좌표, 최초 탐지 역할 배정, 표준 사건 이름, 트랩 기록, 임무 완료 유지 |
| `tests/test_ros_bridge.py` | 토픽 생성, 좌표 frame 보존, Detection/Odometry 변환, 대상 유실과 저전압 중복 방지 |
| `tests/test_state_manager.py` | 상태 갱신·검증, 명령 전이, 활성 경고, 역할 배정, 역할별 명령 검증 |

테스트 실행 명령은 다음과 같다(`backend/`가 CWD여야 `system_monitor` 패키지를 import할 수 있다).

```bash
cd backend
python3 -m unittest discover -s tests -v
```

## 7. 문서 파일

| 파일 | 용도 |
|---|---|
| `docs/MODE_PARITY.md` | Mock/ROS 공통 기능과 의도적으로 다른 기능의 기준 |
| `docs/ROS_INTERFACE_SPEC.md` | 현재 ROS 토픽 후보, 좌표 처리, 향후 Action·Service 연결 위치 |
| `docs/STATUS_ENUM.md` | 상태 문자열의 의미와 허용값 |
| `docs/SYSTEM_MONITOR_FILE_STRUCTURE.md` | 담당 파일, 구조, 책임 경계를 정리한 현재 문서 |
| `docs/UNDERGUARD_BUSINESS_REQUIREMENTS_V4_1.md` | 사업 요구사항과 모니터링 구현의 추적 관계 |

## 8. 소스가 아닌 실행 산출물

다음 파일은 현재 폴더에 존재할 수 있지만 시스템 모니터링의 **관리 대상 소스**는
아니다.

| 경로 | 설명 | Git 처리 |
|---|---|---|
| `system_monitor.db` | 실행 중 생성되는 사용자·탐지·사건·트랩 SQLite 데이터 | 제외, 환경마다 새로 생성 |
| `__pycache__/`, `*.pyc` | Python 실행 캐시 | 제외 |
| `.pytest_cache/` | 테스트 도구 캐시 | 제외 |
| `.venv/`, `venv/` | 로컬 Python 가상환경 | 제외 |
| `.vscode/` | 개인 VS Code 설정과 코드 탐색 DB | 현재 통합 커밋에서 제외 권장 |

## 9. 외부에서 받아 사용하는 파일

`my_map.yaml`과 `my_map.pgm`은 시스템 모니터가 화면에 표시하지만 로봇/SLAM
영역에서 제공받는 데이터다. 시스템 모니터는 `MAP_YAML_PATH`로 해당 YAML을
참조하며, YAML과 같은 폴더의 PGM을 읽는다. 향후 실제 토픽과 맵 정보가 확정되면
로봇 코드를 복사해 넣는 대신 아래 접점만 맞춘다.

1. `.env.example`의 namespace, 토픽, frame, 맵 경로
2. `system_monitor/config.py`의 설정 규칙
3. `system_monitor/ros_bridge.py`의 메시지 구독·변환
4. `docs/ROS_INTERFACE_SPEC.md`의 확정 명세

이 구조를 유지하면 시스템 모니터링 코드와 로봇 동작 코드가 직접 뒤섞이지 않아
코드 리뷰와 후속 통합이 쉬워진다.
