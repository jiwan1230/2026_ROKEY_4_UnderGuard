# System Monitor 전체 파일 역할 정리

갱신일: 2026-08-11

## 1. 문서 목적

`Sysmon/` 아래에 현재 존재하는 파일을 기준으로 각 파일의 역할을 간략하게
정리한다. 새 파일을 추가하거나 역할을 변경하면 이 문서도 함께 갱신한다.

이 목록은 다음 항목을 구분한다.

- 실행 소스와 설정
- 프론트엔드
- 자동 테스트
- 설계·운영 문서
- 실행 중 생성되거나 예제로 포함된 데이터

## 2. 전체 실행 흐름

```text
run_mock.sh / run_ros.sh / run_replay.sh
                    │
                    ▼
          system_monitor/app.py
                    │
        ┌───────────┼───────────┐
        ▼           ▼           ▼
  MockManager   RosBridge   ReplayManager
        └───────────┼───────────┘
                    ▼
              StateManager
                    │
                    ▼
             Flask JSON API
                    │
                    ▼
 dashboard.html + dashboard.js + dashboard.css
```

실시간 상태는 주로 `StateManager` 메모리에 있고, 과거 탐지와 로봇 이동 기록은
`HistoryStore`가 SQLite와 이미지 파일로 별도 보관한다. 지도와 카메라는 각각
`MapService`, `CameraFrameStore`가 담당한다.

## 3. 최상위 파일

| 파일 | 역할 |
|---|---|
| `README.md` | System Monitor의 기능 범위, 화면 구조, Mock/ROS/Replay 실행법, ROS 인터페이스와 테스트 방법을 안내하는 시작 문서다. |

## 4. 백엔드 실행·설정 파일

| 파일 | 역할 |
|---|---|
| `backend/.env.example` | 실행 모드, 로봇 namespace·역할, 타임아웃, 지도 경로와 ROS 토픽 환경변수 예시다. |
| `backend/requirements.txt` | Flask, Pillow, PyYAML 등 Python 의존성 버전을 정의한다. ROS 패키지는 시스템의 ROS 2 환경을 사용한다. |
| `backend/run_mock.sh` | 작업 디렉터리를 `backend/`로 맞추고 `MONITOR_MODE=mock`으로 Flask 앱을 실행한다. |
| `backend/run_ros.sh` | `rclpy` 사용 가능 여부를 확인한 후 `MONITOR_MODE=ros`로 실제 ROS 연동 앱을 실행한다. |
| `backend/run_replay.sh` | 저장된 쥐몰이 검증 궤적을 재생하도록 `MONITOR_MODE=replay`로 앱을 실행한다. |
| `backend/seed_dummy_history.py` | 기록 조회 화면을 시험할 Detection, 쥐구멍별 Trap 설치 상태, 이미지와 로봇 이동 경로 더미 데이터를 HistoryStore에 넣는다. |
| `backend/convert_rosbag_to_replay.py` | rosbag2 경로와 변환 옵션을 받아 Replay JSON을 생성하거나 기존 JSON에 시험을 추가하는 CLI 진입점이다. |

## 5. 백엔드 애플리케이션 패키지

### `backend/system_monitor/`

| 파일 | 역할 |
|---|---|
| `__init__.py` | `system_monitor`를 Python 패키지로 선언한다. |
| `app.py` | Flask 앱의 조립과 실행 진입점이다. 설정, 상태, 지도, 카메라, 기록 저장소와 현재 런타임을 생성하고 화면·상태·지도·카메라·이력 API를 제공한다. |
| `config.py` | 환경변수를 검증된 `Settings`, `RobotConfig`, `RosInterfaceConfig` 객체로 변환하고 로봇별 ROS 토픽 이름을 만든다. |
| `state_manager.py` | 로봇, 현재 위치, 역할, 표적, 실시간 탐지·덫·이벤트와 전체 임무 상태를 메모리 스냅샷으로 관리한다. Offline 판정과 최초 탐지 역할 배정도 수행한다. |
| `detection_service.py` | Mock과 ROS에서 들어온 탐지, 대상 유실, 저전압·복구, Trap 확인 이벤트를 공통 상태 변경 순서로 처리한다. |
| `risk_signals.py` | 여러 센서 라벨을 `LIVE_RODENT`, `ENTRY_POINT`, `DROPPINGS` 표준 위험신호로 정규화한다. |
| `camera_service.py` | 로봇별 최신 압축 카메라 프레임을 메모리에 보관하고 조회 URL 생성에 필요한 정보를 제공한다. |
| `map_service.py` | ROS 지도 YAML/PGM을 읽어 PNG와 지도 metadata를 만들고 map 좌표를 화면 픽셀 좌표로 변환한다. |
| `history_store.py` | SQLite의 `detections`, `trail_points` 테이블과 탐지 이미지 파일을 생성·저장·조회한다. 쥐구멍 탐지에는 Opening/Trap ID와 설치 상태를 선택적으로 보관한다. |
| `runtime_service.py` | Mock, ROS, Replay 구현이 따라야 하는 `available`, `running`, `status`, `start`, `stop` 공통 Protocol을 정의한다. |
| `mock_manager.py` | 장비 없이 로봇 이동, 탐지, 역할 배정, 배터리 경고와 Trap 확인 흐름을 재현하는 Mock 런타임이다. |
| `ros_bridge.py` | Fleet String, Detection3DArray, Odometry, BatteryState, CompressedImage를 공통 관제 상태로 변환한다. 탐지와 map 좌표 odom은 HistoryStore에도 주기 제한으로 저장한다. |
| `replay_manager.py` | 쥐몰이 검증 JSON의 Driver, Blocker, 표적 궤적과 FSM 상태를 실시간처럼 재생하고 기록 화면용 시험 데이터도 제공한다. |
| `rosbag_converter.py` | rosbag 메시지를 시간순 이벤트로 바꾸고 샘플링해 Replay trial/document를 만드는 변환 핵심 로직이다. |

## 6. 프론트엔드 파일

| 파일 | 역할 |
|---|---|
| `frontend/templates/dashboard.html` | 실시간 관제와 기록 조회 화면의 탭, 로봇·지도·카메라·이벤트·쥐몰이 재생 UI 구조를 정의한다. |
| `frontend/static/js/dashboard.js` | Flask API 폴링, 로봇·이벤트 렌더링, Canvas 지도, 기록 필터, 탐지 상세와 쥐몰이 재생 제어를 담당한다. |
| `frontend/static/css/dashboard.css` | 전체 대시보드의 색상, 카드, 지도, 대화상자, 기록·Replay 화면과 반응형 배치를 정의한다. |
| `frontend/static/favicon.svg` | 브라우저 탭에 표시되는 System Monitor 아이콘이다. |

## 7. 자동 테스트 파일

### `backend/tests/`

| 파일 | 검증 범위 |
|---|---|
| `test_app.py` | 공개 화면과 Flask API, 런타임 시작, Mock/ROS 응답 계약, 카메라·기록·쥐몰이 조회 및 잘못된 요청 처리를 검증한다. |
| `test_camera_service.py` | 로봇별 최신 프레임 저장·교체·조회와 이미지 URL 조건을 검증한다. |
| `test_history_store.py` | Detection·이미지·이동 경로와 쥐구멍 Trap 설치 상태의 저장·필터·스키마 이전·영속성을 검증한다. |
| `test_map_service.py` | 임시 YAML/PGM 지도 로드, PNG 변환, 90도 회전 좌표 변환과 지도 누락 fallback을 검증한다. |
| `test_mock_manager.py` | Mock 탐지 좌표, 표준 신호, 역할 배정, Trap 이벤트, 임무 완료와 상태 계산을 검증한다. |
| `test_replay_manager.py` | Replay 파일 로드, 위치·FSM 적용, 탐지·포획 이벤트, 역할별 문구, 시험 선택과 기록 응답을 검증한다. |
| `test_ros_bridge.py` | 토픽 생성, 좌표 frame 보존, Fleet/센서 메시지 변환, 카메라 캐시, 대상 유실, 저전압과 재접속 처리를 검증한다. |
| `test_rosbag_converter.py` | rosbag 메시지 해석, 불규칙 시간 샘플링, 필수 위치 검사, trial 추가와 ReplayManager 호환성을 검증한다. |
| `test_state_manager.py` | 상태 검증, 연결 요약, 최초 Rat 탐지 역할 배정, 세션 ID, Offline과 재연결 처리를 검증한다. |

테스트 실행 기준은 다음과 같다.

```bash
cd Sysmon/backend
python3 -m unittest discover -s tests -v
```

## 8. 문서 파일

### `docs/`

| 파일 | 역할 |
|---|---|
| `SYSTEM_MONITOR_FILE_STRUCTURE.md` | 현재 문서다. `Sysmon/` 전체 파일과 역할을 최신 상태로 정리한다. |
| `SYSTEM_MONITOR_SCOPE.md` | System Monitor가 맡는 기능, 맡지 않는 기능, 개발 우선순위와 공동 확정 항목을 정의한다. |
| `NOTION_SYSTEM_MONITOR_OVERVIEW.md` | 목적, 데이터 흐름, 초기 화면 구성과 모드 등 Notion 공유용 개요를 정리한다. |
| `MODE_PARITY.md` | Mock과 ROS 모드에서 같아야 하는 기능과 의도적으로 다른 기능을 정의한다. |
| `ROS_INTERFACE_SPEC.md` | Fleet 토픽, 선택 센서 토픽, 좌표계와 환경변수 등 ROS 연결 계약을 정리한다. |
| `STATUS_ENUM.md` | `OFFLINE`, `SEARCHING`, `TRACKING`, `RETURNING` 등 UI 상태값의 의미를 정의한다. |
| `UNDERGUARD_BUSINESS_REQUIREMENTS_V4_1.md` | 사업 요구사항을 요약하고 현재 System Monitor 구현과의 관계를 추적한다. |
| `HERDING_HISTORY_WORK_LOG.md` | 쥐몰이 기록 화면을 하위 탭부터 Replay 지도·제어·검증까지 단계별로 구현한 상세 작업 일지다. |
| `HERDING_DB_HANDOFF.md` | 쥐몰이 시험 목록을 SQLite와 파일 경로로 관리하기 위한 스키마·API·보안 원칙의 인수인계 문서다. |
| `ROSBAG_TO_REPLAY_GUIDE.md` | 실제 쥐몰이 rosbag2를 기록하고 Replay JSON으로 변환·추가·확인하는 절차와 오류 해결법이다. |
| `UI_DATA_DISPLAY_RECOMMENDATION.md` | Incident, Detection, Rat Track, Opening, Trap, 점검, Mission, 통계의 UI 표시 원칙과 우선순위를 제안한다. |
| `UI_DATA_IMPLEMENTATION_WORK_LOG.md` | 기존 화면과 겹치지 않는 데이터 확장 기능의 기획부터 구현·검증·완료까지 단계별로 기록한다. |

## 9. 데이터 및 실행 산출물

다음 파일은 애플리케이션 로직이 아니라 화면 재생, 기록 조회 또는 개발 확인에
사용되는 데이터다.

| 파일 | 역할 및 주의사항 |
|---|---|
| `backend/system_monitor/replay_data/real_map_frames.json` | 실제 쥐몰이 검증 결과의 지도 metadata, Trap, 여러 trial과 시간별 Driver·Blocker·표적 프레임을 담은 기본 Replay 데이터다. |
| `backend/data/history.db` | `HistoryStore`가 사용하는 SQLite 파일이다. 현재 개발용 기록이 들어 있으며 실행 중 변경될 수 있는 산출물이다. |
| `backend/data/captures/2.jpg` | Detection ID 2에 연결된 개발·더미 탐지 이미지다. |
| `backend/data/captures/5.jpg` | Detection ID 5에 연결된 개발·더미 탐지 이미지다. |
| `backend/data/captures/8.jpg` | Detection ID 8에 연결된 개발·더미 탐지 이미지다. |
| `backend/data/captures/10.jpg` | Detection ID 10에 연결된 개발·더미 탐지 이미지다. |
| `backend/data/captures/12.jpg` | Detection ID 12에 연결된 개발·더미 탐지 이미지다. |

`history.db`와 `captures/*.jpg`는 운영 원본으로 간주하면 안 된다. 배포 환경에서는
`HISTORY_DB_PATH`, `HISTORY_IMAGE_DIR`로 저장 위치를 정하고 백업·보존 정책을
별도로 적용해야 한다.

## 10. 파일 간 주요 연결 관계

| 입력 또는 기능 | 처리 경로 |
|---|---|
| Mock 상태 | `run_mock.sh` → `app.py` → `mock_manager.py` → `state_manager.py` |
| 실제 ROS 상태 | `run_ros.sh` → `app.py` → `ros_bridge.py` → `detection_service.py`/`state_manager.py` |
| Replay 상태 | `run_replay.sh` → `app.py` → `replay_manager.py` → `state_manager.py` |
| 실시간 카메라 | `ros_bridge.py` → `camera_service.py` → `app.py` 카메라 API → `dashboard.js` |
| 정적 지도 | 외부 YAML/PGM → `map_service.py` → `app.py` 지도 API → `dashboard.js` |
| 탐지 이력 | `history_store.py` ↔ `history.db`/`captures` → `app.py` 이력 API → `dashboard.js` |
| rosbag 변환 | `convert_rosbag_to_replay.py` → `rosbag_converter.py` → Replay JSON → `replay_manager.py` |
| 웹 화면 | `dashboard.html` + `dashboard.css` + `dashboard.js` |

## 11. 관리 시 주의사항

- UI는 SQLite 파일을 직접 읽지 않고 `app.py`의 API를 통해 조회한다.
- 실시간 `StateManager` 데이터와 영속 `HistoryStore` 데이터의 수명은 다르다.
- 카메라 바이트는 상태 스냅샷에 넣지 않고 전용 API로 전달한다.
- ROS 토픽이나 frame이 바뀌면 `.env.example`, `config.py`, `ros_bridge.py`,
  `ROS_INTERFACE_SPEC.md`를 함께 확인한다.
- 새 소스·테스트·문서·데이터 파일을 추가하면 이 파일 목록도 갱신한다.
- `__pycache__`, `.pytest_cache`, 가상환경처럼 자동 생성되는 캐시 디렉터리는 이
  문서의 파일별 목록에서 제외한다.
