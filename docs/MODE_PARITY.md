# Mock/ROS 모드 정합성

코드 리뷰에서는 “화면이 비슷한가”보다 “같은 입력이 같은 도메인 상태와 API 결과로
이어지는가”를 기준으로 설명합니다.

## 공통 처리 흐름

```text
Mock event 또는 ROS message
        ↓
표준 위험신호 변환
        ↓
process_detection
        ├─ SQLite 탐지 저장
        ├─ StateManager 상태/역할 갱신
        └─ 사건 타임라인 및 DB 저장
        ↓
/api/snapshot 공통 응답
```

표준 위험신호는 다음 세 값입니다.

- `LIVE_RODENT`: `rc_car`, `rat`, `mouse` 등 살아있는 설치류
- `ENTRY_POINT`: `rat_hole`, `hole` 등 진입 가능 지점
- `DROPPINGS`: `droppings`, `dropping` 등 배설물

기존 SQLite 데이터의 별칭은 서버 초기화 시 표준값으로 변환됩니다.

## 의도적으로 다른 부분

| 항목 | 차이를 유지한 이유 |
|---|---|
| ROS 명령 송신 | main `/fleet/command`가 지원하는 명령만 송신하고 PAUSE·INSTALL_TRAP은 비활성화 |
| ROS 임무 진행률 | Nav2 Feedback·Result가 없으므로 임의의 백분율을 만들지 않음 |
| 실제 영상 | 영상 전송 방식이 확정되지 않아 상태 메타데이터만 표시 |
| Mock 수동 사건 버튼 | 실제 ROS 데이터에 가짜 사건이 섞이지 않도록 Mock 모드에만 표시 |
| 운영 데이터 초기화 | 두 모드 모두 관리자만 실행. Mock은 임무를 대기시키고 ROS는 수집 노드·현재 로봇 상태를 유지 |

정적 `my_map.pgm/yaml` 배경과 좌표 변환 API는 두 모드가 공통으로 사용합니다.
Mock 위치는 처음부터 설정된 `ROS_MAP_FRAME` 좌표로 생성합니다. ROS의 `/odom`
위치는 TF 변환 전까지 `odom` 좌표이므로 실제 맵 위에는 억지로 표시하지 않고
`TF 미연동` 안내를 표시합니다.
탐지와 덫 위치도 설정된 지도 좌표계가 확인된 경우에만 지도 좌표로 저장합니다.

이 차이는 `/api/snapshot`의 `runtime` 필드로 전달됩니다. 두 구현 모두 동일한 필드를
반환하므로 Flask와 프런트엔드는 실행 모드별 조건문 대신 capability를 사용합니다.

## 공통 화면·API 계약

두 모드는 같은 `dashboard.html`과 `dashboard.js`를 사용하며 아래 기능의 DOM 및
응답 구조가 같습니다.

- 두 로봇 상태 카드와 카메라 카드
- 지도, 이동 경로, 탐지·덫 마커와 마커 상세
- 임무 제어 버튼, 확인 대화상자, 전체 이동 정지 버튼
- 관리자 전용 운영 데이터 초기화 버튼과 확인 대화상자
- `/api/snapshot`, `/api/map`, `/api/detections`, `/api/commands` 응답 형태

Mock 사건 생성 버튼만 실제 ROS 데이터에 시험 사건이 섞이지 않도록 숨깁니다.

## ROS에서 추가된 자동 상태 처리

- `/fleet/status`의 main 상태 enum을 관제 상태로 변환합니다.
- `/fleet/event`의 map 좌표 사건을 표준 위험신호와 트랩 기록으로 저장합니다.
- `/fleet/command`는 main enum으로 의미가 유지되는 명령만 발행합니다.
- 실제 ROS 메시지를 받기 전에는 로봇을 Online으로 표시하지 않습니다.
- OAK-D 살아있는 설치류 탐지가 설정 시간 동안 끊기면 `TARGET_LOST`로 전환합니다.
- 대상 유실 시 기존 `target` 좌표를 지우지 않아 마지막 위치를 유지합니다.
- BatteryState가 임계값 아래로 들어갈 때 저전압 사건을 한 번만 생성합니다.
- 배터리 회복 후 다시 임계값 아래로 내려가면 새 사건으로 기록합니다.

## 검증 명령

```bash
python3 -m unittest discover -s tests -v
```

실제 장비 검증 전에는 `ros2 topic list -t`로 namespace와 메시지 타입을 확인해야 합니다.

실행 명령은 기존 `MONITOR_MODE` 환경 변수 때문에 모드가 뒤바뀌지 않도록 각각
고정되어 있습니다.

```bash
./run_mock.sh

# ROS 2와 로봇 워크스페이스를 source한 터미널
./run_ros.sh
```
