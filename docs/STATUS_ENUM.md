# 관제 상태 정의

| 상태 | 의미 |
|---|---|
| `OFFLINE` | 제한 시간 동안 로봇 데이터가 없음 |
| `IDLE` | 연결됐지만 임무 대기 |
| `SEARCHING` | 쥐구멍·배설물 탐색 |
| `APPROACHING` | 웹캠 좌표 기반 대상 접근 |
| `TRACKING` | OAK-D 기반 대상 추적 |
| `TARGET_LOST` | OAK-D가 대상 유실 |
| `NAVIGATING` | 일반 Nav2 이동 |
| `INSTALLING_TRAP` | 덫 설치 작업 |
| `RETURNING` | 시작점 또는 Dock 복귀 |
| `PAUSED` | 운영자 또는 시스템에 의해 일시정지 |
| `COMPLETED` | 현재 임무 완료 |
| `ERROR` | 복구 판단이 필요한 오류 |

기존 `goal_manager_node` 로그를 바로 UI 상태로 쓰지 않고, ROS Bridge에서 위 상태로 변환한다.
