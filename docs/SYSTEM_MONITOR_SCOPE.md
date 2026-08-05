# System Monitor 책임 범위

## PM 소유
- 전체 비즈니스/System Requirement
- 두 로봇의 최종 역할과 협업 시나리오
- 공통 namespace 및 통합 일정
- 오류 발생 시 실제 로봇의 복구 정책
- Acceptance Test 성공 기준

## System Monitor 담당 소유
- Flask 서버와 웹 UI 구조
- ROS 메시지 → 관제 상태 변환
- 로봇·탐지·오류 상태 모델
- 최초 쥐 탐지 기반 추적·탐색/트랩 역할 자동 배정 상태와 이력
- SQLite3 스키마와 탐지 검색
- 로그인과 접근 제어
- Mock 시나리오
- System Monitor Unit Test

## 공동 확정 필요
- Topic / Service / Action 명칭과 메시지 필드
- 상태 enum
- Offline 판단 시간
- 대상 유실·Nav2 실패·배터리 부족 시 실제 대응
