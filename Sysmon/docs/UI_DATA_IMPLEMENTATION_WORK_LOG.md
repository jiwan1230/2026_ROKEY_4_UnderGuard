# UI 데이터 확장 구현 작업 기록

최초 작성일: 2026-08-11

현재 상태: DB 브랜치 병합 및 운영 DB 읽기 전용 조회 연결 완료, 기록 UI 전환 대기

## 1. 문서 목적

System Monitor의 기존 기능과 겹치지 않는 데이터 화면을 선정하고, 최초 검토부터
설계, 구현, 검증, 완료까지의 과정을 단계별로 기록한다.

이 문서는 계획서이면서 실제 작업 일지다. 각 단계는 작업이 실제로 끝난 뒤에만
`완료`로 변경하고, 변경한 파일, 확인 방법, 남은 문제를 함께 기록한다.

관련 문서:

- `UI_DATA_DISPLAY_RECOMMENDATION.md`: 전체 데이터 분류와 화면 구성 제안
- `SYSTEM_MONITOR_SCOPE.md`: 기존 System Monitor 책임 범위
- `STATUS_ENUM.md`: 현재 관제 상태 정의

## 2. 기록 원칙

- 실제 확인한 코드와 데이터만 `현재 구현`으로 기록한다.
- 계획과 완료 결과를 구분한다.
- 단계가 끝날 때 변경 파일과 검증 결과를 남긴다.
- 테스트하지 않은 항목은 `미검증`으로 표시한다.
- 운영 DB나 ROS 신호가 없어 Mock으로 확인했다면 이를 명시한다.
- 기존 사용자 변경사항과 이번 작업의 변경사항을 구분한다.
- 기능 범위가 바뀌면 기존 기록을 지우지 않고 결정 변경 이력을 추가한다.

## 3. 단계별 진행 현황

| 단계 | 내용 | 상태 |
|---|---|---|
| 0 | 최초 데이터 분류 | 완료 |
| 1 | 기존 System Monitor와 중복 검토 | 완료 |
| 2 | 구현 대상과 우선순위 선정 | 완료 |
| 3 | 1차 구현 범위와 화면 명세 확정 | 완료 |
| 4 | 데이터/API 계약 설계 | 완료(로컬 계약) |
| 5 | 백엔드 집계·조회 구현 | 완료(운영 DB 연동 제외) |
| 6 | 프론트엔드 화면 구현 | 완료 |
| 7 | 단위·통합·화면 검증 | 완료(실물 연동 제외) |
| 8 | 문서 정리와 완료 판정 | 대기 |
| 9 | 기존 탐지·이동 기록 저장 배선과 UI 단순화 | 완료(실물 연동 제외) |
| 10 | DB 브랜치 병합과 System Monitor 조회 계약 결합 | 완료(실제 MySQL 검증 제외) |

## 4. 단계별 작업 기록

### 단계 0. 최초 데이터 분류

상태: 완료

기록일: 2026-08-11

#### 검토 목적

운영 DB에 저장되는 데이터를 기준으로 UI의 전체 정보 범위를 정한다.

#### 검토 결과

다음 8개 대분류를 도출했다.

1. Incident
2. Detection
3. Rat Tracking
4. Opening
5. Trap
6. Trap Inspection
7. Robot Mission
8. Statistics

실시간 관제에는 현재 Incident, Rat 이동경로, Opening/Trap 위치와 상태, Robot
현재 상태를 우선 표시하고, 나머지는 이력·분석 영역으로 분리하는 방향을
검토했다.

#### 산출물

- `Sysmon/docs/UI_DATA_DISPLAY_RECOMMENDATION.md`

### 단계 1. 기존 System Monitor와 중복 검토

상태: 완료

기록일: 2026-08-11

#### 확인한 기존 UI 기능

- 로봇 연결, 배터리, 현재 상태
- 실시간 로봇 위치와 이동 경로
- 실시간 탐지 마커
- 로봇 카메라
- 최근 세션 이벤트
- Detection 이력과 이미지
- 로봇 이동 이력
- 쥐몰이 Replay

#### 확인한 현재 영속 데이터

| 구현 위치 | 테이블 | 용도 |
|---|---|---|
| `Sysmon/backend/system_monitor/history_store.py` | `detections` | 탐지 시간, 로봇, 종류, 좌표, confidence, 이미지 |
| `Sysmon/backend/system_monitor/history_store.py` | `trail_points` | 로봇별 시간순 이동 좌표 |
| `src/turtle_project/turtle_project/db_node.py` | `holes` | 구멍 좌표와 Trap 설치 여부 |

전달받은 7개 운영 테이블과 현재 체크아웃에서 확인되는 테이블 사이에 차이가
있다. Incident, Rat Track, Opening, Trap, Trap Inspection, Robot Mission 화면은
실제 운영 DB 또는 조회 API가 연결되기 전에는 완성 데이터로 구현할 수 없다.

#### 중복으로 판단한 항목

- Robot 현재 상태를 별도 신규 화면으로 다시 구현
- 기존과 같은 일반 Detection 목록
- 기존과 같은 로봇 이동경로 지도

### 단계 2. 구현 대상과 우선순위 선정

상태: 완료

기록일: 2026-08-11

#### 선정 결과

1. 탐지 분석 대시보드
2. Opening·Trap 관리 화면
3. Incident History
4. Trap Inspection 및 Robot Mission 성능 확장

#### 선정 이유

##### 1순위: 탐지 분석 대시보드

현재 `detections` 데이터로 구현할 수 있고 기존의 개별 기록 조회와 역할이
겹치지 않는다.

초기 후보 지표:

- 시간대별 Rat 탐지 횟수
- 객체 종류별 탐지 건수
- Robot별 탐지 건수
- 최근 24시간과 7일 비교
- Rat Detection 위치 Heatmap
- 최다 출몰 시간대

##### 2순위: Opening·Trap 관리

실시간 마커가 아니라 지속 관리 대상의 현재 상태와 점검 필요 여부를 보여준다.
현재 `holes` 테이블만 이용하면 Opening 위치와 Trap 설치 여부 정도로 시작할 수
있다. 조회 API와 데이터 소유권 합의가 선행되어야 한다.

##### 3순위: Incident History

개별 이벤트를 발견부터 `CAPTURED` 또는 `LOST`까지 하나의 사건으로 묶는다.
운영 가치가 크지만 현재 저장소에 Incident와 Rat Track 원본 데이터가 없어 DB/API
연결 이후 구현한다.

##### 후속 범위

Trap Inspection과 Robot Mission 성능은 실제 영속 데이터가 제공된 뒤 추가한다.
Dropping 통계와 신호가 없는 Trap `MISSING`/`CAPTURED` 상태는 현재 범위에서
보류한다.

#### 화면 구조 결정

기존 화면과 신규 화면의 상위 구성을 다음과 같이 검토했다.

```text
실시간 관제
기록 조회
분석
시설·트랩 관리
```

최종 메뉴명과 화면 이동 방식은 단계 3에서 현재 반응형 레이아웃을 기준으로
확정한다.

### 단계 3. 1차 구현 범위와 화면 명세 확정

상태: 완료

작업일: 2026-08-11

#### 우선순위 변경

최초에는 탐지 분석을 먼저 구현할 예정이었으나, 사용자가 쥐구멍 탐지 기록에서
Trap 설치 여부를 먼저 확인할 필요가 있다고 결정했다. 탐지 분석 계획은 삭제하지
않고 후속 작업으로 이동했다.

#### 확정한 1차 범위

- 기존 `기록 조회 → 탐지·이동 기록` 화면을 확장한다.
- `ENTRY_POINT` 기록에만 Trap 설치 상태를 표시한다.
- 상태는 `INSTALLED`, `NOT_INSTALLED`, `UNKNOWN` 세 가지로 제한한다.
- 목록 배지, 선택 상세, 기록 지도 보조 기호와 전용 필터를 제공한다.
- 실제 운영 MySQL 연결 전에는 현재 HistoryStore와 더미 데이터로 UI 계약을
  검증한다.
- `ACTIVE/MOVED` 상세 상태, 점검 거리와 최근 점검시간은 다음 연동 단계로 둔다.

#### 화면 표현

| API 상태 | 화면 문구 | 색상 의미 |
|---|---|---|
| `INSTALLED` | 트랩 설치 확인 | 초록 |
| `NOT_INSTALLED` | 트랩 미설치 | 회색 |
| `UNKNOWN` 또는 누락 | 트랩 확인 필요 | 노랑 |

### 단계 4. 데이터/API 계약 설계

상태: 완료(로컬 계약)

작업일: 2026-08-11

#### DB 브랜치 확인 결과

DB 브랜치를 병합하지 않고 `origin/DB`를 조회했다.

- MySQL `opening`과 `trap` 테이블이 존재한다.
- `trap.status`는 `ACTIVE`, `MOVED`, `MISSING`, `CAPTURED`를 준비하고 있지만
  현재 실제 신호는 `ACTIVE`, `MOVED`만 사용한다.
- `/db/query_hole`은 좌표 기준으로 `trap_installed` bool을 반환한다.
- 전체 Opening/Trap 상태를 System Monitor에 제공하는 조회 API는 아직 없다.
- `QueryHole`의 bool은 `ACTIVE`만 true이므로 `MOVED`와 미설치를 구분할 수 없다.
- 운영 DB의 객체명 `OPENING`은 System Monitor 표준값 `ENTRY_POINT`로 변환해야
  한다.

#### 1차 API 응답 계약

기존 `GET /api/history/detections` 응답에 아래 필드를 추가했다.

```json
{
  "opening_id": "O001",
  "trap_id": "T001",
  "trap_installation_status": "INSTALLED"
}
```

쥐나 배설물 기록에는 세 필드가 `null`이고, 과거 쥐구멍 기록처럼 값이 없으면
API가 `UNKNOWN`으로 반환한다. 다음 필터도 추가했다.

```text
GET /api/history/detections?trap_installation_status=INSTALLED
```

### 단계 5. 백엔드 저장·조회 구현

상태: 완료(운영 DB 연동 제외)

작업일: 2026-08-11

#### 구현 내용

- 기존 SQLite `detections`에 `opening_id`, `trap_id`,
  `trap_installation_status` 필드를 추가했다.
- 기존 DB는 삭제하지 않고 `PRAGMA table_info` 확인 후 `ALTER TABLE`로 이전한다.
- `ENTRY_POINT` 신규 기록은 상태를 생략하면 `UNKNOWN`으로 저장한다.
- 허용하지 않는 상태는 저장과 API 요청에서 거부한다.
- Trap 상태 필터는 쥐구멍 기록만 반환한다.
- 더미 생성기는 쥐구멍에 세 상태와 Opening/Trap ID 예시를 넣는다.

#### 남은 제약

현재 구현은 화면과 API 계약을 먼저 검증하기 위한 로컬 HistoryStore 기준이다.
운영 MySQL의 `opening`/`trap` 전체 조회 API가 제공되면 해당 데이터를 이 응답으로
매핑해야 한다. System Monitor가 MySQL 파일이나 계정으로 직접 접속하지 않는다.

### 단계 6. 프론트엔드 화면 구현

상태: 완료

작업일: 2026-08-11

#### 구현 내용

- `쥐구멍 트랩` 필터를 추가했다.
- Trap 필터를 선택하면 객체 종류를 자동으로 쥐구멍으로 맞춘다.
- 다른 객체 종류를 선택하면 Trap 필터를 해제한다.
- 쥐구멍 기록 카드에 상태 배지를 표시한다.
- 선택 상세에 Opening ID, Trap 설치 여부와 Trap ID를 표시한다.
- 기록 지도 쥐구멍 마커 옆에 `✓`, `–`, `?` 보조 기호를 표시한다.
- 색상만으로 상태를 판단하지 않도록 목록과 상세에는 텍스트를 함께 표시한다.

#### 변경 파일

- `frontend/templates/dashboard.html`
- `frontend/static/js/dashboard.js`
- `frontend/static/css/dashboard.css`

### 단계 7. 단위·통합·화면 검증

상태: 완료(실물·운영 DB 연동 제외)

작업일: 2026-08-11

#### 자동 검증

- Python 전체 단위 테스트 76개 통과
- 기존 SQLite 스키마 자동 이전과 데이터 보존 테스트 통과
- 기본 `UNKNOWN`, 정상 상태, 잘못된 상태 거부와 필터 테스트 통과
- Flask API 및 기존 Mock/ROS/Replay 회귀 테스트 통과
- Python 문법 검사 통과

#### 브라우저 확인

- 임시 DB와 Flask 서버로 실제 API 응답을 확인했다.
- Chrome Headless가 HTML, CSS, JavaScript와 `/api/snapshot`을 정상 요청했다.
- `node`가 설치돼 있지 않아 `node --check`는 실행하지 못했다.
- 실제 MySQL과 실물 Robot의 end-to-end 검증은 DB 조회 계약 완료 후 진행한다.

### 단계 8. 문서 정리와 완료 판정

상태: 대기

#### 예정 작업

- 구현 결과와 최초 계획 차이 정리
- 실행·사용 방법 문서 반영
- API 문서와 상태 정의 갱신
- 후속 Opening·Trap 작업 범위 갱신
- 최종 변경 파일과 테스트 결과 정리

#### 최종 완료 조건

- 운영자가 쥐구멍 탐지별 Trap 설치 여부를 확인할 수 있다.
- 운영 MySQL의 Opening/Trap 상태가 조회 API를 통해 연결돼 있다.
- 빈 데이터와 오류 상태가 정상적으로 표시된다.
- 자동 테스트와 주요 화면 검증이 완료됐다.
- 실제 `ACTIVE/MOVED` 상태와 설치 여부 변환 규칙이 DB 담당자와 확정돼 있다.

### 단계 9. 기존 탐지·이동 기록 저장 배선과 UI 단순화

상태: 완료(실물 연동 제외)

작업일: 2026-08-11

#### 시작 전 확인

- `HistoryStore.record_detection()`과 `record_trail_point()`는 있었지만
  `RosBridge`에 저장소가 전달되지 않아 실제 ROS 수신 데이터가 DB에 쌓이지 않았다.
- 기록 Summary는 필터와 무관한 전체 건수를 반환했다.
- 시간대별 점과 탐지 목록이 같은 선택 동작을 중복 제공했다.
- 목록과 상세가 좌표·신뢰도까지 반복 표시했다.
- README와 책임 범위 문서가 로컬 이력 DB가 없다고 설명해 실제 코드와 달랐다.

#### 백엔드 변경

- `app.py`가 생성한 같은 `HistoryStore`를 `RosBridge`에 주입했다.
- Detection3D와 `/fleet/event` 탐지를 실시간 상태 처리 후 SQLite에도 저장한다.
- 탐지 시점의 최신 카메라 프레임이 있으면 증거 이미지 파일도 함께 저장한다.
- 동일 Robot·객체 탐지는 기본 1초 간격으로 제한한다.
- odom은 좌표 frame이 `map`일 때만 기본 0.5초 간격으로 이동 경로에 저장한다.
- 기록 실패는 ROS 실시간 관제를 중단하지 않고 로그만 남긴다.
- `/api/history/summary`에 기간·종류·Robot·Trap 필터를 적용했다.

#### UI 변경

- 중복된 `시간대별 기록` 타임라인을 제거했다.
- 화면 순서를 `필터 → 지도·탐지 목록 → 선택한 탐지 증거`로 단순화했다.
- 탐지 목록에서는 좌표와 confidence를 제거하고 시간·Robot만 요약한다.
- 좌표, confidence와 Opening/Trap 정보는 선택 상세에 유지한다.
- 상세 패널을 큰 증거 이미지 중심으로 확장했다.
- 상단의 경로 row 개수를 제거하고 현재 필터의 탐지 건수만 표시한다.
- Robot 필터 후보를 이동 경로뿐 아니라 탐지 기록에서도 수집한다.

#### 문서 변경

- README의 데이터 소유·영속 기록·ROS 입력 설명을 현재 구현에 맞췄다.
- `SYSTEM_MONITOR_SCOPE.md`에 로컬 탐지·이동 기록 책임을 반영했다.
- `SYSTEM_MONITOR_FILE_STRUCTURE.md`의 RosBridge 역할을 갱신했다.

#### 검증 결과

- Python 전체 단위 테스트 77개 통과
- ROS 탐지·Fleet 탐지·map odom 저장 테스트 통과
- map이 아닌 odom 좌표의 저장 제외 테스트 통과
- 고주파 탐지·이동 기록 제한 테스트 통과
- 필터 Summary 테스트와 기존 기능 회귀 테스트 통과
- 실물 TurtleBot과 실제 map 위치 토픽 검증은 남아 있다.

### 단계 10. DB 브랜치 병합과 System Monitor 조회 계약 결합

상태: 완료(실제 MySQL 검증 제외)

작업일: 2026-08-11

#### 병합 기준

- `src/`의 MySQL 스키마, DB 노드, ROS 메시지·서비스와 로봇 코드는
  `origin/DB` 최신 내용을 기준으로 반영했다.
- `Sysmon/` 충돌은 현재 HistoryStore, Replay, 기록 UI를 유지하면서 DB 브랜치의
  읽기 전용 조회 기능을 결합했다.

#### 결합 결과

- `/fleet/detection`의 실제 `robot_id`와 `confidence`를 `/fleet/event` 탐지에
  보강한다. 보강 메시지가 없거나 5초 이상 오래되면 기존 로봇 추정으로 되돌아간다.
- System Monitor는 MySQL에 직접 접속하지 않고 `DbQuery` ROS Service를 호출한다.
- `/api/db/detections`, `/api/db/missions`, `/api/db/traps`, `/api/db/report`만
  읽기 전용으로 노출한다.
- DB 조회 장애는 해당 API만 실패하게 하고 실시간 관제와 로컬 HistoryStore 기록은
  계속 동작하도록 분리했다.
- 기존 쥐몰이 Replay 이력 API와 ROS 탐지·map 이동 경로 저장을 모두 보존했다.

#### 검증 결과

- System Monitor Python 전체 단위 테스트 85개 통과
- DB 조회 허용 목록, 쓰기 차단, DB 장애 격리 테스트 통과
- `/fleet/detection` 로봇·confidence 보강과 fallback 테스트 통과
- DB·로봇 Python 소스 문법 컴파일 검사 통과
- 실제 ROS 2 Service와 MySQL을 함께 실행한 end-to-end 검증은 남아 있다.

## 5. 결정 이력

### 2026-08-11: 전체 8개 분류를 한 번에 구현하지 않음

현재 System Monitor와 겹치는 기능이 있고 운영 테이블 전체가 이 체크아웃에서
확인되지 않으므로 단계적으로 구현하기로 했다.

### 2026-08-11: 탐지 분석을 첫 구현 후보로 선정

현재 영속 저장되는 `detections`를 사용할 수 있고, 기존 개별 기록 화면에 없는
집계 가치를 제공하기 때문이다.

### 2026-08-11: Opening·Trap은 자산 관리 화면으로 분리

실시간 탐지 마커와 목적이 다르므로 위치, 설치 여부, 마지막 확인, 점검 필요성을
보여주는 관리 화면으로 설계한다. 현재 데이터가 제한적이므로 API와 스키마를
확인한 뒤 구현한다.

### 2026-08-11: 첫 구현을 Trap 설치 여부로 변경

사용자 결정에 따라 탐지 분석보다 쥐구멍별 Trap 설치 여부를 먼저 구현했다.
기존 탐지 기록 화면을 재사용해 중복 화면을 만들지 않고 목록·상세·지도·필터를
함께 확장했다.

### 2026-08-11: DB 브랜치를 병합하지 않고 계약만 확인

`origin/DB`의 MySQL 스키마와 `db_node.py`를 읽어 `opening`, `trap`,
`trap_inspection` 구조를 확인했다. 현재 조회 인터페이스만으로는 `MOVED`와
미설치를 구분할 수 있어, 운영 연동 전에 DB 담당자의 전체 조회 API가 필요하다.

### 2026-08-11: 기존 기록 화면의 데이터 완결성을 UI보다 우선

새 분석 화면을 추가하기 전에 실제 ROS 입력이 HistoryStore까지 도달하도록
배선하고, 중복 타임라인과 불일치 Summary를 정리했다. 이 변경은 운영 DB 확장과
별개로 현재 탐지·이동 기록 기능 자체를 완성하기 위한 작업이다.

### 2026-08-11: 최신 DB 브랜치를 현재 관제 구현과 병합

DB 브랜치에 `DbQuery` 조회 계약이 추가되어 이전의 계약 확인 단계에서 실제 병합
단계로 진행했다. DB·Robot 소스는 DB 브랜치를 기준으로 하고, System Monitor는
기존 HistoryStore·Replay·기록 UI를 유지한 채 읽기 전용 운영 DB 조회만 결합했다.

## 6. 작업 변경 목록

### 2026-08-11

- 추가: `Sysmon/docs/UI_DATA_DISPLAY_RECOMMENDATION.md`
- 추가: `Sysmon/docs/UI_DATA_IMPLEMENTATION_WORK_LOG.md`
- 코드 변경: 없음
- 테스트: 문서만 추가하여 실행하지 않음

### 2026-08-11 — Trap 설치 여부 1차 구현

- 수정: `backend/system_monitor/history_store.py`
- 수정: `backend/system_monitor/app.py`
- 수정: `backend/seed_dummy_history.py`
- 수정: `backend/tests/test_history_store.py`
- 수정: `backend/tests/test_app.py`
- 수정: `frontend/templates/dashboard.html`
- 수정: `frontend/static/js/dashboard.js`
- 수정: `frontend/static/css/dashboard.css`
- 수정: `docs/SYSTEM_MONITOR_FILE_STRUCTURE.md`
- 테스트: Python 전체 단위 테스트 76개 통과
- 브라우저: Chrome Headless에서 정적 자원과 API 요청 확인
- 미검증: 실제 MySQL Opening/Trap 조회와 실물 Robot 연동

### 2026-08-11 — 기존 탐지·이동 기록 개선

- 수정: `backend/system_monitor/ros_bridge.py`
- 수정: `backend/system_monitor/app.py`
- 수정: `backend/system_monitor/history_store.py`
- 수정: `backend/tests/test_ros_bridge.py`
- 수정: `backend/tests/test_history_store.py`
- 수정: `backend/tests/test_app.py`
- 수정: `frontend/templates/dashboard.html`
- 수정: `frontend/static/js/dashboard.js`
- 수정: `frontend/static/css/dashboard.css`
- 수정: `README.md`
- 수정: `docs/SYSTEM_MONITOR_SCOPE.md`
- 수정: `docs/SYSTEM_MONITOR_FILE_STRUCTURE.md`
- 테스트: Python 전체 단위 테스트 77개 통과
- 미검증: 실물 Robot Detection/odom과 실제 지도 좌표계 end-to-end

### 2026-08-11 — DB 브랜치 병합

- 병합: `origin/DB` 최신 `afb499e`
- 추가: `turtle_interfaces/msg/DetectionEvent`, `srv/DbQuery` 등 DB 계약
- 추가: `backend/tests/test_db_query.py`
- 수정: `backend/system_monitor/app.py`, `config.py`, `ros_bridge.py`
- 수정: `docs/ROS_INTERFACE_SPEC.md`, `SYSTEM_MONITOR_FILE_STRUCTURE.md`
- 테스트: System Monitor Python 전체 단위 테스트 85개 통과
- 문법 검사: DB·Robot Python 소스 전체 통과
- 미검증: 실제 ROS 2 `/db/query`와 MySQL end-to-end

## 7. 다음 작업

다음 단계는 **운영 DB 조회 결과를 현재 기록 UI에 선택적으로 연결하는 것**이다.

현재 `/api/db/detections`와 `/api/db/traps`까지 백엔드 계약은 연결됐다. 다음에는
DB의 `OPENING`을 UI의 `ENTRY_POINT`로 정규화하고, `trap_installed`와 실제
`trap.status`를 구분해 목록·상세·지도에 반영해야 한다. 그 뒤 실제 ROS 2 Service와
MySQL 데이터로 재시작 복원과 장애 fallback을 검증하면 단계 8을 완료한다.
