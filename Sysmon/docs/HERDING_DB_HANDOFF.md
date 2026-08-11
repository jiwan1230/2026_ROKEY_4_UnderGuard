# 쥐몰이 시험 기록 DB 작업 인수인계

이 문서는 쥐몰이 기록 화면의 6단계를 나중에 DB 담당자가 이어서 구현할 수 있도록
현재 구조, 권장 저장 범위, 테이블 초안과 완료 기준을 정리한다.

상태: 구현 보류 · DB 담당자 검토 대기

작성일: 2026-08-10

## 1. 6단계의 목적

현재 쥐몰이 기록 화면은 하나의 Replay JSON 안에 들어 있는 여러 시험을 선택할 수
있다. 시험 파일이 많아지면 사용자가 원하는 JSON을 환경변수로 직접 지정하는
방식만으로는 날짜·모델·성공 여부를 검색하기 어렵다.

6단계에서는 SQLite에 시험의 작은 요약과 원본 파일 위치만 등록한다.

```text
SQLite
└── 시험 이름·날짜·모델·성공 여부·파일 경로

파일 시스템
├── rosbag2 디렉터리: 원본 ROS 메시지
├── Replay JSON: 지도 경로 재생 프레임
└── 영상 파일: 필요한 경우 별도 보관
```

좌표 프레임 전체, 지도 이미지, 카메라 영상이나 rosbag 바이트는 DB에 직접 넣지
않는다. 큰 데이터를 SQLite BLOB으로 넣으면 DB 백업과 조회가 무거워지고 원본
도구인 `ros2 bag`으로 확인하기도 어려워진다.

## 2. 현재 구현되어 있는 기능

### 기존 SQLite 저장소

`backend/system_monitor/history_store.py`의 `HistoryStore`가 다음 두 테이블을
관리한다.

- `detections`: 탐지 시각, 로봇, 종류, 좌표와 증거 이미지 경로
- `trail_points`: 로봇별 시각과 이동 좌표

기본 DB 경로는 `backend/data/history.db`이며 `HISTORY_DB_PATH` 환경변수로 바꿀
수 있다. SQLite 연결은 `check_same_thread=False`와 `RLock`으로 보호된다.

### 쥐몰이 기록 API

현재 `GET /api/history/herding?trial_index=N`은 실행 설정이 가리키는 Replay JSON
한 파일을 `ReplayManager`에서 읽는다. 응답에는 다음 값이 있다.

- `trial_options`: 파일 안에 있는 시험 요약 목록
- `selected_trial_index`, `trial_count`
- 선택된 시험의 전체 `trial.frames`
- 지도 이미지, 좌표 범위, 포획 지점과 알고리즘 파라미터

기록 선택은 읽기 전용이며 실제 Replay 모드의 실행 시험을 바꾸지 않는다.

### rosbag 변환 도구

`backend/convert_rosbag_to_replay.py`가 rosbag2를 Replay JSON으로 변환한다. 하나의
JSON에 여러 시험을 추가할 수 있으므로 DB에서는 파일 경로뿐 아니라 파일 안의
`trial_index`도 함께 저장해야 한다.

## 3. DB에 저장할 권장 범위

### 반드시 필요한 값

| 필드 | 의미 | 출처 |
|---|---|---|
| `id` | DB 내부 시험 식별자 | DB 자동 생성 |
| `recorded_at` | 실제 시험 시작 시각 | rosbag metadata 또는 등록 입력 |
| `name` | 화면에 표시할 시험 이름 | 등록 입력 |
| `model` | 알고리즘 모델 이름 | Replay JSON `trial.model` |
| `success` | 포획 성공 여부 | Replay JSON `trial.success` |
| `duration` | 시험 수행 시간(초) | Replay JSON `trial.duration` |
| `goal_name` | 포획 지점 | Replay JSON `trial.goal_name` |
| `driver_id` | Driver 로봇 이름 | Replay JSON/API 설정 |
| `blocker_id` | Blocker 로봇 이름 | Replay JSON/API 설정 |
| `map_frame` | 좌표계 이름 | 보통 `map` |
| `frame_count` | Replay 프레임 수 | `len(trial.frames)` |
| `rosbag_path` | 원본 rosbag2 디렉터리 경로 | 변환 metadata 또는 등록 입력 |
| `replay_json_path` | 재생용 JSON 경로 | 등록 입력 |
| `replay_trial_index` | JSON 내부 시험 번호 | JSON 배열 위치 |
| `created_at` | DB 등록 시각 | DB 등록 시 생성 |

### 있으면 유용한 값

- `algorithm_version`: Git commit, 태그 또는 알고리즘 버전
- `seed`: 시뮬레이션/알고리즘 랜덤 시드
- `notes`: 현장 조건과 특이사항
- `source_hash`: Replay JSON 또는 rosbag 식별용 해시
- `site_name`: 시험 장소
- `operator_name`: 시험 담당자

개인정보나 운영 규칙상 저장하면 안 되는 값이 있는지는 구현 전에 팀에서 확인해야
한다.

## 4. 테이블 초안

아래 SQL은 설계 검토용 초안이다. DB 담당자가 기존 명명 규칙과 마이그레이션
방식에 맞게 조정한 뒤 적용한다.

```sql
CREATE TABLE herding_trials (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    recorded_at REAL NOT NULL,
    name TEXT NOT NULL,
    model TEXT,
    algorithm_version TEXT,
    success INTEGER NOT NULL DEFAULT 0,
    duration REAL,
    goal_name TEXT,
    driver_id TEXT,
    blocker_id TEXT,
    map_frame TEXT NOT NULL DEFAULT 'map',
    frame_count INTEGER NOT NULL DEFAULT 0,
    rosbag_path TEXT,
    replay_json_path TEXT NOT NULL,
    replay_trial_index INTEGER NOT NULL DEFAULT 0,
    seed INTEGER,
    site_name TEXT,
    operator_name TEXT,
    notes TEXT,
    source_hash TEXT,
    created_at REAL NOT NULL,
    UNIQUE(replay_json_path, replay_trial_index)
);

CREATE INDEX idx_herding_trials_recorded_at
    ON herding_trials(recorded_at DESC);

CREATE INDEX idx_herding_trials_model_success
    ON herding_trials(model, success);
```

JSON 한 파일에 시험이 네 개라면 같은 `replay_json_path`로 네 행을 만들고
`replay_trial_index`를 0, 1, 2, 3으로 저장한다.

## 5. 파일 경로 처리 원칙

- DB에는 파일 내용이 아니라 경로만 저장한다.
- 가능하면 설정된 기록 루트 아래의 상대경로로 저장해 PC가 바뀌어도 옮기기
  쉽게 한다.
- API에서 경로 문자열을 그대로 클라이언트에 노출하지 않는다.
- 서버가 경로를 열 때 `Path.resolve()` 후 허용된 기록 루트 내부인지 검사한다.
- `../`를 이용해 기록 루트 밖의 파일을 읽을 수 없게 한다.
- 파일이 없어져도 DB 행을 자동 삭제하지 않고 `MISSING` 상태로 표시한다.
- 원본 rosbag과 Replay JSON 삭제는 관제 웹 화면에서 제공하지 않는다.

권장 환경변수 예시는 다음과 같다.

```text
HERDING_RECORD_ROOT=/data/underguard/herding
```

## 6. 권장 등록 방식

현재 대시보드는 기록을 조회만 하고 수정·삭제하지 않는 원칙을 사용한다. 따라서
초기 버전에서는 웹 POST API보다 별도 등록 명령을 권장한다.

```text
rosbag 기록
    ↓ convert_rosbag_to_replay.py
Replay JSON 생성
    ↓ register_herding_trial.py (6단계에서 구현)
SQLite에 시험 요약·경로 등록
```

등록 명령은 다음 동작이 필요하다.

1. Replay JSON을 읽고 모든 trial의 필수 필드를 검사한다.
2. JSON 경로와 trial index의 중복을 확인한다.
3. rosbag 경로가 있다면 `metadata.yaml` 존재 여부를 확인한다.
4. 하나의 트랜잭션으로 시험 행을 등록한다.
5. 등록 결과의 DB id와 경고를 터미널에 출력한다.

나중에 운영상 웹 등록이 꼭 필요해지면 인증·권한·감사 로그 범위를 먼저 정한 뒤
POST API를 추가한다.

## 7. 조회 API 초안

### 시험 목록

```text
GET /api/history/herding/trials
GET /api/history/herding/trials?model=reactive_flee&success=true
GET /api/history/herding/trials?since=...&until=...&goal_name=top
```

목록 응답에는 프레임 전체나 `map_image`를 넣지 않고 DB의 작은 요약만 반환한다.
기본 정렬은 `recorded_at DESC`, 기본 개수 제한은 100건 정도가 적당하다.

### 시험 한 건 불러오기

```text
GET /api/history/herding/trials/<id>
```

서버는 DB의 `replay_json_path`와 `replay_trial_index`를 이용해 JSON을 읽고, 현재
`/api/history/herding`과 같은 Replay 응답 구조로 반환한다. 기존 화면 코드를
가능한 한 재사용하기 위해 응답 필드명을 유지하는 것이 좋다.

기존 `GET /api/history/herding?trial_index=N`은 번들 예제와 Replay 모드 확인용으로
남겨도 된다.

## 8. 화면 작업 범위

DB 담당자가 목록 API를 제공하면 쥐몰이 기록 화면에 다음 필터를 연결할 수 있다.

- 시험 날짜 범위
- 알고리즘 모델·버전
- 성공/실패
- 포획 지점
- Driver·Blocker
- 파일 존재 여부

사용자가 목록에서 시험을 고르면 현재 3단계 재생 컨트롤과 지도에 선택한 시험을
그대로 연결한다. 위치 프레임 자체를 DB 검색 결과에 포함하지 않아 목록 조회가
무거워지지 않도록 한다.

## 9. 기존 HistoryStore와 통합할 때 주의할 점

- 기존 `detections`, `trail_points` 테이블과 이름이 겹치지 않게 한다.
- 기존 DB 파일을 지우거나 새로 만들지 않고 `CREATE TABLE IF NOT EXISTS` 또는
  명시적인 migration으로 추가한다.
- 운영 DB에 적용하기 전에 파일을 복사해 이전 버전 앱에서도 열리는지 확인한다.
- `HistoryStore`의 `RLock` 안에서 같은 SQLite 연결을 사용한다.
- 등록 중 오류가 나면 반드시 rollback해 일부 trial만 남지 않게 한다.
- SQLite boolean은 기존 방식처럼 0/1로 저장하고 API에서 bool로 변환한다.
- DB 시간은 epoch 초 또는 UTC ISO 8601 중 하나로 통일한다. 기존 테이블은 epoch
  초를 사용하므로 일관성을 위해 `REAL` epoch 초를 권장한다.

## 10. 담당 범위 제안

| 담당 | 작업 |
|---|---|
| DB 담당 | 스키마·마이그레이션, 등록 명령, 목록/상세 조회, 필터와 인덱스 |
| Sysmon 화면 담당 | 검색 필터, 시험 목록, 기존 재생 화면 연결 |
| 알고리즘 담당 | 모델 버전·FSM·성공 조건·포획 지점 metadata 제공 |
| 운영 담당 | 기록 루트, 보관 기간, 백업과 파일 누락 처리 정책 |

## 11. 착수 전에 결정할 질문

- 시험 한 번을 JSON의 trial 한 개로 볼지, rosbag 디렉터리 한 개로 볼지
- 같은 rosbag에서 여러 알고리즘 결과가 생길 수 있는지
- `recorded_at`의 기준을 rosbag 시작 시각으로 할지 수동 입력으로 할지
- 알고리즘 버전을 Git commit으로 저장할지 별도 버전 문자열로 저장할지
- 파일을 어느 PC와 디렉터리에 영구 보관할지
- DB와 파일의 백업·보관 기간을 어떻게 정할지
- 운영자가 잘못 등록한 시험을 수정·비활성화하는 절차를 어떻게 할지

## 12. 완료 기준

- 기존 DB의 탐지·이동 기록이 손상되지 않는다.
- Replay JSON 한 파일의 여러 trial을 각각 등록할 수 있다.
- 같은 JSON 경로와 trial index는 중복 등록되지 않는다.
- 날짜·모델·성공 여부로 시험 목록을 검색할 수 있다.
- 목록 응답에는 큰 프레임 배열과 지도 이미지가 포함되지 않는다.
- 시험 id를 선택하면 기존 쥐몰이 재생 화면이 정상적으로 열린다.
- 원본 파일이 없을 때 서버가 종료되지 않고 누락 상태를 알려준다.
- 경로 조작으로 허용된 기록 디렉터리 밖의 파일을 읽을 수 없다.
- 등록 실패 시 DB에 일부 데이터가 남지 않는다.
- 스키마, 저장·조회, 중복, 파일 누락과 경로 보안 테스트가 통과한다.

## 13. 현재 결론

6단계는 아직 구현하지 않는다. DB 담당자가 이 문서를 검토해 저장 단위와 파일
보관 정책을 확정한 뒤 착수한다. 그전까지는 rosbag2와 Replay JSON을 파일로
보관하고, System Monitor는 지정된 Replay JSON 안의 시험을 선택해 재생한다.
