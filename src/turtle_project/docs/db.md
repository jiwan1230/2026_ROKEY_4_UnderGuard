# db_node — 구멍 좌표 DB

로봇이 검증한 침입구(opening) 좌표를 MySQL에 영속화한다.
목적은 하나 — **다음 순찰에서 같은 구멍을 다시 검증하지 않게 하는 것**.

구현: `turtle_project/db_node.py` · 실행 위치: 중앙 PC(PC1, MySQL 서버도 같이 뜬다) · DB: `underguard.holes`

로봇 PC(robot4/robot6)는 MySQL에 직접 접근하지 않는다. db_node의 서비스/토픽만 거친다.

## 저장 대상

프로세스가 죽어도 남아야 로봇이 헛수고를 안 하는 정보만 넣는다.
로봇 위치·배터리·미션 상태는 실시간 값이라 넣지 않는다 (`central_node` 메모리 소관).

```sql
CREATE TABLE IF NOT EXISTS holes (
    id INTEGER PRIMARY KEY AUTO_INCREMENT,
    x DOUBLE, y DOUBLE,      -- map 프레임 좌표
    trap_installed INTEGER   -- 0 | 1
)
```

스키마는 SQLite 시절과 동일하다 (테이블 확장은 다음 단계). 실제 생성은 `config/mysql_init.sql` 로 한다.

## 인터페이스

| 종류 | 이름 | 방향 | 용도 |
|---|---|---|---|
| Service | `/db/query_hole` (`QueryHole`) | detector → db | 이 좌표가 기존 구멍인가? |
| Service | `/db/list_holes` (`ListHoles`) | detector → db | SWEEP 순회용 전체 좌표 |
| Topic | `/fleet/event` (`std_msgs/String`) | detector·trap_check → db | `opening_confirmed` / `trap_ok` / `trap_bad` |

조회는 Service(응답 있음), 저장은 Topic(응답 없음)으로 비대칭이다.
저장 실패를 발행 측이 알 방법이 없다 — 아래 한계 2번.

UI(`system_monitor`)는 이 DB를 직접 읽지 않는다. 같은 `/fleet/event`를 자체 구독한다.

## 핵심 규칙

**근접 판정** — 파라미터 `hole_match_dist` (기본 `0.3`, 설계 범위 0.25~0.35m).
좌표 오차 때문에 같은 구멍이 매번 다른 값으로 들어오므로, 이 반경 안이면 동일 구멍으로 본다.

| 상황 | 동작 |
|---|---|
| `opening_confirmed`, 반경 안에 기존 구멍 있음 | INSERT 안 함 (조용히 return) |
| `opening_confirmed`, 없음 | INSERT, `trap_installed=0` |
| `trap_ok` / `trap_bad`, 반경 안 | 해당 행 `trap_installed` 갱신 |
| `trap_ok` / `trap_bad`, 반경 밖 | 무시 |

**원본 좌표 반환** — `query()`는 요청 좌표가 아니라 **최초 저장 좌표**를 돌려준다.
`trap_check_node`의 15cm 판정 기준점이 관측마다 흔들리면 안 되기 때문이다.

**거리 판정은 여기서 안 한다** — trap 정상/이상 판정은 `trap_check_node.trap_ok()` 소관.
db_node는 결과를 받아 적기만 한다.

## 코드 구조

| 함수 | 하는 일 |
|---|---|
| `nearest(holes, x, y)` | 최근접 구멍의 `(idx, 거리)`. 빈 목록이면 `(None, inf)` |
| `_load()` | 전체 행을 `[(x, y, trap)]`로 읽음. 매 호출마다 전량 조회 |
| `event_cb(msg)` | 이벤트 이름으로 분기 |
| `_store_hole(x, y)` | 근접 검사 후 INSERT |
| `_update_trap(x, y, installed)` | 근접 구멍의 trap 상태 UPDATE |
| `query(req, resp)` | `exists` + 저장 좌표 + trap 상태 응답 |
| `list_holes(req, resp)` | 전체 좌표를 `xs` / `ys` 배열로 응답 |

사실상 `nearest()` 하나를 저장·갱신·조회 세 경로가 공유하는 구조다.

## 실행 · 확인

```bash
# PC1에서 최초 1회 — DB/테이블 생성 + db_node 전용 계정
mysql -u root -p < config/mysql_init.sql
mysql -u root -p -e "CREATE USER 'underguard'@'localhost' IDENTIFIED BY '실제비번';
                      GRANT ALL ON underguard.* TO 'underguard'@'localhost';"

export MYSQL_PASSWORD='실제비번'     # db_node를 띄우는 셸에서
ros2 run turtle_project db_node      # 기동 로그에 기존 구멍 개수가 찍힘

python3 db_node.py --check           # 노드 없이 nearest() 로직만 점검 (DB 불필요)

ros2 service call /db/query_hole turtle_interfaces/srv/QueryHole "{x: -2.15, y: 0.35}"
ros2 service call /db/list_holes turtle_interfaces/srv/ListHoles "{}"
ros2 topic pub --once /fleet/event std_msgs/String "data: 'opening_confirmed:-2.15:0.35'"

MYSQL_PWD='실제비번' mysql -u underguard underguard -e "SELECT * FROM holes;"
```

`db_host`(기본 `localhost`), `db_user`(기본 `underguard`), `db_name`(기본 `underguard`), `hole_match_dist` 는
ROS 파라미터로 덮어쓸 수 있다. 비밀번호는 파라미터가 아니라 `MYSQL_PASSWORD` 환경변수로만 받는다 —
코드·launch 파일 어디에도 하드코딩하지 않는다.

## 알려진 한계

| # | 내용 | 조치 |
|---|---|---|
| 1 | `event_cb`에 예외 처리 없음 — 형식 틀린 이벤트 1건에 노드 종료. launch 미등록이라 자동 재시작도 없음 | `try/except (AttributeError, TypeError, ValueError)` 3줄. 같은 파서를 쓰는 Sysmon 쪽 `ros_bridge.py`에 동일 패턴이 이미 있음 |
| 2 | 저장 경로에 ack 없음 | 저장도 Service로 전환 검토 |
| 3 | UPDATE(좌표)·DELETE 경로 없음 — 오탐 제거 불가, 최초 좌표가 영구 기준점 | 재관측 평균화 검토 |
| 4 | `list_holes`가 trap 상태 미반환 — B가 설치 완료 구멍도 재순회 | `ListHoles.srv`에 배열 1개 추가 |
| 5 | 중복 스킵 시 무로그 | 로그 1줄 |
| 6 | `main()`이 `rclpy.shutdown()` 중복 호출 — Ctrl+C 시 트레이스백 | `rclpy.ok()` 확인 후 호출 |
| 7 | 기동 시 MySQL 연결 실패하면 노드가 죽고 재연결을 시도하지 않음 | 의도적 — 최소 예외처리만 넣기로 함(재연결/커넥션풀은 이번 단계 범위 밖) |
| 8 | 런타임 중 연결이 끊기면(idle timeout 등) 각 쿼리가 실패 로그만 남기고 빈 값/무동작으로 넘어감 | 발생 시 노드 재시작 필요. 재연결 로직은 다음 단계 |

1번이 유일하게 가용성에 직접 영향을 준다. `service_is_ready()`가 False면 detector는
"처음 본 구멍"으로 간주하고 진행하므로 순찰은 멈추지 않지만, 아는 구멍을 전부 재검증한다.

## 관련

- `docs/architecture.md` — 전체 노드 구성
- `turtle_project/detector_node.py` — 유일한 조회 클라이언트
- `turtle_project/trap_check_node.py` — trap 정상/이상 판정 주체
- `turtle_interfaces/srv/QueryHole.srv`, `ListHoles.srv`
