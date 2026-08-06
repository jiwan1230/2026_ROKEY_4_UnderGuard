# opening·trap 처리 설계

> 순찰 중 구멍(opening)을 발견해 검증하고, trap 설치/점검까지 끝낸 뒤 순찰을
> 이어가는 전체 흐름. detector가 YOLO를 유일하게 쓰고 상태기계 주인이 된다.
> trap_check는 detector가 준 좌표로 판정·설치 주행·beep만 한다.

## 목표

opening 발견 → 순찰 정지 → 접근 → db 조회로 신구(新舊) 판별 → 진위검증 → trap
설치/점검. **설치가 제대로 안 됐으면 넘어가지 않고 beep+설치동작으로 사람에게
trap을 놓게 한 뒤 재점검**한다. 정상이면 순찰 재개. 로봇 1대 로컬 처리
(central은 PATROLLING으로만 앎).

## 상태기계 (detector)

```
SEARCHING ──opening 감지──> APPROACHING ──홀 앞 도착──> QUERYING
QUERYING ──db:처음──> VERIFYING ──진짜──> [db저장 + install job] ──> AWAIT_TRAP
QUERYING ──db:기존(저장좌표 보관)──> INSPECTING
VERIFYING ──가짜(평평)──> 순찰재개
AWAIT_TRAP ──trap_installed──> INSPECTING          # 설치동작 끝 → 항상 재점검
INSPECTING ──trap 감지──> [inspect job] ──> AWAIT_TRAP
INSPECTING ──trap 미검출(timeout)──> 재설치
AWAIT_TRAP ──trap_ok──> 순찰재개                    # 15cm 이내 → 정상
AWAIT_TRAP ──trap_bad──> 재설치                     # 15cm 초과 → 다시 설치
재설치 = [install job + 카운트++]; 카운트 > reinstall_max ──> 경고 + 순찰재개
```

**핵심:** 새 구멍 최초 설치도, 기존 구멍 재설치도 **똑같은 install job**(beep+
앞뒤왕복)을 쓴다. 설치 뒤엔 항상 INSPECTING으로 가서 15cm 재점검하므로, 사람이
제대로 놓을 때까지 (최대 reinstall_max회) 반복한다. 최초 검증 통과한 설치는
카운트에 안 넣고, trap_bad/미검출로 인한 재설치만 센다.

전이별 부수효과:

| 전이 | detector 동작 |
|------|--------------|
| SEARCHING→APPROACHING | `patrol_hold=True` 발행(순찰 정지) + 접근 target_pose |
| APPROACHING→QUERYING | 홀 앞 도착(TF 거리) 시 감지좌표로 `QueryHole` 호출 |
| QUERYING→VERIFYING | 응답 exists=False → 진위검증 |
| QUERYING→INSPECTING | 응답 exists=True → 저장좌표(resp.hole_x,y) 보관 후 trap 감지 |
| VERIFYING→AWAIT_TRAP | depth_spread 통과 → `opening_confirmed` 발행(db저장) + install job |
| VERIFYING→순찰재개 | depth_spread 미달(가짜) → `patrol_hold=False` |
| AWAIT_TRAP→INSPECTING | `trap_installed` 수신 → 다시 봐서 15cm 점검 |
| INSPECTING→AWAIT_TRAP | trap 감지·좌표추출 → inspect job(hole+trap 좌표) |
| INSPECTING→재설치 | timeout까지 trap 미검출 → install job 재발행(카운트++) |
| AWAIT_TRAP→순찰재개 | `trap_ok` 수신 → `patrol_hold=False` |
| AWAIT_TRAP→재설치 | `trap_bad` 수신 → install job 재발행(카운트++) |
| 재설치 카운트 초과 | 경고 로그 + `patrol_hold=False` (그냥 넘어감) |

## 노드 역할

| 노드 | 이 기능에서 | YOLO |
|------|-----------|------|
| detector | 상태기계 주인. opening·trap 감지, 좌표 추출, db 조회, 순찰 정지/재개 신호, install/inspect job 발행, 재설치 루프 관리, trap_check 이벤트 구독 | ✅ 유일 |
| **trap_check** | job 받아서만 동작. **install=beep 울리고 구멍 20cm 앞 전진→후진 주행 후 `trap_installed` 발행**. inspect=hole↔trap 거리 ≤15cm면 `trap_ok`, 아니면 `trap_bad` 발행 | ❌ |
| db_node | `opening_confirmed` 구독해 구멍 좌표 저장. `QueryHole`로 근처 구멍 조회+저장좌표 반환 | ❌ |
| robot_agent | `patrol_hold=True`면 순찰 취소, `False`면 재개 | ❌ |

## 팀원 구현 대상 — trap_check_node.py

> 이 노드 코드는 팀원이 구현한다. detector·db·인터페이스는 완성돼 있으니
> 아래 계약만 지키면 바로 붙는다. 현재 `trap_check_node.py`에 inspect 판정과
> 배관은 되어 있고, **install의 beep·주행만 채우면 된다**(TODO 표시).

**구독:** `trap_job` (turtle_interfaces/TrapJob), 로봇 namespace 상대토픽
**발행:** `/fleet/event` (std_msgs/String, `fleet_msg.event(name, x, y)` 포맷)

**phase == "inspect"** (이미 구현됨):
- `hole_x,y`(db 저장좌표)와 `trap_x,y`(detector가 감지한 trap 좌표) 거리 계산
- ≤ `trap_ok_dist`(0.15m) → `trap_ok:hole_x:hole_y` 발행
- 초과 → `trap_bad:hole_x:hole_y` 발행

**phase == "install"** (팀원 구현):
1. **beep** — 사람에게 trap 설치를 알림 (TB4 부저)
2. **설치동작** — 구멍(`hole_x,y`) 20cm 앞으로 이동 → 후진해 빠져나옴
   (target_pose 발행, robot_agent가 주행). 사람이 이 사이 trap을 놓는다
3. 동작 끝나면 `trap_installed:hole_x:hole_y` 발행
- ※ install은 새 구멍/재설치 공통. detector가 재점검(15cm)까지 관리하므로
  trap_check는 "설치동작 후 trap_installed 발행"까지만 책임진다.

> beep 하드웨어(추측): TB4는 `/{ns}/cmd_audio`
> (irobot_create_msgs/AudioNoteVector)로 부저를 낸다 — 토픽/타입은 팀원이 확인.

## 인터페이스

### TrapJob.msg (완성)
```
string phase        # "install" | "inspect"
float64 hole_x      # 대상 구멍 좌표 (install=검증/저장좌표, inspect=db 저장좌표)
float64 hole_y
float64 trap_x      # inspect 시 detector가 감지한 trap 좌표 (install 시 0)
float64 trap_y
```
detector → trap_check. 로봇 namespace 상대토픽 `trap_job`.

### QueryHole.srv (완성)
```
float64 x
float64 y
---
bool exists
bool trap_installed
float64 hole_x       # 저장된 구멍 좌표 — 점검 비교 기준
float64 hole_y
```

### /fleet/event (재사용, 문자열)
trap_check 발행 → detector·central 구독:
- `trap_installed:x:y` — 설치동작 끝남 (detector가 INSPECTING 재점검으로)
- `trap_ok:x:y` — 점검 정상 (순찰 재개)
- `trap_bad:x:y` — 설치 이상 (detector가 재설치)

### patrol_hold (완성, 로컬)
`std_msgs/Bool`, 로봇 namespace 상대토픽 `patrol_hold`. detector → robot_agent.
True=순찰 정지, False=재개.

## 판정 규칙

- **진짜 구멍**: `depth_spread ≥ depth_gap`(0.05m) — 기존 로직 그대로.
- **구멍 근처 판정(db)**: 저장 구멍과 조회좌표 거리 ≤ `hole_match_dist`(0.3m)면 동일 구멍.
- **trap 설치 정상**: hole↔trap 거리 ≤ `trap_ok_dist`(0.15m).
- **재설치 한도**: `reinstall_max`(3)회까지. 초과 시 경고 후 순찰 재개.

## 결정 사항

1. trap_check는 db를 직접 안 읽는다. detector가 db에서 꺼낸 저장좌표를 job에 담아 넘김 → trap_check는 순수 판정기+주행기.
2. 설치 이상(trap_bad)이면 넘어가지 않고 beep+설치동작으로 재설치, 재점검 루프. reinstall_max회 초과 시에만 경고 후 순찰 재개.
3. install job은 새 구멍/재설치 공통 (beep+앞뒤왕복). 설치 후 항상 15cm 재점검.
4. INSPECTING에서 trap 아예 미검출(timeout)도 재설치 대상 (사람이 아직 안 놓음).

## 범위 (Nav2 게이트)

**완성(Nav2 무관):**
- TrapJob.msg, QueryHole.srv 확장, 인터페이스 빌드
- db_node: 저장·조회+좌표 반환
- trap_check: inspect 15cm 판정 → 이벤트 발행 (판정·배관)
- detector: 상태기계(QUERYING/VERIFYING/INSPECTING/AWAIT_TRAP), db 분기,
  trap 감지·좌표추출, install/inspect job 발행, 재설치 루프, 이벤트 구독, patrol_hold
- robot_agent: `patrol_hold` 구독 → hold 중 순찰 재시작 차단

**게이트("코드 짜줘" 대기) — trap_check는 팀원 몫:**
- trap_check install: beep + 구멍 20cm 전진→후진 주행
- robot_agent: patrol_hold 시 실제 cancelTask 실행, target_cb 실주행
