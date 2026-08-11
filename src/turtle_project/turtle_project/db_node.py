"""구멍/trap 좌표 DB + 쥐대응 사건·미션 기록 (MySQL). MySQL 접근은 이 노드 하나뿐이다.

로봇 런타임 경로(중복판정·조회)는 그대로 유지한다: opening_confirmed 저장,
QueryHole/ListHoles 조회, trap_ok/trap_bad 반영 — 전부 기존과 동일한 /fleet/event,
/db/query_hole, /db/list_holes 인터페이스로 동작한다. 내부 저장소만 holes에서
opening(+trap)으로 옮겼다 — holes는 최초 1회 opening으로 이전된 뒤 더 이상
갱신되지 않는 동결 스냅숏이다 (mysql_init.sql 참고).

추가로 아래를 기존 토픽만으로(신규 제어 인터페이스 없이) 기록한다:
  - incident: rat_detected~rat_captured/lost 사건 하나 (/fleet/event 그대로)
  - robot_mission: 로봇별 역할 시작/종료 (/fleet/status의 실제 도달 상태 기준)
  - detection_event/rat_track: /fleet/detection (신규, 기록 전용 — confidence·
    robot_id는 /fleet/event(name:x:y)에 못 실어 분리했다)
  - trap/trap_inspection: /fleet/trap_inspection (신규, 기록 전용 — 실제 감지된
    trap 좌표·거리는 /fleet/event(trap_ok/bad)에 hole 좌표만 실려 못 받는다)

DB 기록 실패는 로그만 남기고 조용히 넘어간다 — Robot 제어(주행)는 db_node
응답에 의존하지 않는다. QueryHole만 예외로, 실패 시 fallback은 detector_node가
이미 갖고 있다(service_is_ready 확인, 여기선 안전한 기본값만 돌려주면 된다).
"""
import datetime
import decimal
import json
import math
import os

import mysql.connector

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from turtle_interfaces.msg import DetectionEvent, TrapInspection
from turtle_interfaces.srv import DbQuery, ListHoles, QueryHole
from turtle_project import fleet_msg

# robot_agent가 보고하는 상태 중 "역할"로 볼 수 있는 것만 매핑한다. IDLE/
# UNDOCKING/DOCKED는 과도기·정차 상태라 자체 미션을 만들지 않는다 — 특히
# DOCKED는 RETURNING 안에 이미 도킹 액션까지 포함돼 있어(robot_agent.py) 별도
# 지속시간을 갖는 상태가 아니다.
STATE_TO_ROLE = {
    'PATROLLING': 'PATROL', 'TRACKING': 'TRACK', 'HERDING': 'HERD',
    'SWEEPING': 'SWEEP', 'RETURNING': 'RETURN',
}


def jsonable(v):
    """json이 모르는 DB 타입만 바꾼다 — DATETIME은 문자열, Decimal(AVG 결과)은 float."""
    if isinstance(v, datetime.datetime):
        return v.isoformat(sep=' ', timespec='seconds')
    if isinstance(v, decimal.Decimal):
        return float(v)
    return v


def nearest(rows, x, y):
    """rows[(id,x,y)]에서 (x,y)에 가장 가까운 것 -> (idx, 거리). 비면 (None, inf)."""
    best_i, best_d = None, float('inf')
    for i, (_id, rx, ry) in enumerate(rows):
        d = math.hypot(rx - x, ry - y)
        if d < best_d:
            best_i, best_d = i, d
    return best_i, best_d


class DbNode(Node):

    def __init__(self):
        super().__init__('db_node')
        self.match_dist = self.declare_parameter('hole_match_dist', 0.3).value
        host = self.declare_parameter('db_host', 'localhost').value
        user = self.declare_parameter('db_user', 'underguard').value
        database = self.declare_parameter('db_name', 'underguard').value

        try:
            # autocommit=False가 기본값이라, commit() 없는 SELECT(fetch)만 계속
            # 돌면 트랜잭션이 안 끝난 채 남아 metadata lock을 잡는다 — TRUNCATE/
            # ALTER 같은 DDL이 그 연결이 뭘 하기 전까진 영원히 대기하게 된다.
            self.conn = mysql.connector.connect(
                host=host, user=user,
                password=os.environ.get('MYSQL_PASSWORD', ''),
                database=database, autocommit=True)
        except mysql.connector.Error as e:
            self.get_logger().error(f'MySQL 연결 실패 ({host}/{database}): {e}')
            raise

        # 쥐대응 사건 하나 (rat_detected~rat_captured/lost). central의 rat_mode와
        # 같은 이벤트를 보고 독립적으로 같은 가드를 걸 뿐, 직접 결합은 없다.
        self.incident_id = None
        self.incident_started = None
        # 로봇별 "지금 열려 있는 미션" — robot_id -> (mission_id, role, started_at).
        # 같은 role로 반복되는 status(10초 주기)에 새 미션을 계속 만들지 않는 캐시.
        self.open_mission = {}

        self.srv = self.create_service(QueryHole, '/db/query_hole', self.query)
        self.list_srv = self.create_service(
            ListHoles, '/db/list_holes', self.list_holes)
        self.ui_srv = self.create_service(DbQuery, '/db/query', self.db_query)
        self.create_subscription(String, '/fleet/event', self.event_cb, 10)
        self.create_subscription(String, '/fleet/status', self.status_cb, 10)
        self.create_subscription(
            DetectionEvent, '/fleet/detection', self.detection_cb, 10)
        self.create_subscription(
            TrapInspection, '/fleet/trap_inspection', self.trap_inspection_cb, 10)

        n = len(self._load_openings())
        self.get_logger().info(
            f'DB 노드 시작 ({host}/{database}, 구멍 {n}개) — 조회 + opening/trap 저장')

    # ---------- 공용 ----------

    def _run(self, sql, params=(), fetch=False):
        """짧은 쿼리 실행. 실패해도 로그만 남기고 안전한 기본값을 돌려준다
        (fetch=False는 INSERT의 lastrowid, 실패 시 None / fetch=True는 rows,
        실패 시 빈 리스트) — 호출부가 항상 그대로 이어갈 수 있게."""
        try:
            cur = self.conn.cursor()
            cur.execute(sql, params)
            if fetch:
                rows = cur.fetchall()
                cur.close()
                return rows
            self.conn.commit()
            rid = cur.lastrowid
            cur.close()
            return rid
        except mysql.connector.Error as e:
            self.get_logger().error(f'DB 쿼리 실패: {e}')
            return [] if fetch else None

    def _fetch_dicts(self, sql, params=()):
        """조회 전용 — 컬럼명이 붙은 dict 리스트로 돌려준다(UI 응답을 그대로 JSON
        으로 만들기 위함). 실패하면 빈 리스트 — 호출부가 그대로 이어간다."""
        try:
            cur = self.conn.cursor(dictionary=True)
            cur.execute(sql, params)
            rows = [{k: jsonable(v) for k, v in r.items()} for r in cur.fetchall()]
            cur.close()
            return rows
        except mysql.connector.Error as e:
            self.get_logger().error(f'DB 조회 실패: {e}')
            return []

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _load_openings(self):
        """opening 전체를 (opening_id, x, y)로. 근접 매칭 전용."""
        return self._run('SELECT opening_id, x, y FROM opening', fetch=True)

    def _trap_installed(self, opening_id):
        return bool(self._run(
            'SELECT 1 FROM trap WHERE opening_id=%s AND status="ACTIVE" LIMIT 1',
            (opening_id,), fetch=True))

    def _match_opening(self, x, y):
        """(x, y) 근처 opening의 id. 없으면 None. trap 관련 이벤트가 hole 좌표로
        opening을 찾을 때 공용으로 쓴다."""
        rows = self._load_openings()
        i, d = nearest(rows, x, y)
        if i is None or d > self.match_dist:
            return None
        return rows[i][0]

    # ---------- QueryHole / ListHoles (기존 인터페이스 그대로) ----------

    def list_holes(self, req, resp):
        """저장된 모든 구멍 좌표 반환 (trap 유무 무관 — B가 다 점검한다)."""
        rows = self._load_openings()
        resp.xs = [float(x) for _, x, _ in rows]
        resp.ys = [float(y) for _, _, y in rows]
        self.get_logger().info(f'구멍 목록 조회 — {len(rows)}개')
        return resp

    def query(self, req, resp):
        """(req.x, req.y) 근처 저장 구멍 조회. 있으면 저장좌표까지 반환."""
        rows = self._load_openings()
        i, d = nearest(rows, req.x, req.y)
        if i is not None and d <= self.match_dist:
            oid, hx, hy = rows[i]
            resp.exists = True
            resp.trap_installed = self._trap_installed(oid)
            resp.hole_x, resp.hole_y = hx, hy
            self.get_logger().info(f'조회 ({req.x:.2f}, {req.y:.2f}) — 기존 구멍')
        else:
            resp.exists = False
            resp.trap_installed = False
            resp.hole_x = resp.hole_y = 0.0
            self.get_logger().info(f'조회 ({req.x:.2f}, {req.y:.2f}) — 처음 본 구멍')
        return resp

    # ---------- /fleet/event (기존 포맷 그대로, 분기만 확장) ----------

    def event_cb(self, msg):
        name, x, y = fleet_msg.parse_event(msg.data)
        if name == 'opening_confirmed':
            self._store_opening(x, y)
        elif name in ('trap_ok', 'trap_bad'):
            self._update_trap(x, y, installed=(name == 'trap_ok'))
        elif name == 'trap_installed':
            self._create_trap(x, y)
        elif name == 'rat_detected':
            self._start_incident(x, y)
        elif name == 'rat_captured':
            self._end_incident('CAPTURED', x, y)
        elif name == 'rat_lost':
            self._end_incident('LOST', x, y)

    def _store_opening(self, x, y):
        """새 구멍 저장. 근처에 이미 있으면 신규 생성 대신 재확인으로 갱신
        (verification_count 증가, last_seen/last_verified 갱신)."""
        rows = self._load_openings()
        i, d = nearest(rows, x, y)
        if i is not None and d <= self.match_dist:
            self._run(
                'UPDATE opening SET last_seen_at=NOW(), last_verified_at=NOW(), '
                'verification_count=verification_count+1 WHERE opening_id=%s',
                (rows[i][0],))
            return
        self._run(
            'INSERT INTO opening (x, y, first_seen_at, last_seen_at, status, '
            'last_verified_at, verification_count) '
            'VALUES (%s, %s, NOW(), NOW(), "VERIFIED", NOW(), 1)', (x, y))
        self.get_logger().info(f'구멍 저장 ({x:.2f}, {y:.2f})')

    def _create_trap(self, hx, hy):
        """trap_installed — 물리 설치 완료. 설치는 blind 주행이라 이 시점엔 trap
        좌표를 모른다 — x/y는 점검(trap_inspection_cb)에서 실제로 보이면 채운다."""
        oid = self._match_opening(hx, hy)
        if oid is None:
            return
        existing = self._run(
            'SELECT trap_id FROM trap WHERE opening_id=%s', (oid,), fetch=True)
        if existing:
            self._run(
                'UPDATE trap SET installed_at=NOW(), status="ACTIVE" '
                'WHERE trap_id=%s', (existing[0][0],))
        else:
            self._run(
                'INSERT INTO trap (opening_id, installed_at, status) '
                'VALUES (%s, NOW(), "ACTIVE")', (oid,))

    def _update_trap(self, hx, hy, installed):
        """trap_ok/trap_bad — 거리 판정은 trap_check가 이미 끝냈다. db_node는
        결과만 받아 적는다 (기존 holes.trap_installed와 같은 책임 분리)."""
        oid = self._match_opening(hx, hy)
        if oid is None:
            return
        status = 'ACTIVE' if installed else 'MOVED'
        self._run(
            'UPDATE trap SET status=%s, last_checked_at=NOW() WHERE opening_id=%s',
            (status, oid))
        self.get_logger().info(f'trap 상태 갱신 ({hx:.2f}, {hy:.2f}) → {installed}')

    # ---------- 사건 (incident) ----------

    def _start_incident(self, x, y):
        """rat_detected — 이미 진행 중인 사건이 있으면 무시. detector가 같은 쥐를
        여러 프레임 연속으로 rat_detected 쏘는 동안(중앙 응답 전) 사건이 여러 개
        생기지 않게 막는 가드 — central_node의 rat_mode와 동일한 원리다."""
        if self.incident_id is not None:
            return
        self.incident_started = self._now()
        self.incident_id = self._run(
            'INSERT INTO incident (first_detected_at, first_x, first_y, severity, '
            'status) VALUES (NOW(), %s, %s, %s, %s)',
            (x, y, 'EMERGENCY', 'ACTIVE'))
        if self.incident_id:
            self.get_logger().info(f'사건 시작 #{self.incident_id} ({x:.2f}, {y:.2f})')
        else:
            self.incident_started = None   # 저장 실패 — 다음 rat_detected에서 재시도

    def _end_incident(self, result, x, y):
        """rat_captured/rat_lost — 진행 중인 사건 종료. 사건 중 아니면 무시
        (쥐대응 종료 후 늦게 온 중복 이벤트 등)."""
        if self.incident_id is None:
            return
        resp_sec = int(self._now() - self.incident_started) \
            if self.incident_started is not None else None
        if result == 'CAPTURED':
            self._run(
                'UPDATE incident SET status="CLOSED", result=%s, '
                'response_seconds=%s, captured_at=NOW() WHERE incident_id=%s',
                (result, resp_sec, self.incident_id))
        else:
            self._run(
                'UPDATE incident SET status="CLOSED", result=%s, '
                'response_seconds=%s WHERE incident_id=%s',
                (result, resp_sec, self.incident_id))
        self.get_logger().info(f'사건 종료 #{self.incident_id} → {result}')
        self._close_role_missions(result)
        self.incident_id = None
        self.incident_started = None

    # ---------- 로봇 미션 (/fleet/status — 명령이 아니라 실제 도달 상태 기준) ----------

    def status_cb(self, msg):
        """robot_agent가 실제로 도달한 state로 미션을 관리한다. 명령(/fleet/
        command) 기준으로 하면 undock 후 자동 순찰처럼 명령 없이 일어나는 전이를
        놓친다 — 상태머신/명령 흐름은 안 건드리고 보고만 관찰한다."""
        robot, state, _battery = fleet_msg.parse_status(msg.data)
        role = STATE_TO_ROLE.get(state)
        open_ = self.open_mission.get(robot)
        now = self._now()
        if open_ and open_[1] == role:
            return                      # 같은 role 유지 중 — 중복 생성 방지
        if open_:
            result, reason = self._infer_mission_result(open_[1], state)
            self._close_mission(robot, open_[0], open_[2], now, result, reason)
        if role is not None:
            self._open_mission(robot, role, now)

    def _infer_mission_result(self, prev_role, new_state):
        """상태 전이에서 실제로 판단 가능한 결과만 — 나머지는 모른다(None).
        robot_agent._dock_tick: dock 액션 완료 후 실제 도킹 성공만 DOCKED,
        실패는 IDLE로 떨어진다 — 이 둘만 결과를 확실히 안다."""
        if prev_role == 'RETURN':
            if new_state == 'DOCKED':
                return 'SUCCESS', None
            if new_state == 'IDLE':
                return 'FAILED', '도킹 실패'
        return None, None

    def _open_mission(self, robot, role, now):
        incident_id = self.incident_id if role in ('TRACK', 'HERD', 'SWEEP') else None
        mission_id = self._run(
            'INSERT INTO robot_mission (incident_id, robot_id, role, started_at) '
            'VALUES (%s, %s, %s, NOW())', (incident_id, robot, role))
        if mission_id:
            self.open_mission[robot] = (mission_id, role, now)

    def _close_mission(self, robot, mission_id, started, now, result, reason=None):
        self._run(
            'UPDATE robot_mission SET ended_at=NOW(), duration_sec=%s, result=%s, '
            'failure_reason=%s WHERE mission_id=%s',
            (int(now - started), result, reason, mission_id))
        self.open_mission.pop(robot, None)

    def _close_role_missions(self, result):
        """사건 종료 시 진행 중이던 TRACK/HERD/SWEEP 미션을 사건 결과로 닫는다
        (status 갱신은 최대 10초 늦으므로 여기서 즉시 닫아야 정확하다)."""
        now = self._now()
        for robot, (mission_id, role, started) in list(self.open_mission.items()):
            if role in ('TRACK', 'HERD', 'SWEEP'):
                self._close_mission(robot, mission_id, started, now, result)

    # ---------- 기록 전용 신규 토픽 ----------

    def detection_cb(self, msg):
        """/fleet/detection — RAT/OPENING 탐지 기록. RAT은 사건 중일 때만
        incident_id를 붙이고, 그 좌표는 rat_track에도 남긴다(사건 없으면 안 남김)."""
        incident_id = self.incident_id if msg.object_type == 'RAT' else None
        self._run(
            'INSERT INTO detection_event (incident_id, detected_at, object_type, '
            'x, y, confidence, robot_id) VALUES (%s, NOW(), %s, %s, %s, %s, %s)',
            (incident_id, msg.object_type, msg.x, msg.y, msg.confidence,
             msg.robot_id))
        if msg.object_type == 'RAT' and self.incident_id is not None:
            self._run(
                'INSERT INTO rat_track (incident_id, `timestamp`, x, y, '
                'confidence) VALUES (%s, NOW(), %s, %s, %s)',
                (self.incident_id, msg.x, msg.y, msg.confidence))

    def trap_inspection_cb(self, msg):
        """/fleet/trap_inspection — trap_check가 실제로 감지한 trap 좌표·거리.
        trap.x/y는 관측될 때마다 최신값으로 갱신한다(설치 시점엔 몰라 NULL이었다)."""
        oid = self._match_opening(msg.hole_x, msg.hole_y)
        if oid is None:
            return
        row = self._run(
            'SELECT trap_id FROM trap WHERE opening_id=%s', (oid,), fetch=True)
        if not row:
            return              # install(trap_installed) 이벤트가 먼저 와야 정상
        trap_id = row[0][0]
        self._run('UPDATE trap SET x=%s, y=%s WHERE trap_id=%s',
                  (msg.trap_x, msg.trap_y, trap_id))
        self._run(
            'INSERT INTO trap_inspection (trap_id, robot_id, checked_at, result, '
            'detected_x, detected_y, distance_from_opening) '
            'VALUES (%s, %s, NOW(), %s, %s, %s, %s)',
            (trap_id, msg.robot_id, msg.result, msg.trap_x, msg.trap_y,
             msg.distance))


    # ---------- /db/query (UI 조회 전용, 읽기만) ----------

    def db_query(self, req, resp):
        """UI(System Monitor)의 기록 조회. 읽기 전용이라 로봇 제어와 겹치는 상태가
        없고, 어떤 실패든 ok=False로만 돌려준다 — 노드는 계속 돈다.

        전부 LIMIT/집계로 응답이 작게 끝나는 쿼리다. 실행자가 단일 스레드라 여기서
        오래 걸리면 QueryHole도 같이 밀리는데, detector_node에 5초 워치독 fallback이
        이미 있어 최악의 경우에도 로봇은 멈추지 않는다."""
        resp.ok, resp.result_json, resp.error = False, '', ''
        try:
            p = json.loads(req.params_json) if req.params_json else {}
            limit = max(1, min(int(p.get('limit', 100)), 500))
            if req.query_name == 'detections':
                out = {'rows': self._q_detections(limit)}
            elif req.query_name == 'missions':
                out = {'rows': self._q_missions(limit)}
            elif req.query_name == 'traps':
                out = {'rows': self._q_traps()}
            elif req.query_name == 'report':
                out = self._q_report()
            else:
                resp.error = f'모르는 query_name: {req.query_name}'
                return resp
            resp.result_json = json.dumps(out, ensure_ascii=False)
            resp.ok = True
        except Exception as e:      # JSON 파싱·타입 등 뭐가 터져도 서비스는 응답한다
            resp.error = str(e)
            self.get_logger().warn(f'UI 조회 실패 ({req.query_name}): {e}')
        return resp

    def _q_detections(self, limit):
        """탐지기록. trap_installed는 구멍(OPENING) 행에만 의미가 있어 쥐(RAT)
        행에는 null을 준다 — 화면에서 빈칸으로 두면 된다."""
        return self._fetch_dicts(
            'SELECT d.detection_id, d.incident_id, d.detected_at, d.object_type, '
            'd.x, d.y, d.confidence, d.robot_id, d.image_path, '
            "CASE WHEN d.object_type <> 'OPENING' THEN NULL "
            '     WHEN MAX(t.trap_id) IS NULL THEN 0 ELSE 1 END AS trap_installed '
            'FROM detection_event d '
            'LEFT JOIN opening o ON ABS(o.x - d.x) <= %s AND ABS(o.y - d.y) <= %s '
            'LEFT JOIN trap t ON t.opening_id = o.opening_id '
            'GROUP BY d.detection_id ORDER BY d.detected_at DESC LIMIT %s',
            (self.match_dist, self.match_dist, limit))

    def _q_missions(self, limit):
        """순찰·대응 기록. PATROL/SWEEP은 result가 null인 게 정상이다 — 순찰 중
        이상을 알리는 이벤트가 아직 없어서 판정할 근거가 없다."""
        return self._fetch_dicts(
            'SELECT mission_id, incident_id, robot_id, role, started_at, ended_at, '
            'duration_sec, result, failure_reason '
            'FROM robot_mission ORDER BY started_at DESC LIMIT %s', (limit,))

    def _q_traps(self):
        """트랩 상세 — 구멍 좌표와 점검 이력 요약을 붙여서."""
        return self._fetch_dicts(
            'SELECT t.trap_id, t.opening_id, o.x AS opening_x, o.y AS opening_y, '
            't.x AS trap_x, t.y AS trap_y, t.installed_at, t.status, '
            't.last_checked_at, COUNT(i.inspection_id) AS inspections, '
            "CAST(IFNULL(SUM(i.result = 'MOVED'), 0) AS SIGNED) AS moved_count, "
            'AVG(i.distance_from_opening) AS avg_drift_m '
            'FROM trap t JOIN opening o ON o.opening_id = t.opening_id '
            'LEFT JOIN trap_inspection i ON i.trap_id = t.trap_id '
            'GROUP BY t.trap_id ORDER BY t.trap_id')

    def _q_report(self):
        """한 사이클 보고서 — 저장된 집계 테이블 없이 매번 계산한다. 표가 작아
        빠르고, 기록이 고쳐지면 보고서도 자동으로 맞는다."""
        agg = self._fetch_dicts(
            'SELECT COUNT(*) AS total, '
            "CAST(IFNULL(SUM(result = 'CAPTURED'), 0) AS SIGNED) AS captured, "
            "CAST(IFNULL(SUM(result = 'LOST'), 0) AS SIGNED) AS lost, "
            'AVG(response_seconds) AS avg_response_sec, '
            'MAX(response_seconds) AS max_response_sec '
            "FROM incident WHERE status = 'CLOSED'")
        inc = agg[0] if agg else {'total': 0, 'captured': 0, 'lost': 0,
                                  'avg_response_sec': None, 'max_response_sec': None}
        inc['success_rate_pct'] = (
            round(100.0 * inc['captured'] / inc['total'], 1) if inc['total'] else None)
        return {
            'generated_at': datetime.datetime.now().isoformat(
                sep=' ', timespec='seconds'),
            'incidents': inc,
            'missions_by_role': self._fetch_dicts(
                'SELECT role, COUNT(*) AS count, AVG(duration_sec) AS avg_sec, '
                'MIN(duration_sec) AS min_sec, MAX(duration_sec) AS max_sec '
                'FROM robot_mission WHERE ended_at IS NOT NULL '
                'GROUP BY role ORDER BY AVG(duration_sec) DESC'),
            'traps': self._q_traps(),
        }


def main():
    rclpy.init()
    node = DbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


def _self_check():
    rows = [(1, 0.0, 0.0), (2, 5.0, 5.0)]
    i, d = nearest(rows, 0.1, 0.1)
    assert i == 0 and abs(d - math.hypot(0.1, 0.1)) < 1e-9
    i, _ = nearest(rows, 4.9, 5.1)
    assert i == 1
    assert nearest([], 1.0, 2.0) == (None, float('inf'))
    print('db_node self-check ok (nearest() 로직만 검증 — MySQL 연동은 실제 노드로 확인)')


if __name__ == '__main__':
    import sys
    _self_check() if '--check' in sys.argv else main()
