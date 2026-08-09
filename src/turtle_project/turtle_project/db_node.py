"""구멍/trap 좌표 DB — QueryHole 서비스 + opening_confirmed 저장 (SQLite 영속).

detector가 진짜 구멍을 확인하면 /fleet/event로 opening_confirmed를 쏘고, 여기서
좌표를 SQLite에 저장한다. QueryHole 조회는 근처(hole_match_dist 이내) 구멍이
있으면 exists=True + 그 저장좌표를 돌려준다 (trap_check가 15cm 점검에 쓴다).
trap_ok/trap_bad를 받아 해당 구멍의 trap 설치 상태도 갱신한다.

SQLite라 db_node를 재시작해도 구멍이 남는다. UI(system_monitor)는 이 DB를 직접
안 보고 /fleet/event를 자체 구독하므로, 여기서 UI용 발행은 하지 않는다.
"""
import math
import sqlite3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from turtle_interfaces.srv import ListHoles, QueryHole
from turtle_project import fleet_msg


def nearest(holes, x, y):
    """holes[(x,y,trap)]에서 (x,y)에 가장 가까운 것 -> (idx, 거리). 비면 (None, inf)."""
    best_i, best_d = None, float('inf')
    for i, (hx, hy, _) in enumerate(holes):
        d = math.hypot(hx - x, hy - y)
        if d < best_d:
            best_i, best_d = i, d
    return best_i, best_d


class DbNode(Node):

    def __init__(self):
        super().__init__('db_node')
        self.match_dist = self.declare_parameter('hole_match_dist', 0.3).value
        db_path = self.declare_parameter('db_path', 'holes.db').value

        # 콜백은 단일 스레드지만 sqlite는 기본이 스레드 소유 검사라 끄고 쓴다.
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.conn.execute(
            'CREATE TABLE IF NOT EXISTS holes ('
            'id INTEGER PRIMARY KEY, x REAL, y REAL, trap_installed INTEGER)')
        self.conn.commit()

        self.srv = self.create_service(QueryHole, '/db/query_hole', self.query)
        # B가 쥐대응 시 순회 점검할 전체 구멍 목록 (좌표만).
        self.list_srv = self.create_service(
            ListHoles, '/db/list_holes', self.list_holes)
        self.create_subscription(String, '/fleet/event', self.event_cb, 10)
        n = len(self._load())
        self.get_logger().info(
            f'DB 노드 시작 ({db_path}, 구멍 {n}개) — 조회 + opening/trap 저장')

    def _load(self):
        """저장된 모든 구멍 -> [(x, y, trap_installed)]. 조회/근접판정에 쓴다."""
        cur = self.conn.execute('SELECT x, y, trap_installed FROM holes')
        return [(x, y, bool(t)) for x, y, t in cur.fetchall()]

    def event_cb(self, msg):
        """opening_confirmed는 신규 구멍 저장, trap_ok/bad는 trap 상태 갱신."""
        name, x, y = fleet_msg.parse_event(msg.data)
        if name == 'opening_confirmed':
            self._store_hole(x, y)
        elif name in ('trap_ok', 'trap_bad'):
            self._update_trap(x, y, installed=(name == 'trap_ok'))

    def _store_hole(self, x, y):
        """새 구멍 저장 (근처에 이미 있으면 스킵)."""
        _, d = nearest(self._load(), x, y)
        if d <= self.match_dist:
            return          # 이미 아는 구멍 — 재저장 안 함
        self.conn.execute(
            'INSERT INTO holes (x, y, trap_installed) VALUES (?, ?, 0)', (x, y))
        self.conn.commit()
        self.get_logger().info(f'구멍 저장 ({x:.2f}, {y:.2f})')

    def _update_trap(self, x, y, installed):
        """근처 구멍의 trap 설치 상태 갱신 (UI/조회가 최신 상태를 보게)."""
        holes = self._load()
        i, d = nearest(holes, x, y)
        if i is None or d > self.match_dist:
            return          # 아는 구멍 아님 — 무시
        hx, hy, _ = holes[i]
        self.conn.execute(
            'UPDATE holes SET trap_installed=? WHERE x=? AND y=?',
            (1 if installed else 0, hx, hy))
        self.conn.commit()
        self.get_logger().info(
            f'trap 상태 갱신 ({hx:.2f}, {hy:.2f}) → {installed}')

    def list_holes(self, req, resp):
        """저장된 모든 구멍 좌표 반환 (trap 유무 무관 — B가 다 점검한다)."""
        holes = self._load()
        resp.xs = [float(x) for x, _, _ in holes]
        resp.ys = [float(y) for _, y, _ in holes]
        self.get_logger().info(f'구멍 목록 조회 — {len(holes)}개')
        return resp

    def query(self, req, resp):
        """(req.x, req.y) 근처 저장 구멍 조회. 있으면 저장좌표까지 반환."""
        holes = self._load()
        i, d = nearest(holes, req.x, req.y)
        if i is not None and d <= self.match_dist:
            hx, hy, trap = holes[i]
            resp.exists = True
            resp.trap_installed = trap
            resp.hole_x, resp.hole_y = hx, hy
            self.get_logger().info(f'조회 ({req.x:.2f}, {req.y:.2f}) — 기존 구멍')
        else:
            resp.exists = False
            resp.trap_installed = False
            resp.hole_x = resp.hole_y = 0.0
            self.get_logger().info(f'조회 ({req.x:.2f}, {req.y:.2f}) — 처음 본 구멍')
        return resp


def main():
    rclpy.init()
    node = DbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


def _self_check():
    holes = [(0.0, 0.0, False), (5.0, 5.0, True)]
    i, d = nearest(holes, 0.1, 0.1)
    assert i == 0 and abs(d - math.hypot(0.1, 0.1)) < 1e-9
    i, _ = nearest(holes, 4.9, 5.1)
    assert i == 1
    assert nearest([], 1.0, 2.0) == (None, float('inf'))

    # SQLite 저장/조회/갱신 round-trip (임시 in-memory DB)
    conn = sqlite3.connect(':memory:')
    conn.execute('CREATE TABLE holes (id INTEGER PRIMARY KEY, x REAL, y REAL, '
                 'trap_installed INTEGER)')
    conn.execute('INSERT INTO holes (x, y, trap_installed) VALUES (1.0, 2.0, 0)')
    conn.commit()
    rows = [(x, y, bool(t)) for x, y, t in
            conn.execute('SELECT x, y, trap_installed FROM holes')]
    assert rows == [(1.0, 2.0, False)]
    conn.execute('UPDATE holes SET trap_installed=1 WHERE x=1.0 AND y=2.0')
    conn.commit()
    rows = [(x, y, bool(t)) for x, y, t in
            conn.execute('SELECT x, y, trap_installed FROM holes')]
    assert rows == [(1.0, 2.0, True)]
    print('db_node self-check ok')


if __name__ == '__main__':
    import sys
    _self_check() if '--check' in sys.argv else main()
