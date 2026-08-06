"""구멍/trap 좌표 DB — QueryHole 서비스 + opening_confirmed 저장.

detector가 진짜 구멍을 확인하면 /fleet/event로 opening_confirmed를 쏘고, 여기서
좌표를 저장한다. QueryHole 조회는 근처(hole_match_dist 이내) 구멍이 있으면
exists=True + 그 저장좌표를 돌려준다 (trap_check가 15cm 점검에 쓴다).

저장은 메모리 리스트 (프로세스 살아있는 동안만). 영속화는 필요하면 나중에.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from turtle_interfaces.srv import QueryHole
from turtle_project import fleet_msg


def nearest(holes, x, y):
    """holes에서 (x,y)에 가장 가까운 구멍 -> (idx, 거리). 비었으면 (None, inf)."""
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
        self.holes = []     # [(x, y, trap_installed)]
        self.srv = self.create_service(QueryHole, '/db/query_hole', self.query)
        self.create_subscription(String, '/fleet/event', self.event_cb, 10)
        self.get_logger().info('DB 노드 시작 — /db/query_hole + opening_confirmed 저장')

    def event_cb(self, msg):
        """opening_confirmed 수신 시 구멍 좌표 저장 (중복이면 스킵)."""
        name, x, y = fleet_msg.parse_event(msg.data)
        if name != 'opening_confirmed':
            return
        _, d = nearest(self.holes, x, y)
        if d <= self.match_dist:
            return          # 이미 아는 구멍 — 재저장 안 함
        self.holes.append((x, y, False))
        self.get_logger().info(f'구멍 저장 ({x:.2f}, {y:.2f}) — 총 {len(self.holes)}개')

    def query(self, req, resp):
        """(req.x, req.y) 근처 저장 구멍 조회. 있으면 저장좌표까지 반환."""
        i, d = nearest(self.holes, req.x, req.y)
        if i is not None and d <= self.match_dist:
            hx, hy, trap = self.holes[i]
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
    i, d = nearest(holes, 4.9, 5.1)
    assert i == 1
    assert nearest([], 1.0, 2.0) == (None, float('inf'))
    print('db_node self-check ok')


if __name__ == '__main__':
    import sys
    _self_check() if '--check' in sys.argv else main()
