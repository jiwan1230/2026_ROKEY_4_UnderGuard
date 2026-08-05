"""구멍/trap 좌표 DB — QueryHole 서비스 서버.

좌표를 받아 전에 감지된 구멍이 있는지, trap이 설치됐는지 응답. 저장/조회
로직은 TODO(팀원). 뼈대는 빈 저장소 + 항상 exists=False 응답.
"""
import rclpy
from rclpy.node import Node

from turtle_interfaces.srv import QueryHole


class DbNode(Node):

    def __init__(self):
        super().__init__('db_node')
        self.holes = []     # TODO(팀원): [(x, y, trap_installed)] 기록/조회
        self.srv = self.create_service(QueryHole, '/db/query_hole', self.query)
        self.get_logger().info('DB 노드 시작 — /db/query_hole 대기')

    def query(self, req, resp):
        # TODO(팀원): self.holes에서 (req.x, req.y) 근처 구멍 찾기.
        self.get_logger().info(f'조회 ({req.x:.2f}, {req.y:.2f}) — 뼈대: 없음')
        resp.exists = False
        resp.trap_installed = False
        return resp


def main():
    rclpy.init()
    node = DbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
