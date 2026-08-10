"""Nav2 목표 계산 헬퍼 — 접근점 계산·PoseStamped 생성. 노드가 import해서 사용.

    g = approach_point(dx, dy, stop_dist)  # 물체 앞 stop_dist 지점 (gx, gy, yaw)
    pose = make_pose('map', x, y, yaw)     # target_pose로 발행할 PoseStamped
"""
import math

from geometry_msgs.msg import PoseStamped


def approach_point(x, y, stop_dist):
    """base_link 기준 물체 (x, y) -> 물체 앞 stop_dist 지점 (gx, gy, yaw).

    물체를 바라보는 방향으로 yaw를 잡는다. 이미 stop_dist 안이면 None.
    """
    d = math.hypot(x, y)
    if d <= stop_dist:
        return None
    r = (d - stop_dist) / d
    return x * r, y * r, math.atan2(y, x)


def make_pose(frame, x, y, yaw):
    """(x, y, yaw) -> PoseStamped. stamp는 0 = TF 최신값 사용."""
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2)
    p.pose.orientation.w = math.cos(yaw / 2)
    return p


def _self_check():
    x, y, yaw = approach_point(2.0, 0.0, 0.5)          # 정면 2m
    assert abs(x - 1.5) < 1e-9 and abs(y) < 1e-9 and abs(yaw) < 1e-9

    x, y, yaw = approach_point(0.0, 2.0, 0.5)          # 왼쪽 2m
    assert abs(x) < 1e-9 and abs(y - 1.5) < 1e-9
    assert abs(yaw - math.pi / 2) < 1e-9

    x, y, _ = approach_point(3.0, 4.0, 1.0)            # d=5 -> r=0.8
    assert abs(x - 2.4) < 1e-9 and abs(y - 3.2) < 1e-9

    assert approach_point(0.3, 0.0, 0.5) is None       # 이미 가까움

    p = make_pose('map', 1.0, 0.0, math.pi / 2)
    assert abs(p.pose.orientation.z - math.sqrt(0.5)) < 1e-9
    assert abs(p.pose.orientation.w - math.sqrt(0.5)) < 1e-9
    print('nav_controller self-check ok')


if __name__ == '__main__':
    _self_check()
