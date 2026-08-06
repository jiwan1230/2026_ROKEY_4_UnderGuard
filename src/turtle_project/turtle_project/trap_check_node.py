"""trap 설치/점검 — detector가 준 TrapJob으로만 동작 (지각 안 함).

detector가 YOLO를 유일하게 쓰고, 여기는 좌표만 받아 판정/주행한다.
  install: 구멍 20cm 앞 전진→후진 주행 후 trap_installed 발행 (주행은 Nav2 게이트)
  inspect: 구멍좌표 vs trap좌표 거리 ≤ trap_ok_dist 면 trap_ok, 아니면 trap_bad

판정 결과는 /fleet/event로 발행 → detector가 받아 순찰 재개.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from turtle_interfaces.msg import TrapJob
from turtle_project import fleet_msg


def trap_ok(hole_x, hole_y, trap_x, trap_y, ok_dist):
    """구멍↔trap 거리가 ok_dist 이내면 True (설치 정상)."""
    return math.hypot(hole_x - trap_x, hole_y - trap_y) <= ok_dist


class TrapCheckNode(Node):

    def __init__(self):
        super().__init__('trap_check_node')
        self.ok_dist = self.declare_parameter('trap_ok_dist', 0.15).value

        self.event_pub = self.create_publisher(String, '/fleet/event', 10)
        self.create_subscription(TrapJob, 'trap_job', self.job_cb, 10)
        self.get_logger().info(f'trap 점검 노드 시작 — 정상 거리 {self.ok_dist}m')

    def job_cb(self, msg):
        if msg.phase == 'inspect':
            self._inspect(msg)
        elif msg.phase == 'install':
            self._install(msg)
        else:
            self.get_logger().warn(f'알 수 없는 phase: {msg.phase}')

    def _inspect(self, job):
        """기존 구멍 점검 — 저장좌표 vs 감지된 trap좌표 거리 판정."""
        ok = trap_ok(job.hole_x, job.hole_y, job.trap_x, job.trap_y, self.ok_dist)
        name = 'trap_ok' if ok else 'trap_bad'
        d = math.hypot(job.hole_x - job.trap_x, job.hole_y - job.trap_y)
        self.get_logger().info(f'점검: 거리 {d:.3f}m → {name}')
        self.event_pub.publish(String(data=fleet_msg.event(
            name, job.hole_x, job.hole_y)))

    def _install(self, job):
        """trap 설치 동작 — beep로 사람 호출 + 구멍 20cm 앞 전진→후진 주행.

        새 구멍 최초 설치와 재설치(trap_bad/미검출) 공통. 사람이 이 사이 trap을
        놓는다. 동작이 끝나면 trap_installed 발행 → detector가 15cm 재점검한다.
        """
        # TODO(팀원, Nav2 게이트): 아래 두 동작 구현 후 trap_installed 발행.
        #   1) beep — TB4 부저로 사람에게 설치 알림 (예: /{ns}/cmd_audio,
        #      irobot_create_msgs/AudioNoteVector — 토픽/타입 확인 필요)
        #   2) 구멍(hole_x,y) 20cm 앞으로 target_pose 발행 → 도착 후 후진해 빠짐
        #      (robot_agent가 주행). 주행 완료를 확인한 뒤 trap_installed 발행.
        self.get_logger().info(
            f'설치 job ({job.hole_x:.2f}, {job.hole_y:.2f}) — TODO: beep + 설치 주행')
        self.event_pub.publish(String(data=fleet_msg.event(
            'trap_installed', job.hole_x, job.hole_y)))


def main():
    rclpy.init()
    node = TrapCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


def _self_check():
    assert trap_ok(0.0, 0.0, 0.1, 0.0, 0.15) is True        # 10cm < 15cm
    assert trap_ok(0.0, 0.0, 0.1, 0.1, 0.15) is True        # hypot=14.1cm < 15cm
    assert trap_ok(0.0, 0.0, 0.2, 0.0, 0.15) is False       # 20cm > 15cm
    assert trap_ok(0.0, 0.0, 0.15, 0.0, 0.15) is True       # 정확히 15cm — 경계 포함
    assert trap_ok(1.0, 1.0, 1.2, 1.2, 0.15) is False       # hypot=28.3cm > 15cm
    print('trap_check self-check ok')


if __name__ == '__main__':
    import sys
    _self_check() if '--check' in sys.argv else main()
