"""trap 설치/점검 — detector가 준 TrapJob으로만 동작 (지각 안 함).

detector가 YOLO를 유일하게 쓰고, 여기는 좌표만 받아 판정/주행한다.
  install: 구멍 20cm 앞 전진→후진 주행 후 trap_installed 발행 (beep로 사람 호출)
  inspect: 구멍좌표 vs trap좌표 거리 ≤ trap_ok_dist 면 trap_ok, 아니면 trap_bad

판정 결과는 /fleet/event로 발행 → detector가 받아 순찰 재개.
"""
import math
import shutil
import subprocess
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from turtle_interfaces.msg import TrapJob
from turtle_project import fleet_msg
from turtle_project.nav_controller import make_pose

FRAME = 'map'


def trap_ok(hole_x, hole_y, trap_x, trap_y, ok_dist):
    """구멍↔trap 거리가 ok_dist 이내면 True (설치 정상)."""
    return math.hypot(hole_x - trap_x, hole_y - trap_y) <= ok_dist


class TrapCheckNode(Node):

    def __init__(self):
        super().__init__('trap_check_node')
        self.ok_dist = self.declare_parameter('trap_ok_dist', 0.15).value
        # install: 구멍 앞 접근 거리 / 후진 거리(둘 다 m).
        self.approach_dist = self.declare_parameter('approach_dist', 0.20).value
        self.retreat_dist = self.declare_parameter('retreat_dist', 0.6).value
        # install: 각 이동 뒤 대기 시간(초) — Nav2 도착 피드백이 여기로 안 와서 고정 시간으로 근사.
        self.install_wait = self.declare_parameter('install_wait', 4.0).value  # 접근 후 사람이 trap 놓을 시간
        self.retreat_wait = self.declare_parameter('retreat_wait', 2.0).value  # 후진 후 정착 시간
        self.beep_hz = self.declare_parameter('beep_hz', 1000).value
        self.beep_sec = self.declare_parameter('beep_sec', 1.0).value

        self.tf = Buffer()
        TransformListener(self.tf, self)

        self.event_pub = self.create_publisher(String, '/fleet/event', 10)
        # 내 로봇 detector 전용 결과 채널 (상대 토픽 → 네임스페이스로 분리).
        # /fleet/event만 쓰면 다른 로봇 detector가 남의 결과로 상태 전이한다.
        self.local_pub = self.create_publisher(String, 'trap_event', 10)
        # 실제 주행은 robot_agent가 한다 — 여기선 목표 좌표만 발행.
        self.pose_pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.audio_pub = self.create_publisher(AudioNoteVector, 'cmd_audio', 10)
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
        msg = String(data=fleet_msg.event(name, job.hole_x, job.hole_y))
        self.event_pub.publish(msg)     # db 저장·central 관찰용 (전역)
        self.local_pub.publish(msg)     # 내 detector 상태 전이용 (로봇별)

    def _install(self, job):
        """trap 설치 동작 — beep로 사람 호출 + 구멍 20cm 앞 전진→후진 주행.

        새 구멍 최초 설치와 재설치(trap_bad/미검출) 공통. 사람이 이 사이 trap을
        놓는다. 동작이 끝나면 trap_installed 발행 → detector가 15cm 재점검한다.
        """
        hx, hy = job.hole_x, job.hole_y
        self.get_logger().info(f'설치 job ({hx:.2f}, {hy:.2f}) — beep + 설치 주행')
        self._beep()
        self._drive_to(hx, hy, self.approach_dist)
        time.sleep(self.install_wait)
        self._drive_to(hx, hy, self.retreat_dist)
        time.sleep(self.retreat_wait)
        msg = String(data=fleet_msg.event('trap_installed', hx, hy))
        self.event_pub.publish(msg)     # 관찰용 (전역)
        self.local_pub.publish(msg)     # 내 detector 상태 전이용 (로봇별)

    def _drive_to(self, hx, hy, dist):
        """hole(hx,hy)과 로봇 현재 위치를 잇는 선 위에서, hole로부터 dist
        떨어진 지점으로 target_pose 발행. dist가 현재 거리보다 크면(=후진
        상황) 로봇 뒤쪽 지점이 나온다 — 접근·후진 둘 다 이 한 함수로 처리."""
        robot = self._robot_xy()
        if robot is None:
            return
        rx, ry = robot
        dx, dy = rx - hx, ry - hy   # hole -> robot 벡터
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return               # 로봇이 hole과 같은 위치 — 방향 정의 불가
        ux, uy = dx / d, dy / d
        px, py = hx + ux * dist, hy + uy * dist
        yaw = math.atan2(hy - py, hx - px)   # hole을 바라보는 방향
        self.pose_pub.publish(make_pose(FRAME, px, py, yaw))

    def _robot_xy(self):
        try:
            t = self.tf.lookup_transform(
                FRAME, 'base_link', rclpy.time.Time()).transform.translation
            return t.x, t.y
        except TransformException as e:
            self.get_logger().warn(f'TF 실패: {e}', throttle_duration_sec=5.0)
            return None

    def _beep(self):
        """TB4 부저(cmd_audio) + PC 스피커(speaker-test, 하드웨어 무관 폴백)."""
        note = AudioNote(frequency=self.beep_hz,
                         max_runtime=Duration(sec=int(self.beep_sec)))
        self.audio_pub.publish(AudioNoteVector(notes=[note], append=False))
        if shutil.which('speaker-test') is None:
            time.sleep(self.beep_sec)
            return
        try:
            subprocess.run(
                ['speaker-test', '-t', 'sine', '-f', str(self.beep_hz), '-l', '1'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=self.beep_sec)
        except subprocess.TimeoutExpired:
            pass


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
