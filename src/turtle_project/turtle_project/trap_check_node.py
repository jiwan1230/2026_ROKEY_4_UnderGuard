"""trap 설치/점검 — detector가 준 TrapJob으로만 동작 (지각 안 함).

detector가 YOLO를 유일하게 쓰고, 여기는 좌표만 받아 판정/주행한다.
  install: 0.5m로 전진 → 10초 대기(사람이 trap 놓음) → 후진 후 trap_installed 발행
  inspect: 구멍좌표 vs trap좌표 거리 ≤ trap_ok_dist 면 trap_ok, 아니면 trap_bad

설치 주행은 전부 cmd_vel open-loop — 로봇이 구멍을 정면으로 본 채(detector가
0.8m 앞에 세워둠) 0.3m 전진/후진하고, 후진은 Create3가 곧은 후진을 못 해서
180° 회전→전진→180° 복귀로 대신한다. TF·Nav2 안 쓴다.

판정 결과는 /fleet/event로 발행 → detector가 받아 순찰 재개.
"""
import math
import shutil
import subprocess
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
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
        # 구멍은 wall-band depth, trap은 center depth로 재 좌표가 체계적으로
        # 벌어진다(트랩이 벽 앞에 놓임) → 15cm면 정상 설치도 trap_bad. 0.25로
        # 오프셋 흡수. match_dist(0.3)보다 낮게 둬 옆 구멍 트랩까지 ok로 세지 않게.
        self.ok_dist = self.declare_parameter('trap_ok_dist', 0.25).value
        # install 주행 — 전부 cmd_vel open-loop. 시간(sec)이 거리/각도 튜닝 노브.
        # 로봇은 구멍 0.8m 앞에서 시작 → 0.3m 전진(0.5m) → 대기 → 0.3m 후진(0.8m).
        self.fwd_speed = self.declare_parameter('fwd_speed', 0.1).value       # 전/후진 m/s
        self.approach_sec = self.declare_parameter('approach_sec', 3.0).value  # 0.8→0.5m 전진 (≈0.3m)
        self.retreat_sec = self.declare_parameter('retreat_sec', 3.0).value    # 0.5→0.8m 후진 전진분 (≈0.3m)
        # 후진의 180° 회전 — Create3가 곧은 후진(linear.x 음수)을 못 해서 돌아서 감.
        # 두 회전을 반대 방향으로 줘 heading 오차 상쇄. turn_sec은 실측 180°에 맞출 것.
        self.turn_speed = self.declare_parameter('turn_speed', 1.0).value      # rad/s
        self.turn_sec = self.declare_parameter('turn_sec', 3.14).value         # 실측 180° 소요(z=1.0)
        self.install_wait = self.declare_parameter('install_wait', 10.0).value  # 사람이 trap 놓을 시간
        # 후진 180° 회전이 명령상 끝나도 로봇은 감속·정착 중 — 이만큼 더 기다린 뒤
        # trap_installed를 쏴야 detector가 완전히 멈춘 자세에서 trap 위치를 뽑는다.
        self.settle_sec = self.declare_parameter('settle_sec', 1.5).value
        self.beep_hz = self.declare_parameter('beep_hz', 1000).value
        self.beep_sec = self.declare_parameter('beep_sec', 1.0).value

        self.event_pub = self.create_publisher(String, '/fleet/event', 10)
        # 내 로봇 detector 전용 결과 채널 (상대 토픽 → 네임스페이스로 분리).
        # /fleet/event만 쓰면 다른 로봇 detector가 남의 결과로 상태 전이한다.
        self.local_pub = self.create_publisher(String, 'trap_event', 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)   # 설치 주행 open-loop
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
        """trap 설치 — beep로 사람 호출 → 0.5m 전진 → 대기 → 후진 → trap_installed.

        전부 cmd_vel open-loop (blocking). 로봇은 detector가 구멍 0.8m 앞에 세워둔
        상태로 시작하며 구멍을 정면으로 본다. 0.3m 전진해 0.5m에서 사람이 trap을
        놓고, 0.3m 후진해 0.8m에서 detector가 재점검한다. 새 구멍 최초 설치·재설치
        공통. blocking 동안(~24s) detector는 AWAIT_TRAP으로 기다린다.
        """
        hx, hy = job.hole_x, job.hole_y
        self.get_logger().info(f'설치 job ({hx:.2f}, {hy:.2f}) — beep + 0.5m 전진')
        self._beep()
        self._drive_fwd(self.approach_sec)                  # 0.8→0.5m 전진
        self.get_logger().info(f'0.5m 도착 — {self.install_wait:.0f}초 설치 대기')
        time.sleep(self.install_wait)                       # 사람이 trap 놓음
        self._retreat()                                     # 0.5→0.8m 후진
        time.sleep(self.settle_sec)                         # 회전 완전 정지·정착 후 판단
        msg = String(data=fleet_msg.event('trap_installed', hx, hy))
        self.event_pub.publish(msg)     # 관찰용 (전역)
        self.local_pub.publish(msg)     # 내 detector 상태 전이용 (로봇별)

    def _retreat(self):
        """후진 = 180° 회전 → 전진 → 180° 복귀 (Create3가 곧은 후진을 못 함).
        두 회전을 반대 방향으로 줘서 heading 오차가 상쇄되고, 끝나면 다시 구멍을 본다."""
        self._spin(self.turn_speed, self.turn_sec)      # 180° 돌기
        self._drive_fwd(self.retreat_sec)               # 뒤로(=현 정면) 이동
        self._spin(-self.turn_speed, self.turn_sec)     # 180° 복귀 → 구멍 재조준
        self.get_logger().info('후진 완료 (회전-전진-회전) — trap_installed')

    def _spin(self, w, sec):
        tw = Twist()
        tw.angular.z = float(w)
        self._pub_for(tw, sec)

    def _drive_fwd(self, sec):
        tw = Twist()
        tw.linear.x = abs(float(self.fwd_speed))
        self._pub_for(tw, sec)

    def _pub_for(self, tw, sec):
        """tw를 10Hz로 sec초간 발행 후 정지 (cmd_vel watchdog 유지용 반복 발행)."""
        for _ in range(int(sec * 10)):
            self.cmd_pub.publish(tw)
            time.sleep(0.1)
        self.cmd_pub.publish(Twist())

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
