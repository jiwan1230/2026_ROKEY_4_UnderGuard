"""쥐몰이 배관 — fleet 프로토콜 ↔ herding_controller 패키지의 HerdingNode를 잇는다.

몰이 알고리즘 자체(HerdingCore)는 여기 없다 — `herding_controller`(플랜 A)
또는 `herding_controller_dual`(플랜 B) 패키지의 `herding_node`(별도 프로세스로
실행)가 담당한다. 이 노드는 그 노드가 원하는 입력(PoseStamped 3종:
target_pose/robot1_pose/robot2_pose)을 fleet 쪽 데이터(String 프로토콜 + TF)
로부터 만들어 공급하고, 출력(robot1_goal/robot2_goal)을 fleet가 이해하는
`{robot}/target_pose`로 relay하는 순수 배관 역할만 한다.

로봇A(추적)/로봇B(몰이)는 고정이 아니라 central이 그때그때 배정하므로,
/fleet/command의 TRACK/HERD 수신으로 동적으로 추적한다.

플랜 A/B 공용이다 (2026-08-09):
  플랜 A(herding_controller)      robot2_goal만 발행 -> 로봇 B만 우리가 조종
  플랜 B(herding_controller_dual) 둘 다 발행         -> 로봇 A, B 모두 조종
robot1_goal을 구독해도 플랜 A에서는 아무도 발행하지 않아 콜백이 아예 돌지
않는다. 그래서 어느 쪽을 띄우든 이 파일을 고칠 필요가 없다 -- 단, 두 패키지의
노드 이름을 똑같이 'herding_controller'로 띄워야 토픽 경로가 일치한다.

주의(플랜 B 전용): 플랜 B에서는 로봇 A의 target_pose를 detector_node도
쓴다(추적 goal을 rat_goal_period마다 발행). 같은 토픽에 둘이 쏘면 로봇이
진동하므로, 로봇 A의 detector를 `-p plan:=b`로 띄워 추적 goal 발행을 꺼야
한다 (detector에 훅 반영 완료 — 2026-08-10).

주의: 로봇 pose는 fleet_msg 프로토콜에 없어서 각 로봇의 amcl_pose(map
프레임)를 구독해 최신값을 재발행한다. TF 조회 방식은 이 노드가 도는 중앙
PC에서 로봇 TF(네임스페이스 토픽 /robotN/tf)가 아예 안 보여 폐기했다
(2026-08-10). amcl_pose는 로봇이 움직일 때만 갱신되지만, 정지 중엔 마지막
값이 그대로 유효하므로 문제없다.
"""
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

from turtle_project import fleet_msg

MAP_FRAME = 'map'


class RatHerdingNode(Node):

    def __init__(self):
        super().__init__('rat_herding_node')
        control_rate_hz = self.declare_parameter('control_rate_hz', 5.0).value

        self.robot_a = None      # TRACK 명령 받은 로봇 (추적 = Driver)
        self.robot_b = None      # HERD 명령 받은 로봇 (몰이 = Blocker)
        # 각 로봇이 정해지면 그 로봇의 target_pose로 생성한다. 플랜 A에서는
        # robot1_goal이 아예 안 오므로 relay_a는 만들어져도 쓰이지 않는다.
        self.relay_a = None
        self.relay_b = None
        self.pose_a = None       # 각 로봇의 마지막 amcl_pose (PoseStamped)
        self.pose_b = None
        self.sub_a = None        # 로봇별 amcl_pose 구독 — 배정 시 생성
        self.sub_b = None

        # herding_controller의 HerdingNode가 구독하는 입력 3종 (~/target_pose 등,
        # 이 노드가 'herding_controller' 네임스페이스로 리맵돼 실행된다고 가정).
        self.target_pub = self.create_publisher(PoseStamped, 'herding_controller/target_pose', 10)
        self.robot1_pose_pub = self.create_publisher(PoseStamped, 'herding_controller/robot1_pose', 10)
        self.robot2_pose_pub = self.create_publisher(PoseStamped, 'herding_controller/robot2_pose', 10)

        self.create_subscription(String, '/fleet/command', self.command_cb, 10)
        self.create_subscription(String, '/fleet/event', self.event_cb, 10)
        self.create_subscription(
            PoseStamped, 'herding_controller/robot1_goal', self.goal1_cb, 10)
        self.create_subscription(
            PoseStamped, 'herding_controller/robot2_goal', self.goal2_cb, 10)
        self.create_timer(1.0 / control_rate_hz, self._publish_robot_poses)
        self.get_logger().info('쥐몰이 배관 노드 시작 — TRACK/HERD 대상 대기')

    def command_cb(self, msg):
        """TRACK 대상 = 로봇A(Driver), HERD 대상 = 로봇B(Blocker)."""
        robot, cmd = fleet_msg.parse_command(msg.data)
        if cmd == 'TRACK' and robot != self.robot_a:
            self.robot_a = robot
            self.relay_a = self.create_publisher(
                PoseStamped, f'{robot}/target_pose', 10)
            self.sub_a = self._resub(self.sub_a, robot, self._pose_a_cb)
            self.get_logger().info(f'robot A(추적) = {robot} — goal relay 대상')
        elif cmd == 'HERD' and robot != self.robot_b:
            self.robot_b = robot
            self.relay_b = self.create_publisher(
                PoseStamped, f'{robot}/target_pose', 10)
            self.sub_b = self._resub(self.sub_b, robot, self._pose_b_cb)
            self.get_logger().info(f'robot B(몰이) = {robot} — goal relay 대상')

    def event_cb(self, msg):
        """쥐 위치를 herding_controller가 원하는 target_pose로 그대로 변환·발행."""
        name, x, y = fleet_msg.parse_event(msg.data)
        if name != 'rat_detected':
            return
        self.target_pub.publish(self._make_pose(x, y))

    def goal1_cb(self, msg):
        """robot1_goal → 로봇A의 target_pose로 relay (플랜 B에서만 들어온다).

        플랜 A(herding_controller)는 robot1_goal을 발행하지 않으므로 이
        콜백은 아예 돌지 않는다. 로봇 A는 그때 상위 시스템(detector_node)이
        몬다.
        """
        if self.relay_a is None:
            return               # A 미배정 (TRACK 전이거나 순찰중)
        self.relay_a.publish(msg)

    def goal2_cb(self, msg):
        """robot2_goal → 로봇B의 target_pose로 relay (플랜 A/B 공통)."""
        if self.relay_b is None:
            return               # B 미배정 (HERD 전이거나 순찰중)
        self.relay_b.publish(msg)

    def _publish_robot_poses(self):
        """저장된 최신 amcl_pose를 herding 입력으로 재발행 (control_rate_hz)."""
        if self.pose_a is not None:
            self.robot1_pose_pub.publish(self.pose_a)
        if self.pose_b is not None:
            self.robot2_pose_pub.publish(self.pose_b)

    def _resub(self, old, robot, cb):
        """로봇 (재)배정 — 이전 구독 정리 후 그 로봇의 amcl_pose 구독."""
        if old is not None:
            self.destroy_subscription(old)
        return self.create_subscription(
            PoseWithCovarianceStamped, f'{robot}/amcl_pose', cb, 10)

    def _pose_a_cb(self, msg):
        self.pose_a = self._to_pose_stamped(msg)

    def _pose_b_cb(self, msg):
        self.pose_b = self._to_pose_stamped(msg)

    @staticmethod
    def _to_pose_stamped(msg):
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose.pose
        return out

    def _make_pose(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        return msg


def main():
    rclpy.init()
    node = RatHerdingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
