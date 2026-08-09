"""쥐몰이 배관 — fleet 프로토콜 ↔ herding_controller 패키지의 HerdingNode를 잇는다.

몰이 알고리즘 자체(HerdingCore)는 여기 없다 — `herding_controller` 패키지의
`herding_node`(별도 프로세스로 실행)가 담당한다. 이 노드는 그 노드가 원하는
입력(PoseStamped 3종: target_pose/robot1_pose/robot2_pose)을 fleet 쪽 데이터
(String 프로토콜 + TF)로부터 만들어 공급하고, 출력(robot2_goal)을 fleet가
이해하는 `{robot}/target_pose`로 relay하는 순수 배관 역할만 한다.

로봇A(추적)/로봇B(몰이)는 고정이 아니라 central이 그때그때 배정하므로,
/fleet/command의 TRACK/HERD 수신으로 동적으로 추적한다.

주의: 로봇 pose는 fleet_msg 프로토콜에 없어서 TF에서 직접 조회한다
(detector_node.py::robot_xy()와 같은 패턴). map 프레임 기준 각 로봇의
base_link 프레임 이름은 `robot_frame_template` 파라미터로 뺐다 — 기본값
`{robot}/base_link`은 이 저장소의 네임스페이스 관례(`{ns}/...`)를 따른
가정이며, 실제 Nav2/AMCL launch 설정과 다를 수 있으니 실기 연동 전에
반드시 팀원과 확인해야 한다.
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from turtle_project import fleet_msg

MAP_FRAME = 'map'


class RatHerdingNode(Node):

    def __init__(self):
        super().__init__('rat_herding_node')
        self.robot_frame_template = self.declare_parameter(
            'robot_frame_template', '{robot}/base_link').value
        control_rate_hz = self.declare_parameter('control_rate_hz', 5.0).value

        self.robot_a = None      # TRACK 명령 받은 로봇 (추적 = Driver)
        self.robot_b = None      # HERD 명령 받은 로봇 (몰이 = Blocker)
        self.goal_relay_pub = None  # robot_b 정해지면 그 로봇의 target_pose로 생성

        self.tf = Buffer()
        TransformListener(self.tf, self)

        # herding_controller의 HerdingNode가 구독하는 입력 3종 (~/target_pose 등,
        # 이 노드가 'herding_controller' 네임스페이스로 리맵돼 실행된다고 가정).
        self.target_pub = self.create_publisher(PoseStamped, 'herding_controller/target_pose', 10)
        self.robot1_pose_pub = self.create_publisher(PoseStamped, 'herding_controller/robot1_pose', 10)
        self.robot2_pose_pub = self.create_publisher(PoseStamped, 'herding_controller/robot2_pose', 10)

        self.create_subscription(String, '/fleet/command', self.command_cb, 10)
        self.create_subscription(String, '/fleet/event', self.event_cb, 10)
        self.create_subscription(
            PoseStamped, 'herding_controller/robot2_goal', self.goal_cb, 10)
        self.create_timer(1.0 / control_rate_hz, self._publish_robot_poses)
        self.get_logger().info('쥐몰이 배관 노드 시작 — TRACK/HERD 대상 대기')

    def command_cb(self, msg):
        """TRACK 대상 = 로봇A(Driver), HERD 대상 = 로봇B(Blocker)."""
        robot, cmd = fleet_msg.parse_command(msg.data)
        if cmd == 'TRACK' and robot != self.robot_a:
            self.robot_a = robot
            self.get_logger().info(f'robot A(추적) = {robot}')
        elif cmd == 'HERD' and robot != self.robot_b:
            self.robot_b = robot
            self.goal_relay_pub = self.create_publisher(
                PoseStamped, f'{robot}/target_pose', 10)
            self.get_logger().info(f'robot B(몰이) = {robot} — goal relay 대상')

    def event_cb(self, msg):
        """쥐 위치를 herding_controller가 원하는 target_pose로 그대로 변환·발행."""
        name, x, y = fleet_msg.parse_event(msg.data)
        if name != 'rat_detected':
            return
        self.target_pub.publish(self._make_pose(x, y))

    def goal_cb(self, msg):
        """HerdingCore가 계산한 robot2_goal → 실제 로봇B의 target_pose로 relay."""
        if self.goal_relay_pub is None:
            return               # B 미배정 시 발행 대상 없음 (HERD 전이거나 순찰중)
        self.goal_relay_pub.publish(msg)

    def _publish_robot_poses(self):
        """robot_a/robot_b가 정해져 있으면 TF에서 각자 위치를 조회해 발행."""
        if self.robot_a is not None:
            pose = self._lookup_robot_pose(self.robot_a)
            if pose is not None:
                self.robot1_pose_pub.publish(pose)
        if self.robot_b is not None:
            pose = self._lookup_robot_pose(self.robot_b)
            if pose is not None:
                self.robot2_pose_pub.publish(pose)

    def _lookup_robot_pose(self, robot):
        frame = self.robot_frame_template.format(robot=robot)
        try:
            t = self.tf.lookup_transform(MAP_FRAME, frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f'TF 실패 ({frame}): {e}', throttle_duration_sec=5.0)
            return None
        msg = PoseStamped()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.orientation = t.transform.rotation
        return msg

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
