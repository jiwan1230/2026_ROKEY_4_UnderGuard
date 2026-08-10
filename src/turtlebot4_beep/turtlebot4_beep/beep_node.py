import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from irobot_create_msgs.msg import AudioNote, AudioNoteVector


class BeepNode(Node):
    def __init__(self):
        super().__init__('beep_node')
        self.pub = self.create_publisher(AudioNoteVector, 'robot4/cmd_audio', 10)

    def beep(self):
        # 삐-뽀-삐-뽀
        msg = AudioNoteVector()
        msg.append = False
        msg.notes = [
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300000000)),  # 삐
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300000000)),  # 뽀
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300000000)),  # 삐
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300000000)),  # 뽀
        ]
        self.pub.publish(msg)
        self.get_logger().info('beep!')



def main():
    rclpy.init()
    node = BeepNode()

    # 구독자(로봇)가 잡힐 때까지 최대 5초 대기 후 한 번 publish
    for _ in range(50):
        if node.pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    node.beep()
    rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
