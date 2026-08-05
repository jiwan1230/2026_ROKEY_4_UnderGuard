"""고정 웹캠 쥐 감시 — 노트북 웹캠에서 YOLO로 rat 감지.

감지하면 homography로 map 좌표를 구해 /fleet/event(rat_detected:x:y) 발행 →
중앙이 쥐대응 진입. YOLO·homography는 TODO(팀원). 뼈대는 프레임 읽기·발행 배관.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WebcamNode(Node):

    def __init__(self):
        super().__init__('webcam_node')
        self.camera_index = self.declare_parameter('camera_index', 0).value
        self.model_path = self.declare_parameter('model_path', '').value
        self.homography_file = self.declare_parameter('homography_file', '').value

        self.event_pub = self.create_publisher(String, '/fleet/event', 10)
        # TODO(팀원): cv2.VideoCapture(camera_index) 열고, 타이머로 프레임 읽어
        # YOLO 감지 + homography 변환 → rat_detected event 발행.
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'웹캠 노드 시작 — index {self.camera_index}')

    def tick(self):
        # TODO(팀원): 프레임 읽기 → YOLO → homography → event.
        pass


def main():
    rclpy.init()
    node = WebcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
