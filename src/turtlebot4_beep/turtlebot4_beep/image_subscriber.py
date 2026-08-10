import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CamSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera_image',
            self.image_callback,
            10
        )
        self.subscription  
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info('Received image')
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        cv2.imshow('Received Image', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()  

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CamSubscriber()

    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()  # Close OpenCV windows
        rclpy.shutdown()

if __name__ == '__main__':
    main()
