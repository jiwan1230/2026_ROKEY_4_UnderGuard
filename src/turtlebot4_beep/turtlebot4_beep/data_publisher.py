import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class DataPublisher(Node):
    def __init__(self):
        super().__init__('data_publisher')
        self.publisher_ = self.create_publisher(String, 'data_topic', 10)
        self.timer = self.create_timer(0.5, self.publish_data)
        self.get_logger().info('Data Publisher')

    def publish_data(self):
        
        message = "hello"
        self.publisher_.publish(message)
        self.get_logger().info(f'Published data: {message}')
          

def main(args=None):
    rclpy.init(args=args)
    node = DataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
