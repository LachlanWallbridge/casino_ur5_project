#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import os

class FakeCamera(Node):
    def __init__(self):
        super().__init__('fake_camera')

        # Declare ROS 2 parameters
        self.declare_parameter('image_path', '')
        self.declare_parameter('topic', '/camera/camera/color/image_raw')

        image_path = self.get_parameter('image_path').get_parameter_value().string_value
        topic = self.get_parameter('topic').get_parameter_value().string_value

        # Validate image path
        if not os.path.exists(image_path):
            self.get_logger().error(f"Image file does not exist: {image_path}")
            raise FileNotFoundError(f"Image file does not exist: {image_path}")

        # Load image and setup publisher
        self.frame = cv2.imread(image_path)
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, topic, 10)
        self.timer = self.create_timer(0.5, self.publish_image)  # 2 Hz

        self.get_logger().info(f"Publishing image '{image_path}' to topic '{topic}'")

    def publish_image(self):
        msg = self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8')
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
