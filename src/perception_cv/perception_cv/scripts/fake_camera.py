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

        # Declare a ROS 2 parameter for the image path
        self.declare_parameter('image_path', '/home/mtrn/MTRN4231_Project/images/contentimage_5F00_203314.png')
        image_path = self.get_parameter('image_path').get_parameter_value().string_value

        if not os.path.exists(image_path):
            self.get_logger().error(f"Image file does not exist: {image_path}")
            raise FileNotFoundError(f"Image file does not exist: {image_path}")

        self.frame = cv2.imread(image_path)
        self.pub = self.create_publisher(Image, '/camera/camera/color/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.5, self.publish_image)  # 2 Hz

        self.get_logger().info(f"Publishing image: {image_path}")

    def publish_image(self):
        msg = self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8')
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
