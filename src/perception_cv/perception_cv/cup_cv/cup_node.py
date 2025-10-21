import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CupDetector(Node):
    def __init__(self):
        super().__init__('cup_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('Cup detector node started.')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # TODO: Add cup centroid detection
        cv2.imshow('Cup', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CupDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
