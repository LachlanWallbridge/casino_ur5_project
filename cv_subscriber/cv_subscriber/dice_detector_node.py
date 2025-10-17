import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os

WINDOW_NAME = "Dice Recognition"
PRIMARY_COLOR = (0, 255, 0)
SECONDARY_COLOR = (0, 0, 255)


class DiceDetector(Node):
    def __init__(self):
        super().__init__("dice_detector")
        self.bridge = CvBridge()

        # Load YOLO model
        # self.model = YOLO("runs/detect/train/weights/best.pt")
        package_share_dir = get_package_share_directory('cv_subscriber')
        model_path = os.path.join(package_share_dir, 'weights', 'best.pt')
        self.model = YOLO(model_path)

        # Subscribe to ROS 2 image topic
        self.subscription = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",  # Change this topic name if needed
            self.image_callback,
            10,
        )

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.get_logger().info("Dice detector node started. Listening to camera...")

    def image_callback(self, msg):
        # Convert ROS Image message → OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Crop region using numpy slicing: frame[y1:y2, x1:x2]
        x1, y1 = 300, 100
        x2, y2 = 1200, 500
        cropped_frame = frame[y1:y2, x1:x2]
        frame = cropped_frame       

        # Run YOLO inference
        detected_dice = self.model(frame)[0].boxes
        dice_count = len(detected_dice)
        total_dice_sum = sum(int(box.cls[0]) + 1 for box in detected_dice)

        for box in detected_dice:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), SECONDARY_COLOR, 2)
            cv2.putText(
                frame,
                str(int(box.cls[0]) + 1),
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                SECONDARY_COLOR,
                2,
            )

        cv2.putText(frame, f"Dice Count: {dice_count}", (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, PRIMARY_COLOR, 2)
        cv2.putText(frame, f"Total Sum: {total_dice_sum}", (30, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, PRIMARY_COLOR, 2)

        # Display image
        cv2.imshow(WINDOW_NAME, frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DiceDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
