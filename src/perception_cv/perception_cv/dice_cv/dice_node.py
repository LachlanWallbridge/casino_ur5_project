import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
from std_msgs.msg import Float64MultiArray, Int32MultiArray
from message_filters import Subscriber, ApproximateTimeSynchronizer

WINDOW_NAME = "Dice Recognition"
PRIMARY_COLOR = (0, 255, 0)
SECONDARY_COLOR = (0, 0, 255)

class DiceDetector(Node):
    def __init__(self):
        super().__init__("dice_detector")
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter("depth", False)
        self.use_depth = self.get_parameter("depth").get_parameter_value().bool_value
        self.default_depth_value = 800.0  # mm (0.8 m)

        # YOLO model load
        package_share_dir = get_package_share_directory('perception_cv')
        model_path = os.path.join(package_share_dir, 'dice_cv', 'weights', 'best.pt')
        self.model = YOLO(model_path)

        # --- Internal buffers ---
        self.warp_matrix = None
        self.warp_size = (0, 0)

        # --- Subscribers ---
        self.rgb_sub = Subscriber(self, Image, "board/warped_image")

        if self.use_depth:
            self.depth_sub = Subscriber(self, Image, "/camera/depth/image_rect_raw")

            self.ts = ApproximateTimeSynchronizer(
                [self.rgb_sub, self.depth_sub],
                queue_size=10,
                slop=0.05
            )
            self.ts.registerCallback(self.synced_callback)
        else:
            # Only RGB, no sync needed
            self.rgb_sub = self.create_subscription(
                Image, "board/warped_image", self.image_callback, 10
            )

        # Warp metadata
        self.warp_matrix_sub = self.create_subscription(
            Float64MultiArray, "board/warp_matrix", self.warp_matrix_callback, 10
        )
        self.warp_size_sub = self.create_subscription(
            Int32MultiArray, "board/warp_size", self.warp_size_callback, 10
        )

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.get_logger().info(f"🎲 Dice detector node started (depth={self.use_depth}).")

    # --- Metadata callbacks ---
    def warp_matrix_callback(self, msg):
        self.warp_matrix = np.array(msg.data, dtype=np.float64).reshape((3, 3))

    def warp_size_callback(self, msg):
        self.warp_size = tuple(msg.data)

    # --- Callbacks ---
    def image_callback(self, rgb_msg):
        self.process_frame(rgb_msg, None)

    def synced_callback(self, rgb_msg, depth_msg):
        self.process_frame(rgb_msg, depth_msg)

    # --- Shared processing logic ---
    def process_frame(self, rgb_msg, depth_msg):
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")

        # Crop to region of interest
        board_h_mm, board_w_mm = 390, 756
        lower_half_y = board_h_mm / 2
        marker_size = 100
        x1, y1, x2, y2 = marker_size, 0, board_w_mm - marker_size, int(lower_half_y)
        frame = frame[y1:y2, x1:x2]

        # Depth handling
        cropped_depth = None
        if self.warp_matrix is not None:
            try:
                if self.use_depth and depth_msg is not None:
                    depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
                    warped_depth = cv2.warpPerspective(depth_image, self.warp_matrix, self.warp_size)
                    cropped_depth = warped_depth[y1:y2, x1:x2]
                else:
                    # Fake constant depth (same shape as RGB)
                    cropped_depth = np.full(frame.shape[:2], self.default_depth_value, dtype=np.float32)
            except Exception as e:
                self.get_logger().warn(f"⚠️ Depth warp failed: {e}")
                cropped_depth = np.full(frame.shape[:2], self.default_depth_value, dtype=np.float32)

        # --- YOLO inference ---
        results = self.model(frame)[0]
        detected_dice = results.boxes
        dice_count = len(detected_dice)
        total_dice_sum = sum(int(box.cls[0]) + 1 for box in detected_dice)

        for box in detected_dice:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), SECONDARY_COLOR, 2)
            label = str(int(box.cls[0]) + 1)

            # Depth annotation
            if cropped_depth is not None:
                roi = cropped_depth[y1:y2, x1:x2]
                valid = roi[roi > 0]
                if valid.size > 0:
                    depth_val = np.median(valid)
                else:
                    depth_val = self.default_depth_value
                label += f" ({depth_val/1000:.2f}m)"

            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, SECONDARY_COLOR, 2)

        # Get image width
        img_h, img_w = frame.shape[:2]

        # Text settings
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        color = (0, 0, 0)  # black
        line_spacing = 20   # vertical spacing between lines

        # Dice Count (top right)
        text1 = f"Dice Count: {dice_count}"
        (text_width1, text_height1), _ = cv2.getTextSize(text1, font, font_scale, thickness)
        x1 = img_w - text_width1 - 10
        y1 = 10 + text_height1
        cv2.putText(frame, text1, (x1, y1), font, font_scale, color, thickness)

        # Total Sum (below Dice Count)
        text2 = f"Total Sum: {total_dice_sum}"
        (text_width2, text_height2), _ = cv2.getTextSize(text2, font, font_scale, thickness)
        x2 = img_w - text_width2 - 10
        y2 = y1 + line_spacing
        cv2.putText(frame, text2, (x2, y2), font, font_scale, color, thickness)

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
