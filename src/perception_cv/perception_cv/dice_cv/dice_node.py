#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os
from custom_interface.msg import DiceResult, DiceResults
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs

# ✅ Import shared coordinate utilities
from perception_cv import pixel_to_world_pose

WINDOW_NAME = "Dice Recognition"
SECONDARY_COLOR = (0, 0, 255)

DICE_HALF_HEIGHT = 0.03  # 30 mm height offset for board frame

class DiceDetector(Node):
    def __init__(self):
        super().__init__("dice_detector")
        self.bridge = CvBridge()

        # --- YOLO model load ---
        package_share_dir = get_package_share_directory('perception_cv')
        model_path = os.path.join(package_share_dir, 'dice_cv', 'weights', 'best.pt')
        self.model = YOLO(model_path)

        # --- Subscriber ---
        self.image_sub = self.create_subscription(Image, "board/warped_image", self.image_callback, 10)

        # --- Publishers ---
        self.dice_pub = self.create_publisher(DiceResults, "dice_results", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "dice_markers", 10)

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.get_logger().info("🎲 Dice detector node started (board frame mode).")

    # --- Image callback ---
    def image_callback(self, rgb_msg):
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        img_h, img_w = frame.shape[:2]

        # YOLO inference
        results = self.model(frame)[0]
        detected_dice = results.boxes
        dice_count = len(detected_dice)
        total_sum = sum(int(box.cls[0]) + 1 for box in detected_dice)

        dice_results_msg = DiceResults()
        dice_list = []

        for box in detected_dice:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2

            self.get_logger().info(f"Detected dice {int(box.cls[0]) + 1} at pixel ({cx}, {cy})")

            # Use shared transform utility
            point_world = pixel_to_world_pose(
                self.tf_buffer,
                x_px=cx,
                y_px=cy,
                z_offset=DICE_HALF_HEIGHT,
                img_w=img_w,
                img_h=img_h,
                node=self
            )

            self.get_logger().info(f"Dice world position: ({point_world.point.x:.3f}, {point_world.point.y:.3f}, {point_world.point.z:.3f})")

            # Draw on image
            cv2.rectangle(frame, (x1, y1), (x2, y2), SECONDARY_COLOR, 2)
            label = f"{int(box.cls[0]) + 1}"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.75, SECONDARY_COLOR, 2)

            # Build DiceResult
            dr = DiceResult()
            dr.x = x1
            dr.y = y1
            dr.width = x2 - x1
            dr.height = y2 - y1
            dr.dice_number = int(box.cls[0]) + 1
            dr.confidence = float(box.conf[0])
            dr.pose.position = point_world.point
            dr.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            dice_list.append(dr)

        dice_results_msg.dice = dice_list
        self.dice_pub.publish(dice_results_msg)
        self.publish_dice_markers(dice_list)
        self._draw_stats(frame, dice_count, total_sum)

        cv2.imshow(WINDOW_NAME, frame)
        cv2.waitKey(1)

    # --- Visualization helpers ---
    def _draw_stats(self, frame, dice_count, total_sum):
        img_h, img_w = frame.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        text1 = f"Dice Count: {dice_count}"
        text2 = f"Total Sum: {total_sum}"
        cv2.putText(frame, text1, (10, img_h - 40), font, 0.6, (0, 0, 0), 2)
        cv2.putText(frame, text2, (10, img_h - 20), font, 0.6, (0, 0, 0), 2)

    # --- RViz Markers ---
    def publish_dice_markers(self, dice_results):
        marker_array = MarkerArray()
        for i, dr in enumerate(dice_results):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "dice"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = dr.pose
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03
            marker.color.r = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


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
