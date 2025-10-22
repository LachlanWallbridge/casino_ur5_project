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
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, Point, Quaternion
from custom_interface.msg import DiceResult, DiceResults
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

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
        self.default_depth_value = 800.0  # mm

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

        # Publisher for dice results
        self.dice_pub = self.create_publisher(DiceResults, "dice_results", 10)

        # Publicher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, 'dice_markers', 10)

    
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # OpenCV window
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

    # --- Crop helpers ---
    def _crop_board(self, frame):
        board_h_mm, board_w_mm = 390, 756
        lower_half_y = board_h_mm / 2
        marker_size = 100
        x1, y1, x2, y2 = marker_size, 0, board_w_mm - marker_size, int(lower_half_y)
        return frame[y1:y2, x1:x2]

    # -- Dice processing ---
    def _get_cropped_depth(self, frame, depth_msg):
        if self.warp_matrix is None:
            return np.full(frame.shape[:2], self.default_depth_value, dtype=np.float32)
        try:
            if self.use_depth and depth_msg is not None:
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
                warped_depth = cv2.warpPerspective(depth_image, self.warp_matrix, self.warp_size)
                return warped_depth[:frame.shape[0], :frame.shape[1]]
            else:
                return np.full(frame.shape[:2], self.default_depth_value, dtype=np.float32)
        except Exception as e:
            self.get_logger().warn(f"⚠️ Depth warp failed: {e}")
            return np.full(frame.shape[:2], self.default_depth_value, dtype=np.float32)

    # -- Dice processing ---
    def _process_single_dice(self, box, frame, cropped_depth):
        x1_box, y1_box, x2_box, y2_box = map(int, box.xyxy[0])
        cv2.rectangle(frame, (x1_box, y1_box), (x2_box, y2_box), SECONDARY_COLOR, 2)
        label = str(int(box.cls[0]) + 1)

        # Depth
        if cropped_depth is not None:
            roi = cropped_depth[y1_box:y2_box, x1_box:x2_box]
            valid = roi[roi > 0]
            depth_val = np.median(valid)/1000.0 if valid.size > 0 else self.default_depth_value/1000.0
            label += f" ({depth_val:.2f}m)"
        else:
            depth_val = self.default_depth_value / 1000.0

        cv2.putText(frame, label, (x1_box, y1_box - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.75, SECONDARY_COLOR, 2)

        # Transform to world frame
        point_cam = PointStamped()
        point_cam.header.frame_id = 'camera_frame'
        point_cam.header.stamp = self.get_clock().now().to_msg()
        point_cam.point.x = (x1_box + x2_box)/2
        point_cam.point.y = (y1_box + y2_box)/2
        point_cam.point.z = depth_val

        try:
            point_world = self.tf_buffer.transform(point_cam, 'world', timeout=rclpy.duration.Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            point_world = point_cam

        # Create DiceResult
        dr = DiceResult()
        dr.x = x1_box
        dr.y = y1_box
        dr.width = x2_box - x1_box
        dr.height = y2_box - y1_box
        dr.dice_number = int(box.cls[0]) + 1
        dr.confidence = float(box.conf[0])
        dr.pose.position = point_world.point
        dr.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return dr

    # -- Visualization helpers ---
    def _draw_stats(self, frame, dice_count, total_dice_sum):
        img_h, img_w = frame.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        color = (0, 0, 0)
        line_spacing = 20

        text1 = f"Dice Count: {dice_count}"
        (tw1, th1), _ = cv2.getTextSize(text1, font, font_scale, thickness)
        x1_text = img_w - tw1 - 10
        y1_text = 10 + th1

        text2 = f"Total Sum: {total_dice_sum}"
        (tw2, th2), _ = cv2.getTextSize(text2, font, font_scale, thickness)
        x2_text = img_w - tw2 - 10
        y2_text = y1_text + line_spacing

        cv2.putText(frame, text1, (x1_text, y1_text), font, font_scale, color, thickness)
        cv2.putText(frame, text2, (x2_text, y2_text), font, font_scale, color, thickness)

    # -- RViz Marker Publishing ---
    def publish_dice_markers(self, dice_results):
        """
        Publish RViz markers for each detected dice.

        dice_results: list of DiceResult messages with .pose filled in world frame
        """
        marker_array = MarkerArray()

        for i, dr in enumerate(dice_results):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "dice"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Use dice pose
            marker.pose = dr.pose

            # Size of cube in meters
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            # Color (RGBA)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)
        
        self.get_logger().info(f"Publishing {len(dice_results)} dice markers at: {[dr.pose.position for dr in dice_results]}")

        self.marker_pub.publish(marker_array)

    # --- Shared processing logic ---
    def process_frame(self, rgb_msg, depth_msg):
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")

        # Crop to board region
        frame = self._crop_board(frame)

        # Handle depth
        cropped_depth = self._get_cropped_depth(frame, depth_msg)

        # YOLO inference
        results = self.model(frame)[0]
        detected_dice = results.boxes
        dice_count = len(detected_dice)
        total_dice_sum = sum(int(box.cls[0]) + 1 for box in detected_dice)

        # Prepare DiceResults message
        dice_results_msg = DiceResults()

        for box in detected_dice:
            dr = self._process_single_dice(box, frame, cropped_depth)
            dice_results_msg.dice.append(dr)

        # Publish results
        self.dice_pub.publish(dice_results_msg)

        # Publish RViz markers
        self.publish_dice_markers(dice_results_msg.dice)

        # Draw stats
        self._draw_stats(frame, dice_count, total_dice_sum)

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
