#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os
import math
from tf_transformations import quaternion_from_euler
from tf_transformations import quaternion_multiply
from custom_interface.msg import DiceResult, DiceResults
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.time import Time
import tf2_ros
from geometry_msgs.msg import PoseStamped
import rclpy.time
import numpy as np


from perception_cv import pixel_to_board_coords, pixel_to_world_pose, visualise_pose_in_rviz, BOARD_W_MM, BOARD_H_MM, PIXEL_TO_METERS

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
        self.image_sub = self.create_subscription(
            Image,
            "board/warped_image",
            self.image_callback,
            10
        )

        # --- Publishers ---
        self.dice_pub = self.create_publisher(DiceResults, "dice_results", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "dice_markers", 10)
        self.ee_marker_pub = self.create_publisher(MarkerArray, "dice_ee_goal_markers", 10)

        # -- Buffer --
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- OpenCV window ---
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.gui_timer = self.create_timer(0.1, self.gui_tick) # 10 Hz

        self.get_logger().info("🎲 Dice detector node started (cropped top-half mode + rotation).")

    def gui_tick(self):
        """
        This fires even when NO new image is received.
        It processes OpenCV GUI events so VS Code doesn't freeze.
        """
        cv2.waitKey(1)

    # ----------------------------------------------------------
    def image_callback(self, rgb_msg):
        frame_full = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        img_h, img_w = frame_full.shape[:2]

        # --- Define crop window ---
        pad = 100  # px padding on left/right
        x_start = pad
        x_end = img_w - pad
        y_start = 0
        y_end = img_h // 2  # top half only

        cropped = frame_full[y_start:y_end, x_start:x_end]

        # --- YOLO inference on cropped region ---
        results = self.model(cropped, verbose=False)[0]
        detected_dice = results.boxes
        dice_count = len(detected_dice)
        total_sum = sum(int(box.cls[0]) + 1 for box in detected_dice)

        dice_results_msg = DiceResults()
        dice_list_world = []  # published
        dice_list_board = []  # visualised
        viz_poses = []        # PoseStamped list for RViz axes

        for box in detected_dice:
            # YOLO outputs are relative to cropped region → convert back to full image coords
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            x1 += x_start
            x2 += x_start
            y1 += y_start
            y2 += y_start
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2

            dice_num = int(box.cls[0]) + 1
            conf = float(box.conf[0])

            self.get_logger().info(f"Detected dice {dice_num} at cropped pixel ({cx:.1f}, {cy:.1f})")

            # --- Estimate rotation using minAreaRect ---
            dice_crop = frame_full[y1:y2, x1:x2]
            yaw_rad = 0.0  # default if contour fails
            try:
                gray = cv2.cvtColor(dice_crop, cv2.COLOR_BGR2GRAY)
                _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    cnt = max(contours, key=cv2.contourArea)
                    rect = cv2.minAreaRect(cnt)
                    (_, _), (w, h), angle = rect
                    # OpenCV angle is clockwise from x-axis
                    yaw_rad = -math.radians(angle)
            except Exception as e:
                self.get_logger().warn(f"Rotation estimation failed: {e}")


            # --- Draw rotated bounding box + dice number on the display frame ---
            bbox_w = x2 - x1
            bbox_h = y2 - y1

            rot_pts = self.draw_rotated_bbox(
                frame_full,
                cx=cx,
                cy=cy,
                w=bbox_w,
                h=bbox_h,
                yaw_rad=-yaw_rad,
                color=(0, 0, 255),
                thickness=2
            )

            # Dice number drawn at centre
            cv2.putText(
                frame_full,
                (str(dice_num)+str(conf)[1:4]),
                (int(cx), int(cy)),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )


            # --- Convert pixel → board coordinates directly (full image space) ---
            x_m, y_m, z_m = pixel_to_board_coords(
                x_px=cx,
                y_px=cy,
                img_w=img_w,
                img_h=img_h,
                z_offset=DICE_HALF_HEIGHT
            )

            pose_world = pixel_to_world_pose(
                tf_buffer=self.tf_buffer,
                x_px=cx,
                y_px=cy,
                z_offset=DICE_HALF_HEIGHT,
                img_w=img_w,
                img_h=img_h,
                yaw_rad=yaw_rad,
                node=self
            )

            if pose_world is None:
                self.get_logger().warn("Skipping dice detection due to TF failure.")
                continue

            self.get_logger().info(
                f"Dice board position: ({x_m:.3f}, {y_m:.3f}, {z_m:.3f}), "
                f"yaw={math.degrees(yaw_rad):.1f}°"
            )

            # --- Build DiceResult (world frame pose) ---
            dr = DiceResult()
            dr.x = x1
            dr.y = y1
            dr.width = x2 - x1
            dr.height = y2 - y1
            dr.dice_number = dice_num
            dr.confidence = conf

            # WORLD pose from TF
            dr.pose = pose_world.pose

            # Optional: extra rotation around Y if needed for end-effector convention
            q_tf = dr.pose.orientation
            q_tf_np = [q_tf.x, q_tf.y, q_tf.z, q_tf.w]

            # Example: 180° around Y (adjust if needed)
            q_extra = quaternion_from_euler(0.0, math.pi, 0.0)
            q_final = quaternion_multiply(q_tf_np, q_extra)

            dr.pose.orientation.x = q_final[0]
            dr.pose.orientation.y = q_final[1]
            dr.pose.orientation.z = q_final[2]
            dr.pose.orientation.w = q_final[3]

            self.get_logger().info(
                f"Dice world position: ({dr.pose.position.x:.3f}, "
                f"{dr.pose.position.y:.3f}, {dr.pose.position.z:.3f}), "
                f"yaw={math.degrees(yaw_rad):.1f}°"
            )

            dice_list_world.append(dr)

            # --- Board-frame version for markers ---
            dr_board = DiceResult()
            dr_board.pose.position.x = x_m
            dr_board.pose.position.y = y_m
            dr_board.pose.position.z = z_m

            q_board = quaternion_from_euler(0.0, 0.0, yaw_rad)
            dr_board.pose.orientation = Quaternion(
                x=float(q_board[0]),
                y=float(q_board[1]),
                z=float(q_board[2]),
                w=float(q_board[3]),
            )
            dr_board.dice_number = dice_num
            dice_list_board.append(dr_board)

            # --- PoseStamped for RViz EE axes ---
            viz_pose = PoseStamped()
            viz_pose.header.frame_id = "world"
            viz_pose.header.stamp = rclpy.time.Time().to_msg()
            viz_pose.pose = dr.pose
            viz_poses.append(viz_pose)

        # --- Publish results and markers ---
        dice_results_msg.dice = dice_list_world
        self.dice_pub.publish(dice_results_msg)
        self.publish_dice_markers(dice_list_board)

        # --- Visualise end-effector goal poses in RViz ---
        if viz_poses:
            visualise_pose_in_rviz(
                node=self,
                poses=viz_poses,            # function supports single or list
                marker_pub=self.ee_marker_pub,
                scale=0.06,                 # slightly smaller axes for dice
            )

        # --- Overlay crop region for debugging ---
        cv2.rectangle(frame_full, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
        self._draw_stats(frame_full, dice_count, total_sum)
        cv2.imshow(WINDOW_NAME, frame_full)

    # ----------------------------------------------------------
    def _draw_stats(self, frame, dice_count, total_sum):
        img_h, _ = frame.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        text1 = f"Dice Count: {dice_count}"
        text2 = f"Total Sum: {total_sum}"
        cv2.putText(frame, text1, (10, img_h - 40), font, 0.6, (0, 0, 0), 2)
        cv2.putText(frame, text2, (10, img_h - 20), font, 0.6, (0, 0, 0), 2)

    # ----------------------------------------------------------
    def draw_rotated_bbox(self, frame, cx, cy, w, h, yaw_rad, color=(0,0,255), thickness=2):
        """
        Draw a rotated bounding box given centre (cx, cy), width, height, and yaw.
        Width/height from YOLO box (x2-x1, y2-y1).
        """
        # Rotation matrix
        R = cv2.getRotationMatrix2D((cx, cy), -math.degrees(yaw_rad), 1.0)

        # Original rectangle corners (unrotated)
        rect = np.array([
            [cx - w/2, cy - h/2],
            [cx + w/2, cy - h/2],
            [cx + w/2, cy + h/2],
            [cx - w/2, cy + h/2]
        ])

        # Rotate each point
        rect_rot = []
        for p in rect:
            px, py = p
            x_new = R[0,0]*px + R[0,1]*py + R[0,2]
            y_new = R[1,0]*px + R[1,1]*py + R[1,2]
            rect_rot.append([int(x_new), int(y_new)])

        rect_rot = np.array(rect_rot)

        # Draw
        cv2.polylines(frame, [rect_rot], isClosed=True, color=color, thickness=thickness)

        return rect_rot  # return points in case needed


    # ----------------------------------------------------------
    def publish_dice_markers(self, dice_results):
        """Publish dice detections as cube + text markers in the board_frame."""

        # 1) CLEAR ALL PREVIOUS MARKERS
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        delete_all.header.frame_id = "board_frame"
        delete_all.header.stamp = Time().to_msg()

        clear_array = MarkerArray()
        clear_array.markers.append(delete_all)
        self.marker_pub.publish(clear_array)

        # 2) NOW PUBLISH THE NEW MARKERS
        marker_array = MarkerArray()
        zero_stamp = Time().to_msg()

        for i, dr in enumerate(dice_results):
            # --- Cube marker ---
            cube = Marker()
            cube.header.frame_id = "board_frame"
            cube.header.stamp = zero_stamp
            cube.ns = "dice"
            cube.id = i
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose = dr.pose
            cube.scale.x = 0.03
            cube.scale.y = 0.03
            cube.scale.z = 0.03
            cube.color.r = 1.0
            cube.color.g = 0.0
            cube.color.b = 0.0
            cube.color.a = 1.0
            marker_array.markers.append(cube)

            # --- Text marker ---
            text = Marker()
            text.header.frame_id = "board_frame"
            text.header.stamp = zero_stamp
            text.ns = "dice_text"
            text.id = i + 100
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = dr.pose.position.x
            text.pose.position.y = dr.pose.position.y
            text.pose.position.z = dr.pose.position.z + 0.05
            text.scale.z = 0.05
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = str(dr.dice_number)
            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)



# ----------------------------------------------------------
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
