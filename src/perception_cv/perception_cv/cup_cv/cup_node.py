#!/usr/bin/env python3
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from tf_transformations import quaternion_from_euler
from tf_transformations import quaternion_multiply
import numpy as np

import tf2_ros

from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray

from custom_interface.msg import CupResult
from geometry_msgs.msg import PoseStamped
import rclpy.time

from perception_cv import (
    pixel_to_board_coords,
    pixel_to_world_pose,
    visualise_pose_in_rviz,
    BOARD_W_MM,
    BOARD_H_MM,
    PIXEL_TO_METERS,
)

import numpy as np
from tf_transformations import (
    quaternion_from_euler,
    quaternion_multiply,
    quaternion_conjugate,
    quaternion_matrix
)

WINDOW_NAME = "Cup Detection"
CUP_HALF_HEIGHT = 0.08  # [m] approximate height offset for end-effector above board
MIN_CONTOUR_AREA = 100  # px^2, filter tiny blobs


class CupDetector(Node):
    def __init__(self):
        super().__init__("cup_detector")
        self.bridge = CvBridge()

        # --- Subscriber: warped board image (same as dice node) ---
        self.image_sub = self.create_subscription(
            Image,
            "board/warped_image",
            self.image_callback,
            10,
        )

        # --- Publishers ---
        self.cup_pub = self.create_publisher(CupResult, "cup_result", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "cup_markers", 10)
        self.ee_marker_pub = self.create_publisher(MarkerArray, "cup_ee_goal_markers", 10)

        # --- TF buffer/listener for pixel_to_world_pose ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- OpenCV window ---
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.gui_timer = self.create_timer(0.1, self.gui_tick) # 10 Hz

        self.get_logger().info("☕ Cup detector node started.")

    def gui_tick(self):
        """
        This fires even when NO new image is received.
        It processes OpenCV GUI events so VS Code doesn't freeze.
        """
        cv2.waitKey(1)

    # ==============================================================
    #   IMAGE CALLBACK
    # ==============================================================
    def image_callback(self, rgb_msg: Image):
        frame_full = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        img_h, img_w = frame_full.shape[:2]

        # ----------------------------------------------------------
        # Crop region where cup is expected
        # ----------------------------------------------------------
        pad_x = 50
        y_top = 0
        y_bottom = min(150, img_h)
        cropped = frame_full[y_top:y_bottom, pad_x:img_w - pad_x]

        if cropped.size == 0:
            self.get_logger().warn("Cropped region is empty, skipping frame.")
            return

        # ----------------------------------------------------------
        # HSV threshold for yellow cup
        # ----------------------------------------------------------
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([0, 0, 80], dtype=np.uint8)
        upper_yellow = np.array([90, 180, 255], dtype=np.uint8)

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.imshow("Cup Mask", mask_vis)

        # Morphological noise cleanup
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # ----------------------------------------------------------
        # Find contours
        # ----------------------------------------------------------
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        debug_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        if not contours:
            self.get_logger().info("No yellow contours found.")
            self.cup_pub.publish(CupResult())
            cv2.imshow(WINDOW_NAME, frame_full)
            return

        # Largest contour above area threshold
        largest_cnt = None
        largest_area = 0.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > MIN_CONTOUR_AREA and area > largest_area:
                largest_area = area
                largest_cnt = cnt

        if largest_cnt is None:
            self.get_logger().info("No contour above area threshold.")
            self.cup_pub.publish(CupResult())
            cv2.imshow(WINDOW_NAME, frame_full)
            return

        # ----------------------------------------------------------
        # Bounding box + centroid (CROPPED coords)
        # ----------------------------------------------------------
        x, y, w, h = cv2.boundingRect(largest_cnt)
        M = cv2.moments(largest_cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX = x + w // 2
            cY = y + h // 2

        # ----------------------------------------------------------
        # MinAreaRect (must come BEFORE using `box`)
        # ----------------------------------------------------------
        rect = cv2.minAreaRect(largest_cnt)
        box = cv2.boxPoints(rect)
        box = np.int32(box)

        # Draw rotated box on cropped debug image
        cv2.drawContours(debug_bgr, [box], 0, (0, 255, 0), 2)
        cv2.circle(debug_bgr, (cX, cY), 4, (0, 0, 255), -1)

        # ----------------------------------------------------------
        # Convert centroid → FULL coords
        # ----------------------------------------------------------
        cx_full = cX + pad_x
        cy_full = cY + y_top

        # ----------------------------------------------------------
        # Shift rotated box → FULL coords
        # ----------------------------------------------------------
        box_full = box.copy()
        box_full[:, 0] += pad_x
        box_full[:, 1] += y_top

        cv2.drawContours(frame_full, [box_full], 0, (0, 255, 0), 2)
        cv2.circle(frame_full, (cx_full, cy_full), 6, (0, 0, 255), -1)

        # ----------------------------------------------------------
        # Orientation from PCA (robust to perspective)
        # ----------------------------------------------------------
        pts = largest_cnt.reshape(-1, 2).astype(np.float32)

        # Compute PCA on contour points
        mean, eigenvectors = cv2.PCACompute(pts, mean=None)

        # Major axis (first eigenvector)
        major_axis = eigenvectors[0]   # unit vector [vx, vy]

        # Angle of major axis
        angle_rad = math.atan2(major_axis[1], major_axis[0])
        angle_deg = math.degrees(angle_rad)

        # Normalize angle to 0–180 range
        if angle_deg < 0:
            angle_deg += 180

        yaw_rad = -math.radians(angle_deg)   # keep your original yaw convention

        self.get_logger().info(
            f"Cup centroid (cropped): ({cX}, {cY}), "
            f"bbox {w}x{h}, PCA orientation ~ {angle_deg:.2f}°"
        )

        # ----------------------------------------------------------
        # Draw orientation arrow on FULL image
        # ----------------------------------------------------------
        scale = 40
        vx, vy = major_axis
        end_x = int(cx_full + vx * scale)
        end_y = int(cy_full + vy * scale)

        cv2.arrowedLine(
            frame_full,
            (cx_full, cy_full),
            (end_x, end_y),
            (255, 0, 0),
            2,
            tipLength=0.2
        )


        # ----------------------------------------------------------
        # Pixel → board coords
        # ----------------------------------------------------------
        x_m, y_m, z_m = pixel_to_board_coords(
            x_px=cx_full, y_px=cy_full,
            img_w=img_w, img_h=img_h,
            z_offset=CUP_HALF_HEIGHT,
        )

        # ----------------------------------------------------------
        # Pixel → world pose via TF
        # ----------------------------------------------------------
        pose_world = pixel_to_world_pose(
            tf_buffer=self.tf_buffer,
            x_px=cx_full,
            y_px=cy_full,
            z_offset=CUP_HALF_HEIGHT,
            img_w=img_w,
            img_h=img_h,
            yaw_rad=yaw_rad,
            node=self,
        )

        if pose_world is None:
            self.get_logger().warn("Skipping cup detection due to TF failure.")
            self.cup_pub.publish(CupResult())
            cv2.imshow(WINDOW_NAME, frame_full)
            return

        # Logging world position
        self.get_logger().info(
            f"Cup board position: ({x_m:.3f}, {y_m:.3f}, {z_m:.3f}), "
            f"world position: ({pose_world.pose.position.x:.3f}, "
            f"{pose_world.pose.position.y:.3f}, {pose_world.pose.position.z:.3f}), "
            f"yaw ~ {angle_deg:.1f}°"
        )

        # ----------------------------------------------------------
        # Build CupResult (WORLD)
        # ----------------------------------------------------------
        cup_world = CupResult()
        cup_world.x = int(x + pad_x)
        cup_world.y = int(y + y_top)
        cup_world.width = int(w)
        cup_world.height = int(h)
        cup_world.confidence = 1.0
        cup_world.pose = pose_world.pose
        cup_world.drop_pose = pose_world.pose

        # ----------------------------------------------------------
        # Apply LOCAL offset, then correction rotation
        # ----------------------------------------------------------
        q_tf = cup_world.pose.orientation
        q_tf_np = np.array([q_tf.x, q_tf.y, q_tf.z, q_tf.w])

        # ----------------------------------------------------------
        # 2. Compute world-space offset along LOCAL X
        # ----------------------------------------------------------
        offset_local = np.array([0.15, 0.0, 0.0, 0.0])   # homogeneous for matrix mult
        R_tf = quaternion_matrix(q_tf_np)                # 4×4 transform matrix

        offset_world = R_tf[:3, :3].dot(offset_local[:3])

        cup_world.pose.position.x -= offset_world[0]
        cup_world.pose.position.y -= offset_world[1]
        cup_world.pose.position.z -= offset_world[2]

        q_extra = quaternion_from_euler(math.pi/2, math.pi, math.pi/2)
        q_final = quaternion_multiply(q_tf_np, q_extra)

        cup_world.pose.orientation = Quaternion(
            x=float(q_final[0]),
            y=float(q_final[1]),
            z=float(q_final[2]),
            w=float(q_final[3])
        )

        # ----------------------------------------------------------
        # Board-frame copy for markers
        # ----------------------------------------------------------
        cup_board = CupResult()
        q = quaternion_from_euler(math.pi, 0.0, yaw_rad)
        cup_board.pose.position.x = x_m
        cup_board.pose.position.y = y_m
        cup_board.pose.position.z = z_m
        cup_board.pose.orientation = Quaternion(
            x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])
        )
        cup_board.drop_pose = cup_board.pose

        # Publish output
        self.cup_pub.publish(cup_world)
        self.publish_cup_markers([cup_board])

        # Visualise pose in RViz
        viz_pose = PoseStamped()
        viz_pose.header.frame_id = "world"
        viz_pose.header.stamp = rclpy.time.Time().to_msg()
        viz_pose.pose = cup_world.pose
        visualise_pose_in_rviz(
            node=self,
            poses=viz_pose,
            marker_pub=self.ee_marker_pub,
            scale=0.1
        )

        # ----------------------------------------------------------
        # Debug view
        # ----------------------------------------------------------
        cv2.imshow(WINDOW_NAME, frame_full)



    # ==============================================================
    #   RVIZ MARKERS (board_frame)
    # ==============================================================
    def publish_cup_markers(self, cup_results):
        """
        Publish cup detections as boxes (10×4×8 cm) in the board_frame.
        Only pose from CupResult is used.
        """
        marker_array = MarkerArray()
        zero_stamp = Time().to_msg()  # latest TF

        for i, cr in enumerate(cup_results):
            box = Marker()
            box.header.frame_id = "board_frame"
            box.header.stamp = zero_stamp
            box.ns = "cups"
            box.id = i
            box.type = Marker.CUBE          
            box.action = Marker.ADD

            box.pose = cr.pose

            # 10 × 4 × 8 cm  →  metres
            box.scale.x = 0.10   # length
            box.scale.y = 0.04   # width
            box.scale.z = 0.08   # height

            box.pose.position.z -= box.scale.z/2

            box.color.r = 1.0
            box.color.g = 1.0
            box.color.b = 0.0
            box.color.a = 0.9

            marker_array.markers.append(box)

        self.marker_pub.publish(marker_array)



# ==============================================================
#   MAIN
# ==============================================================
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


if __name__ == "__main__":
    main()
