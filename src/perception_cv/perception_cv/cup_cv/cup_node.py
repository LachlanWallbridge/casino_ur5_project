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

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.get_logger().info("☕ Cup detector node started (yellow colour + board_frame).")

    # ==============================================================
    #   IMAGE CALLBACK
    # ==============================================================
    def image_callback(self, rgb_msg: Image):
        frame_full = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        img_h, img_w = frame_full.shape[:2]

        # ----------------------------------------------------------
        # Crop region where cup is expected (adapted from your script)
        #   warped = img[0:150, 50:(765-50)]
        # Here we generalise to current image width:
        # ----------------------------------------------------------
        pad_x = 50
        y_top = 0
        y_bottom = min(150, img_h)

        cropped = frame_full[y_top:y_bottom, pad_x : img_w - pad_x]

        if cropped.size == 0:
            self.get_logger().warn("Cropped region is empty, skipping frame.")
            return

        # ----------------------------------------------------------
        # HSV threshold for yellow cup
        # ----------------------------------------------------------
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([35, 50, 100], dtype=np.uint8)
        upper_yellow = np.array([90, 110, 255], dtype=np.uint8)

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Optional: small morphology to clean noise
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # ----------------------------------------------------------
        # Find contours of yellow region
        # ----------------------------------------------------------
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Debug image (BGR) to draw centroid + box
        debug_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        if not contours:
            self.get_logger().info("No yellow contours found.")
            self.cup_pub.publish(CupResult())  # publish empty
            cv2.imshow(WINDOW_NAME, frame_full)
            cv2.waitKey(1)
            return

        # Pick the largest contour above area threshold
        largest_cnt = None
        largest_area = 0.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_CONTOUR_AREA:
                continue
            if area > largest_area:
                largest_area = area
                largest_cnt = cnt

        if largest_cnt is None:
            self.get_logger().info("No contour above area threshold.")
            self.cup_pub.publish(CupResult())
            cv2.imshow(WINDOW_NAME, frame_full)
            cv2.waitKey(1)
            return

        # ----------------------------------------------------------
        # Bounding box + centroid (in CROPPED coordinates)
        # ----------------------------------------------------------
        x, y, w, h = cv2.boundingRect(largest_cnt)
        moments = cv2.moments(largest_cnt)
        if moments["m00"] != 0:
            cX = int(moments["m10"] / moments["m00"])
            cY = int(moments["m01"] / moments["m00"])
        else:
            # fallback: box centre
            cX = x + w // 2
            cY = y + h // 2

        # Draw in cropped debug image
        cv2.rectangle(debug_bgr, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.circle(debug_bgr, (cX, cY), 5, (0, 0, 255), -1)

        # ----------------------------------------------------------
        # Orientation from minAreaRect (in CROPPED coordinates)
        # ----------------------------------------------------------
        rect = cv2.minAreaRect(largest_cnt)
        box = cv2.boxPoints(rect)
        box = np.int32(box)

        # Draw rotated rectangle
        cv2.drawContours(debug_bgr, [box], 0, (0, 255, 0), 2)

        # Determine longer side to compute orientation
        d1 = np.linalg.norm(box[0] - box[1])
        d2 = np.linalg.norm(box[1] - box[2])
        if d1 >= d2:
            pt1, pt2 = box[0], box[1]
        else:
            pt1, pt2 = box[1], box[2]

        dx = float(pt2[0] - pt1[0])
        dy = float(pt2[1] - pt1[1])

        angle_deg = math.degrees(math.atan2(dy, dx))  # [-180, 180]
        if angle_deg < 0:
            angle_deg += 360
        if angle_deg > 180:
            angle_deg -= 180

        yaw_rad = math.radians(angle_deg)*-1

        self.get_logger().info(
            f"Cup centroid (cropped): ({cX}, {cY}), "
            f"bbox {w}x{h}, orientation ~ {angle_deg:.2f}°"
        )

        # ----------------------------------------------------------
        # Convert CROPPED centroid → FULL image pixel coordinates
        # ----------------------------------------------------------
        cx_full = cX + pad_x
        cy_full = cY + y_top

        # Draw on full frame for debug (box & centroid)
        cv2.circle(frame_full, (cx_full, cy_full), 6, (0, 0, 255), -1)
        cv2.rectangle(
            frame_full,
            (x + pad_x, y + y_top),
            (x + pad_x + w, y + y_top + h),
            (0, 0, 255),
            2,
        )

        # ----------------------------------------------------------
        # Pixel → board frame coordinates (in metres)
        # ----------------------------------------------------------
        x_m, y_m, z_m = pixel_to_board_coords(
            x_px=cx_full,
            y_px=cy_full,
            img_w=img_w,
            img_h=img_h,
            z_offset=CUP_HALF_HEIGHT,
        )

        # ----------------------------------------------------------
        # Pixel → world frame (through TF), as PointStamped
        # ----------------------------------------------------------
        pose_world = pixel_to_world_pose(
            tf_buffer=self.tf_buffer,
            x_px=cx_full,
            y_px=cy_full,
            z_offset=CUP_HALF_HEIGHT,
            img_w=img_w,
            img_h=img_h,
            yaw_rad=yaw_rad,
            node=self
        )

        if pose_world is None:
            self.get_logger().warn("Skipping cup detection due to TF failure.")
            self.cup_pub.publish(CupResult())
            cv2.imshow(WINDOW_NAME, frame_full)
            cv2.waitKey(1)
            return

        self.get_logger().info(
            f"Cup board position: ({x_m:.3f}, {y_m:.3f}, {z_m:.3f}), "
            f"world position: ({pose_world.pose.position.x:.3f}, "
            f"{pose_world.pose.position.y:.3f}, {pose_world.pose.position.z:.3f}), "
            f"yaw ~ {angle_deg:.1f}°"
        )

        # ----------------------------------------------------------
        # Build CupResult (WORLD frame pose)
        # ----------------------------------------------------------
        cup_world = CupResult()
        cup_world.x = int(x + pad_x)
        cup_world.y = int(y + y_top)
        cup_world.width = int(w)
        cup_world.height = int(h)

        # Colour-based detector → treat as "confident" for now
        cup_world.confidence = 1.0


        cup_world.pose = pose_world.pose
        
        # Extract TF quaternion
        q_tf = pose_world.pose.orientation
        q_tf_np = [q_tf.x, q_tf.y, q_tf.z, q_tf.w]

        # Extra +90° rotation around Y
        q_extra = quaternion_from_euler(0.0, 3*math.pi/2, 0.0)

        # Compose rotations → q_final = q_tf ⊗ q_extra
        q_final = quaternion_multiply(q_tf_np, q_extra)

        # Assign back
        cup_world.pose.orientation.x = q_final[0]
        cup_world.pose.orientation.y = q_final[1]
        cup_world.pose.orientation.z = q_final[2]
        cup_world.pose.orientation.w = q_final[3]

        # ----------------------------------------------------------
        # Build a board-frame version for RViz markers
        # ----------------------------------------------------------
        cup_board = CupResult()
        q = quaternion_from_euler(math.pi, 0.0, yaw_rad)
        cup_board.pose.position.x = x_m
        cup_board.pose.position.y = y_m
        cup_board.pose.position.z = z_m
        cup_board.pose.orientation = Quaternion(
            x=float(q[0]),
            y=float(q[1]),
            z=float(q[2]),
            w=float(q[3]),
        )

        # ----------------------------------------------------------
        # Publish results + markers
        # ----------------------------------------------------------
        self.cup_pub.publish(cup_world)

        # Visualise cup
        self.publish_cup_markers([cup_board])
        
        # ----------- Visualise pose ---------------
        # Wrap cup_world.pose into a PoseStamped
        viz_pose = PoseStamped()
        viz_pose.header.frame_id = "world"   # IMPORTANT
        viz_pose.header.stamp = rclpy.time.Time().to_msg()
        viz_pose.pose = cup_world.pose
        visualise_pose_in_rviz(
            node=self,
            poses=viz_pose,
            marker_pub=self.ee_marker_pub,
            scale=0.1  # 10 cm axes
        )


        # ----------------------------------------------------------
        # Debug views
        # ----------------------------------------------------------
        # Optional: show cropped debug window with mask/box/centroid
        # cv2.imshow("Cup Cropped Debug", debug_bgr)

        cv2.imshow(WINDOW_NAME, frame_full)
        cv2.waitKey(1)

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
