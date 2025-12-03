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
CUP_HALF_HEIGHT = 0.06  # [m] approximate height offset for end-effector above board
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

        # ---------------------------------------
        # 1. Preprocess & crop
        # ---------------------------------------
        frame_full = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        cropped, mask, debug_bgr, frame_full, (pad_x, y_top) = self.preprocess_image(
            frame_full
        )

        if cropped is None:
            self.get_logger().warn("Empty crop.")
            return

        # ---------------------------------------
        # 2. Find largest yellow contour
        # ---------------------------------------
        cnt = self.find_largest_contour(mask)
        if cnt is None:
            self.cup_pub.publish(CupResult())
            cv2.imshow(WINDOW_NAME, frame_full)
            return


        # ---------------------------------------
        # 3. Yaw orientation
        # ---------------------------------------
        angle_deg, yaw_rad, box, dir_vec = self.compute_orientation(cnt)

        # ---------------------------------------
        # 4. Base centroid
        # ---------------------------------------
        base_rect = self.reconstruct_base_box(box, dir_vec)

        # rectangle centroid = mean of its corners
        cX = int(np.mean(base_rect[:, 0]))
        cY = int(np.mean(base_rect[:, 1]))

        # Draw debug on the cropped mask image
        self.draw_on_frame(
            frame_full=frame_full,
            box=box,
            cX=cX,
            cY=cY,
            pad_x=pad_x,
            y_top=y_top,
            reconstructed=base_rect,
        )

        # ---------------------------------------
        # Convert to full-frame coords
        # ---------------------------------------
        cx_full = cX + pad_x
        cy_full = cY + y_top

        # Orientation arrow on full frame
        scale = 40
        dx, dy = math.cos(yaw_rad), -math.sin(yaw_rad)
        end_x = int(cx_full + dx * scale)
        end_y = int(cy_full + dy * scale)
        cv2.arrowedLine(frame_full, (cx_full, cy_full), (end_x, end_y), (255, 0, 0), 2)

        # ---------------------------------------
        # 6. Pixel → world pose
        # ---------------------------------------
        img_h, img_w = frame_full.shape[:2]
        pose_world = self.compute_world_pose(cx_full, cy_full, yaw_rad, img_w, img_h)

        if pose_world is None:
            self.get_logger().warn("Skipping cup detection due to TF failure.")
            cv2.imshow(WINDOW_NAME, frame_full)
            return

        # ---------------------------------------
        # 7. Publish everything and show main figure
        # ---------------------------------------
        self.publish_results(
            cnt=cnt,
            cx_full=cx_full,
            cy_full=cy_full,
            angle_deg=angle_deg,
            yaw_rad=yaw_rad,
            pose_world=pose_world,
            frame_full=frame_full,
            pad_x=pad_x,
            y_top=y_top,
        )



    def preprocess_image(self, frame_full):
        img_h, img_w = frame_full.shape[:2]

        pad_x = 110
        y_top = 0
        y_bottom = img_h // 2 - 30

        cropped = frame_full[y_top:y_bottom, pad_x:img_w - pad_x]
        if cropped.size == 0:
            return None, None, None, None, (pad_x, y_top)

        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([0, 0, 80], dtype=np.uint8)
        upper_yellow = np.array([90, 227, 255], dtype=np.uint8)

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        debug_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Visualise mask (like before)
        cv2.imshow("Cup Mask", debug_bgr)

        return cropped, mask, debug_bgr, frame_full, (pad_x, y_top)

    def find_largest_contour(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest = None
        max_area = 0
        for c in contours:
            a = cv2.contourArea(c)
            if a > MIN_CONTOUR_AREA and a > max_area:
                largest = c
                max_area = a

        return largest

    def compute_base_centroid(self, cnt):
        cnt_pts = cnt[:, 0, :]          # (N, 2)
        ys = cnt_pts[:, 1]
        max_y = np.max(ys)

        band = 3
        base_pts = cnt_pts[ys > max_y - band]

        x, y, w, h = cv2.boundingRect(cnt)

        if len(base_pts) < 3:
            return x + w // 2, y + h // 2

        cX = int(np.mean(base_pts[:, 0]))
        cY = int(np.mean(base_pts[:, 1]))

        return cX, cY
    
    def compute_orientation(self, cnt):
        """
        Compute yaw and direction of the cup's long axis using minAreaRect,
        but bias against horizontal edges so that near-square boxes still
        choose the more vertical long edge.
        """
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect).astype(np.float32)

        # ------------------------------------------------------
        # Build the 4 edges of the rotated rect
        # ------------------------------------------------------
        edges = [
            (box[0], box[1]),   # top
            (box[1], box[2]),   # right
            (box[2], box[3]),   # bottom
            (box[3], box[0]),   # left
        ]

        K = 0.5  # vertical preference factor (tune if needed)

        best_idx = 0
        best_score = -1.0

        for i, (pA, pB) in enumerate(edges):
            d = pB - pA
            length = float(np.linalg.norm(d))
            if length < 1e-6:
                continue

            dx = d[0] / length
            dy = d[1] / length

            # verticalness = |dy|: 1 = perfectly vertical, 0 = perfectly horizontal
            verticalness = abs(dy)

            # Prefer longer and more vertical edges
            score = length * (1.0 + K * verticalness)

            if score > best_score:
                best_score = score
                best_idx = i

        # ------------------------------------------------------
        # If everything was degenerate, fall back to your old logic
        # ------------------------------------------------------
        if best_score <= 0.0:
            # Original behaviour: compare edge (0–1) vs (1–2)
            d1 = np.linalg.norm(box[0] - box[1])
            d2 = np.linalg.norm(box[1] - box[2])
            if d1 >= d2:
                pt1, pt2 = box[0], box[1]
            else:
                pt1, pt2 = box[1], box[2]
        else:
            pt1, pt2 = edges[best_idx]

        # ------------------------------------------------------
        # From chosen edge → dir_vec, angle, yaw
        # ------------------------------------------------------
        dx = float(pt2[0] - pt1[0])
        dy = float(pt2[1] - pt1[1])

        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            # Super defensive fallback
            dir_vec = np.array([1.0, 0.0], dtype=float)
            dx, dy = dir_vec  # for angle calc
        else:
            dir_vec = np.array([dx / norm, dy / norm], dtype=float)

        angle_deg = math.degrees(math.atan2(dy, dx))
        if angle_deg < 0:
            angle_deg += 360
        if angle_deg > 180:
            angle_deg -= 180

        yaw_rad = math.radians(angle_deg) * -1

        return angle_deg, yaw_rad, box.astype(np.int32), dir_vec



    def draw_debug_overlay(self, debug_bgr, box, cX, cY, reconstructed=None):
        cv2.drawContours(debug_bgr, [box], 0, (0, 255, 0), 2)
        cv2.circle(debug_bgr, (cX, cY), 4, (0, 0, 255), -1)

        if reconstructed is not None:
            cv2.drawContours(debug_bgr, [np.int32(reconstructed)], 0, (0, 255, 255), 2)

        cv2.imshow("Cup Debug", debug_bgr)

    def draw_on_frame(self, frame_full, box, cX, cY, pad_x, y_top, reconstructed=None):

        # --- Draw minAreaRect on full frame ---
        box_full = box.copy()
        box_full[:, 0] += pad_x
        box_full[:, 1] += y_top
        cv2.drawContours(frame_full, [box_full], 0, (0, 255, 0), 2)

        # --- Draw centroid ---
        cv2.circle(frame_full, (cX + pad_x, cY + y_top), 6, (0, 0, 255), -1)

        # --- Draw reconstructed base rectangle ---
        if reconstructed is not None:
            rect_full = reconstructed.copy()
            rect_full[:, 0] += pad_x
            rect_full[:, 1] += y_top
            cv2.drawContours(frame_full, [rect_full.astype(np.int32)], 0, (0, 255, 255), 2)


    def get_long_edges(self, box):
        """
        Return the TWO longest edges, but weighted so that more vertical
        edges are favoured. Helps when the rectangle becomes nearly square.
        """

        # --------- SAFETY CHECKS ---------  
        if box is None:
            return None
        if len(box) != 4:
            return None

        # edges are in order: top, right, bottom, left
        edges = [
            (box[0], box[1]),
            (box[1], box[2]),
            (box[2], box[3]),
            (box[3], box[0]),
        ]

        weighted_lengths = []
        K = 0.5  # vertical preference factor

        for pA, pB in edges:
            d = pB - pA
            length = np.linalg.norm(d)
            if length < 1e-6:
                weighted_lengths.append(0)
                continue

            dx, dy = d / length
            verticalness = abs(dy)  # 1 = vertical, 0 = horizontal

            # Weighted score: vertical edges get boosted
            score = length * (1.0 + K * verticalness)
            weighted_lengths.append(score)

        # --------- SAFETY: handle empty or invalid ----------
        if not weighted_lengths or max(weighted_lengths) == 0:
            # return nothing → skip this frame safely
            return None

        # best edge index
        i = int(np.argmax(weighted_lengths))

        # opposite edge index
        j = (i + 2) % 4

        return [edges[i], edges[j]]



    def pick_leftmost_edge(self, long_edges):
        """
        Given two parallel long edges, return the one with the smallest
        mean X coordinate → leftmost edge.
        """
        cx0 = (long_edges[0][0][0] + long_edges[0][1][0]) / 2.0
        cx1 = (long_edges[1][0][0] + long_edges[1][1][0]) / 2.0

        if cx0 < cx1:
            return long_edges[0]
        else:
            return long_edges[1]


    def reconstruct_base_box(self, box, dir_vec):
        """
        Construct a 10x4cm base rectangle that:
        - Shares the leftmost long edge of the minAreaRect box
        - Extends inward (to the RIGHT) by the known base width
        """

        BASE_LONG_M = 0.08
        BASE_SHORT_M = 0.045

        L_px = BASE_LONG_M / PIXEL_TO_METERS
        S_px = BASE_SHORT_M / PIXEL_TO_METERS

        # ---------------------------
        # 1. Get BOTH long edges
        # ---------------------------
        long_edges = self.get_long_edges(box)

        if long_edges is None:
            self.get_logger().warn("Failed to find long edges for base reconstruction.")
            return None

        # ---------------------------
        # 2. Pick the LEFTMOST one
        # ---------------------------
        pL, pR = self.pick_leftmost_edge(long_edges)
        pL = pL.astype(float)
        pR = pR.astype(float)

        # ---------------------------
        # 3. Build perpendicular vector pointing right
        # ---------------------------
        perp = np.array([-dir_vec[1], dir_vec[0]])
        if perp[0] < 0:  # ensure pointing right
            perp = -perp

        # ---------------------------
        # 4. Construct rectangle
        # ---------------------------
        p2 = pR + perp * S_px  # inward side
        p1 = pL + perp * S_px

        return np.vstack([pL, pR, p2, p1])





    
    def compute_world_pose(self, cx_full, cy_full, yaw_rad, img_w, img_h):
        return pixel_to_world_pose(
            tf_buffer=self.tf_buffer,
            x_px=cx_full,
            y_px=cy_full,
            z_offset=CUP_HALF_HEIGHT,
            img_w=img_w,
            img_h=img_h,
            yaw_rad=yaw_rad,
            node=self,
        )

    def publish_results(
        self,
        cnt,
        cx_full,
        cy_full,
        angle_deg,
        yaw_rad,
        pose_world,
        frame_full,
        pad_x,
        y_top,
    ):
        import copy

        img_h, img_w = frame_full.shape[:2]

        # Board-frame coordinates from full-frame centroid
        x_m, y_m, z_m = pixel_to_board_coords(
            x_px=cx_full,
            y_px=cy_full,
            img_w=img_w,
            img_h=img_h,
            z_offset=CUP_HALF_HEIGHT,
        )

        # Bounding box in CROPPED coords, then shifted
        x, y, w, h = cv2.boundingRect(cnt)

        # Logging world + board positions
        self.get_logger().info(
            f"Cup board position: ({x_m:.3f}, {y_m:.3f}, {z_m:.3f}), "
            f"world position: ({pose_world.pose.position.x:.3f}, "
            f"{pose_world.pose.position.y:.3f}, {pose_world.pose.position.z:.3f}), "
            f"yaw ~ {angle_deg:.1f}°"
        )

        # ------------------------------------------------------
        # Build CupResult (WORLD)
        # ------------------------------------------------------
        cup_world = CupResult()
        cup_world.x = int(x + pad_x)
        cup_world.y = int(y + y_top)
        cup_world.width = int(w)
        cup_world.height = int(h)
        cup_world.confidence = 1.0
        cup_world.pose = pose_world.pose

        cup_world.drop_pose = copy.deepcopy(pose_world.pose)

        # ------------------------------------------------------
        # Apply LOCAL offset, then correction rotation
        # ------------------------------------------------------
        q_tf = cup_world.pose.orientation
        q_tf_np = np.array([q_tf.x, q_tf.y, q_tf.z, q_tf.w])

        # world-space offset along LOCAL X
        offset_local = np.array([0.14, 0.0, 0.0, 0.0])   # homogeneous
        R_tf = quaternion_matrix(q_tf_np)                # 4×4 matrix
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
            w=float(q_final[3]),
        )

        # ------------------------------------------------------
        # Board-frame copy for markers
        # ------------------------------------------------------
        cup_board = CupResult()
        q_board = quaternion_from_euler(math.pi, 0.0, yaw_rad)

        cup_board.pose.position.x = x_m
        cup_board.pose.position.y = y_m
        cup_board.pose.position.z = z_m
        cup_board.pose.orientation = Quaternion(
            x=float(q_board[0]),
            y=float(q_board[1]),
            z=float(q_board[2]),
            w=float(q_board[3]),
        )
        cup_board.drop_pose = cup_board.pose

        # Publish outputs
        self.cup_pub.publish(cup_world)
        self.publish_cup_markers([cup_board])

        # Visualise EE goal in RViz
        viz_pose = PoseStamped()
        viz_pose.header.frame_id = "world"
        viz_pose.header.stamp = rclpy.time.Time().to_msg()
        viz_pose.pose = cup_world.pose

        visualise_pose_in_rviz(
            node=self,
            poses=viz_pose,
            marker_pub=self.ee_marker_pub,
            scale=0.1,
        )

        # ------------------------------------------------------
        # Main debug view (Cup Detection figure)
        # ------------------------------------------------------
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
