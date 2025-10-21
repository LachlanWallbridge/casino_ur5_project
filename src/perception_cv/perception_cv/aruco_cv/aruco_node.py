#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from custom_interface.msg import Player, Players


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ---- Parameters ----
        self.declare_parameter('camera_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('show_image', True)
        self.show_image = self.get_parameter('show_image').get_parameter_value().bool_value

        # ---- CV + ArUco setup ----
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # ---- ROS I/O ----
        self.subscription = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').get_parameter_value().string_value,
            self.image_callback,
            10
        )
        self.players_pub = self.create_publisher(Players, 'players', 10)
        self.warped_pub = self.create_publisher(Image, 'board/warped_image', 10)

        self.get_logger().info('✅ ArUco detector node started.')

    # ----------------------------------------------------------
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = self.detector.detectMarkers(frame)

        players_msg = Players()

        if ids is not None:
            ids = ids.flatten()

            # --- Draw markers for visual debugging ---
            cv2.aruco.drawDetectedMarkers(frame, corners)

            for i, marker_id in enumerate(ids):
                c = corners[i][0]
                center = tuple(c.mean(axis=0).astype(int))  # compute center of marker
                cv2.putText(frame, str(marker_id), center, 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # --- Detect board corners (0–3) ---
            board_points = self.get_board_corners(ids, corners)

            # warp frame first
            if len(board_points) == 4:
                warped, H = self.warp_board(frame, board_points)
                if H is not None:
                    warped_msg = self.bridge.cv2_to_imgmsg(warped, encoding='bgr8')
                    self.warped_pub.publish(warped_msg)
                    if self.show_image:
                        cv2.imshow('Board Warped', warped)

                    # Warp marker corners into top-down board frame
                    for i, marker_id in enumerate(ids):
                        if marker_id not in [0,1,2,3]:
                            player_msg = Player()
                            player_msg.player_id = str(marker_id)

                            src_pts = np.array(corners[i][0], dtype=np.float32)  # 4x2
                            dst_pts = cv2.perspectiveTransform(src_pts[None, :, :], H)  # 1x4x2
                            player_msg.position = self.map_marker_to_position(dst_pts[0])

                            players_msg.players.append(player_msg)


        # --- Publish players ---
        self.players_pub.publish(players_msg)

        if self.show_image:
            cv2.imshow('Aruco Detection', frame)
            cv2.waitKey(1)

    # ----------------------------------------------------------
    def get_board_corners(self, ids, corners):
        """Extracts corner centers for markers 0-3."""
        board_pts = {}
        for i, marker_id in enumerate(ids):
            if marker_id in [0, 1, 2, 3]:
                c = corners[i][0]
                center = np.mean(c, axis=0)
                board_pts[marker_id] = center

        if len(board_pts) != 4:
            return []

        # Return in fixed order: top-left, top-right, bottom-right, bottom-left
        return [board_pts[i] for i in range(4)]

    # ----------------------------------------------------------
    def warp_board(self, frame, board_points):
        """Warp the image to a 400x800mm top-down view."""
        try:
            board_h_mm, board_w_mm = 390, 756
            dst_pts = np.array([
                [0, 0],
                [board_w_mm - 1, 0],
                [board_w_mm - 1, board_h_mm - 1],
                [0, board_h_mm - 1]
            ], dtype=np.float32)
            src_pts = np.array(board_points, dtype=np.float32)
            H, _ = cv2.findHomography(src_pts, dst_pts)
            warped = cv2.warpPerspective(frame, H, (int(board_w_mm), int(board_h_mm)))
            return warped, H
        except Exception as e:
            self.get_logger().warn(f"warp_board failed: {e}")
            return frame, None

    # ----------------------------------------------------------
    def map_marker_to_position(self, marker_corners: np.ndarray) -> int:
        """
        Map marker coordinates in the warped board image to a player area (1,2,3).

        Parameters
        ----------
        marker_corners : np.ndarray
            Array of shape (4,2) with the marker corner coordinates in the warped image.

        Returns
        -------
        int
            Player position: 1, 2, or 3. Returns -1 if outside player area.
        """

        board_h_mm, board_w_mm = 400, 800  # same as warp
        lower_half_y = board_h_mm / 2  # y threshold for lower half

        # Compute marker center
        center = np.mean(marker_corners, axis=0)
        x, y = center

        # Check if marker is in the lower half
        if y < lower_half_y:
            return -1  # not in player area

        # Define horizontal player area boundaries with 50px margin
        x_start = 50
        x_end = board_w_mm - 50
        zone_width = (x_end - x_start) / 3

        # Determine which zone the marker center falls into
        if x < x_start or x > x_end:
            return -1  # outside playable area
        elif x < x_start + zone_width:
            return 1
        elif x < x_start + 2 * zone_width:
            return 2
        else:
            return 3



def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
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
