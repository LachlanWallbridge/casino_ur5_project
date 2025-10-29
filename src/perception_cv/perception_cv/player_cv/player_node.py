#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_interface.msg import Player, Players
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.time import Time
import math
from tf_transformations import quaternion_from_euler

from perception_cv import pixel_to_board_coords

WINDOW_NAME = "Player Detection"
PLAYER_COLOR = (0, 255, 0)  # green
ZONE_COLOR = (255, 0, 0)    # blue overlay

class PlayerDetector(Node):
    def __init__(self):
        super().__init__("player_detector")
        self.bridge = CvBridge()

        # --- ArUco setup ---
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # --- Subscribers ---
        self.image_sub = self.create_subscription(
            Image,
            "board/warped_image",
            self.image_callback,
            10
        )

        # --- Publishers ---
        self.players_pub = self.create_publisher(Players, "players", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "player_markers", 10)

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.get_logger().info("🧍 Player detector node started (warped image mode).")

    # ----------------------------------------------------------
    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img_h, img_w = frame.shape[:2]

        # --- Define lower-half 3 zones (exclude 100 mm padding each side) ---
        pad_px = 100  # 100 mm → pixels
        y_start = img_h // 2
        y_end = img_h

        zone_width = (img_w - 2 * pad_px) // 3
        zone_bounds = [
            (pad_px + i * zone_width, pad_px + (i + 1) * zone_width)
            for i in range(3)
        ]
        # self.get_logger().info(f"Zone bounds (px): {zone_bounds}")

        # --- ArUco detection ---
        corners, ids, _ = self.detector.detectMarkers(frame)
        players_msg = Players()
        marker_array = MarkerArray()

        if ids is not None:
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids):
                c = corners[i][0]
                cx, cy = c.mean(axis=0)
                cy = int(cy)
                cx = int(cx)

                # --- Determine which zone it's in ---
                position = None
                if y_start <= cy <= y_end:
                    for idx, (x_min, x_max) in enumerate(zone_bounds):
                        if x_min <= cx <= x_max:
                            position = idx + 1  # 1, 2, 3
                            break

                if position is not None:
                    # Convert to metric board coords
                    x_m, y_m, z_m = pixel_to_board_coords(
                        x_px=cx,
                        y_px=cy,
                        img_w=img_w,
                        img_h=img_h,
                        z_offset=0.0
                    )

                    # Build player message
                    p = Player()
                    p.player_id = str(marker_id)
                    p.position = position
                    players_msg.players.append(p)

                    # --- Add marker for RViz ---
                    q = quaternion_from_euler(0.0, 0.0, 0.0)
                    cube = Marker()
                    cube.header.frame_id = "board_frame"
                    cube.header.stamp = Time().to_msg()
                    cube.ns = "players"
                    cube.id = int(marker_id)
                    cube.type = Marker.CUBE
                    cube.action = Marker.ADD
                    cube.pose.position.x = float(x_m)
                    cube.pose.position.y = float(y_m)
                    cube.pose.position.z = float(z_m)
                    cube.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                    cube.scale.x = cube.scale.y = 0.1
                    cube.scale.z = 0.02
                    cube.color.r = 0.0
                    cube.color.g = 1.0
                    cube.color.b = 0.0
                    cube.color.a = 1.0
                    marker_array.markers.append(cube)

                    # Text label
                    text = Marker()
                    text.header.frame_id = "board_frame"
                    text.header.stamp = cube.header.stamp
                    text.ns = "player_text"
                    text.id = int(marker_id) + 100
                    text.type = Marker.TEXT_VIEW_FACING
                    text.action = Marker.ADD
                    text.pose.position.x = cube.pose.position.x
                    text.pose.position.y = cube.pose.position.y
                    text.pose.position.z = cube.pose.position.z + 0.05
                    text.scale.z = 0.06
                    text.color.r = text.color.g = text.color.b = text.color.a = 1.0
                    text.text = f"Player {marker_id} (P{position})"
                    marker_array.markers.append(text)

                    # Overlay for debugging
                    cv2.putText(frame, f"P{position}", (cx - 20, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, PLAYER_COLOR, 2)

        # Publish results
        self.players_pub.publish(players_msg)
        self.marker_pub.publish(marker_array)

        # --- Draw crop zones for visualization ---
        for i, (x_min, x_max) in enumerate(zone_bounds):
            cv2.rectangle(frame, (x_min, y_start), (x_max, y_end), ZONE_COLOR, 2)
            cv2.putText(frame, f"Zone {i+1}", (x_min + 10, y_start + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, ZONE_COLOR, 2)

        cv2.imshow(WINDOW_NAME, frame)
        cv2.waitKey(1)


# ----------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PlayerDetector()
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
