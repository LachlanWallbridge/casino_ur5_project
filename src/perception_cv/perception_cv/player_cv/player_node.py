#!/usr/bin/env python3
import math
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
from tf_transformations import quaternion_from_euler

from perception_cv import pixel_to_board_coords

WINDOW_NAME = "Player Detection"
PLAYER_COLOR = (0, 255, 0)
ZONE_COLOR = (255, 0, 0)
LOWER_LIMIT = 30  # px
UPPER_LIMIT = 42  # px

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

        # --- OpenCV window ---
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.gui_timer = self.create_timer(0.1, self.gui_tick) # 10 Hz

        self.get_logger().info("🧍 Player detector node started (warped image mode).")

    def gui_tick(self):
        """
        This fires even when NO new image is received.
        It processes OpenCV GUI events so VS Code doesn't freeze.
        """
        cv2.waitKey(1)

    # ----------------------------------------------------------
    def create_player_markers(self, marker_id, x_m, y_m, z_m):
        """Return a list of RViz markers for a player (cube + text)."""
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        now = Time().to_msg()

        cube = Marker()
        cube.header.frame_id = "board_frame"
        cube.header.stamp = now
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

        text = Marker()
        text.header.frame_id = "board_frame"
        text.header.stamp = now
        text.ns = "player_text"
        text.id = int(marker_id) + 100
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = cube.pose.position.x
        text.pose.position.y = cube.pose.position.y
        text.pose.position.z = cube.pose.position.z + 0.05
        text.scale.z = 0.06
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f"Player {marker_id}"

        return [cube, text]

    # ----------------------------------------------------------
    def detect_chips_in_zones(self, src_frame, display_frame, zone_bounds,
                          x_offset=0, y_offset=0):
        hsv = cv2.cvtColor(src_frame, cv2.COLOR_BGR2HSV)
        detections = []

        # --- HSV color thresholds ---
        color_ranges = {
            "blue":  ((100, 150, 80), (130, 255, 255)),
            "green": ((85,  60,  40), (95, 255, 255)),
            "red":   ((150, 25, 25),  (179, 255, 255)),
            "white": ((95, 60, 150),  (115, 140, 255)),
        }

        color_bgr = {
            "red":   (150, 103, 135),
            "blue":  (38, 116, 166),
            "green": (47, 141, 152),
            "white": (136, 192, 234),
        }

        # Precompute valid area bounds based on diameters
        AREA_MIN = math.pi * (LOWER_LIMIT / 2) ** 2
        AREA_MAX = math.pi * (UPPER_LIMIT / 2) ** 2

        for name, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            kernel = np.ones((7, 7), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.medianBlur(mask, 5)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            color_name = "red" if "red" in name else name

            # debug_window = f"mask_{name}"
            # cv2.imshow(debug_window, mask)

            for cnt in contours:
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                diameter = 2 * radius
                area = cv2.contourArea(cnt)

                # --- Filter by size and shape ---
                if not (LOWER_LIMIT <= diameter <= UPPER_LIMIT):
                    continue
                if not (AREA_MIN <= area <= AREA_MAX):
                    continue

                hull = cv2.convexHull(cnt)
                solidity = float(area) / cv2.contourArea(hull)
                if solidity < 0.8:
                    continue  # skip irregular or fragmented blobs

                cx, cy = int(x) + x_offset, int(y) + y_offset
                zone_id = None
                for idx, (x_min, x_max) in enumerate(zone_bounds):
                    if x_min <= cx <= x_max:
                        zone_id = idx + 1
                        break
                if zone_id is None:
                    continue

                detections.append((color_name, cx, cy, diameter, zone_id))

                # --- Draw overlay ---
                cv2.circle(display_frame, (cx, cy), int(radius), color_bgr[color_name], 2)
                label = f"{color_name} ({int(diameter)}px) Z{zone_id}"
                cv2.putText(display_frame, label, (cx - 45, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr[color_name], 2)

        return detections


    # ----------------------------------------------------------
    def image_callback(self, msg: Image):
        src_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        display_frame = src_frame.copy()
        img_h, img_w = src_frame.shape[:2]

        pad_px = 100
        x_start, x_end = pad_px, img_w - pad_px
        y_start, y_end = img_h // 2 - 10, img_h

        # Crop to player region
        player_area = src_frame[y_start:y_end, x_start:x_end]

        zone_width = (img_w - 2 * pad_px) // 3
        zone_bounds = [(pad_px + i * zone_width, pad_px + (i + 1) * zone_width)
                    for i in range(3)]

        players_msg = Players()
        marker_array = MarkerArray()

        # --------------------------------------------------
        # 1️⃣ Detect ArUco players (positions)
        # --------------------------------------------------
        players_msg = Players()
        marker_array = MarkerArray()

        corners, ids, _ = self.detector.detectMarkers(player_area)
        if ids is not None:
            ids = ids.flatten()
            for i, marker_id in enumerate(ids):
                c = corners[i][0] + np.array([x_start, y_start])
                cx, cy = map(int, c.mean(axis=0))

                position = None
                for idx, (x_min, x_max) in enumerate(zone_bounds):
                    if x_min <= cx <= x_max:
                        position = idx + 1
                        break

                if position is not None:
                    x_m, y_m, z_m = pixel_to_board_coords(
                        x_px=cx, y_px=cy, img_w=img_w, img_h=img_h, z_offset=0.0
                    )

                    p = Player()
                    p.player_id = str(marker_id)
                    p.position = position
                    p.bet_is_odd = True  # Assume player has placed a bet TODO: Update based on actual detection

                    # Initialize arrays for multiple bets
                    p.bet_colors = []
                    p.bet_x = []
                    p.bet_y = []
                    p.bet_diameter_px = []

                    players_msg.players.append(p)
                    marker_array.markers.extend(
                        self.create_player_markers(marker_id, x_m, y_m, z_m)
                    )

                    cv2.putText(display_frame, f"P{position}",
                                (cx - 20, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                PLAYER_COLOR, 2)

        # --------------------------------------------------
        # 2️⃣ Detect chips and assign to players by zone
        # --------------------------------------------------
        chip_detections = self.detect_chips_in_zones(
            player_area, display_frame, zone_bounds,
            x_offset=x_start, y_offset=y_start
        )

        # Assign chips to players based on their zone_id
        for color, cx, cy, diameter, zone_id in chip_detections:
            for p in players_msg.players:
                if p.position == zone_id:
                    p.bet_colors.append(color)
                    p.bet_x.append(float(cx))
                    p.bet_y.append(float(cy))
                    p.bet_diameter_px.append(float(diameter))

        # --------------------------------------------------
        # 3️⃣ Publish and visualize
        # --------------------------------------------------
        self.players_pub.publish(players_msg)
        self.marker_pub.publish(marker_array)

        cv2.rectangle(display_frame, (x_start, y_start), (x_end, y_end), (255, 255, 0), 2)
        for i, (x_min, x_max) in enumerate(zone_bounds):
            cv2.rectangle(display_frame, (x_min, y_start), (x_max, y_end), ZONE_COLOR, 2)
            cv2.putText(display_frame, f"Zone {i+1}", (x_min + 10, y_start - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, ZONE_COLOR, 2)

        cv2.imshow(WINDOW_NAME, display_frame)


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
