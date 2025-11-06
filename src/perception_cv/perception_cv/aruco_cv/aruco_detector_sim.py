#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_interface.msg import Player, Players
from geometry_msgs.msg import TransformStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf_transformations import quaternion_from_euler

# Shared helpers
from perception_cv import broadcast_camera_to_world
from perception_cv import pixel_to_world_pose


class ArucoDetectorSim(Node):
    def __init__(self):
        super().__init__('aruco_detector_sim')

        # ---- Parameters ----
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('show_image', True)
        self.show_image = self.get_parameter('show_image').get_parameter_value().bool_value

        # ---- CV / ArUco ----
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # ---- Subscribers ----
        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        self.color_sub = self.create_subscription(Image, color_topic, self.color_callback, 10)

        # ---- Publishers ----
        self.players_pub = self.create_publisher(Players, 'players', 10)
        self.warped_pub = self.create_publisher(Image, 'board/warped_image', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'aruco_markers', 10)

        # ---- TF ----
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Broadcast static transforms ----
        broadcast_camera_to_world(self)
        self.broadcast_static_board()

        self.get_logger().info("🧩 ArUco simulator started (no depth camera).")

    # ----------------------------------------------------------
    def broadcast_static_board(self):
        """Broadcast a fixed board_frame below the camera."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_frame'
        t.child_frame_id = 'board_frame'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = -0.9  # 0.9 m below
        q = quaternion_from_euler(0, 0, np.deg2rad(30))
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.static_broadcaster.sendTransform(t)
        self.get_logger().info("📡 Broadcasted static board_frame (z=-0.9 m, yaw=30°).")

    # ----------------------------------------------------------
    def color_callback(self, msg: Image):
        """Main ArUco detection pipeline (2D-only)."""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = self.detector.detectMarkers(frame)

        players_msg = Players()
        marker_array = MarkerArray()

        if ids is not None:
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners)

            # --- Draw and process markers
            for i, marker_id in enumerate(ids):
                c = corners[i][0]
                center_px = tuple(c.mean(axis=0).astype(int))
                cv2.putText(frame, str(marker_id), center_px,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                # --- Convert pixel to board/world (z=0)
                point_world = pixel_to_world_pose(
                    self.tf_buffer,
                    x_px=center_px[0],
                    y_px=center_px[1],
                    z_offset=0.0,
                    img_w=frame.shape[1],
                    img_h=frame.shape[0],
                    node=self
                )

                # --- Visualization marker
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = msg.header.stamp
                marker.ns = "aruco"
                marker.id = int(marker_id)
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position = point_world.point
                marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.01
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker_array.markers.append(marker)

                # --- Player message
                player_msg = Player()
                player_msg.player_id = str(marker_id)
                player_msg.position = 1
                players_msg.players.append(player_msg)

            # --- Warp board if 0–3 detected
            board_points = self.get_board_corners(ids, corners)
            if len(board_points) == 4:
                self.publish_warped_board(frame, board_points, msg.header.stamp)

        # --- Publish
        self.players_pub.publish(players_msg)
        self.marker_pub.publish(marker_array)

        if self.show_image:
            cv2.imshow("Aruco Detection (Sim)", frame)
            cv2.waitKey(1)

    # ----------------------------------------------------------
    def get_board_corners(self, ids, corners):
        """Collect ArUco marker corners (IDs 0-3) for warp."""
        board_pts = {}
        for i, marker_id in enumerate(ids):
            if marker_id in [0, 1, 2, 3]:
                # Average corner positions
                c = corners[i][0]
                board_pts[marker_id] = np.mean(c, axis=0)
        if len(board_pts) != 4:
            return []
        # Order: top-left, top-right, bottom-right, bottom-left
        return [board_pts[i] for i in [0, 1, 2, 3]]

    # ----------------------------------------------------------
    def publish_warped_board(self, frame, board_points, stamp):
        """Warp board to top-down using detected ArUco corners."""
        try:
            board_h_mm, board_w_mm = 390, 756
            src_pts = np.array(board_points, dtype=np.float32)
            dst_pts = np.array([[0, 0],
                                [board_w_mm - 1, 0],
                                [board_w_mm - 1, board_h_mm - 1],
                                [0, board_h_mm - 1]], dtype=np.float32)
            H, _ = cv2.findHomography(src_pts, dst_pts)
            warped = cv2.warpPerspective(frame, H, (int(board_w_mm), int(board_h_mm)))

            warped_msg = self.bridge.cv2_to_imgmsg(warped, encoding='bgr8')
            warped_msg.header.stamp = stamp
            self.warped_pub.publish(warped_msg)

            if self.show_image:
                cv2.imshow('Board Warped', warped)
        except Exception as e:
            self.get_logger().warn(f"warp_board failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorSim()
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
