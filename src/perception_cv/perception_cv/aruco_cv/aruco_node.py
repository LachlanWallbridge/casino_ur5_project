#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from custom_interface.msg import Player, Players
from geometry_msgs.msg import TransformStamped, Point
import tf2_ros
from tf_transformations import quaternion_from_euler

# Import the reusable static broadcaster
from perception_cv import broadcast_camera_to_world


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ---- Parameters ----
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('show_image', True)
        self.show_image = self.get_parameter('show_image').get_parameter_value().bool_value

        # ---- CV / ArUco ----
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # ---- Subscribers ----
        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value

        self.color_sub = self.create_subscription(Image, color_topic, self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10
        )

        # ---- Publishers ----
        self.players_pub = self.create_publisher(Players, 'players', 10)
        self.warped_pub = self.create_publisher(Image, 'board/warped_image', 10)

        # ---- TF broadcasters ----
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---- Broadcast static camera → world ----
        broadcast_camera_to_world(self)

        # ---- State ----
        self.latest_color = None
        self.latest_depth = None
        self.fx = self.fy = self.cx = self.cy = None

        self.get_logger().info("ArUco detector node started.")

    # ------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def color_callback(self, msg: Image):
        self.latest_color = msg
        self.try_process_frame()

    def depth_callback(self, msg: Image):
        self.latest_depth = msg
        self.try_process_frame()

    # ------------------------------
    def try_process_frame(self):
        if self.latest_color is None or self.latest_depth is None:
            return
        if None in [self.fx, self.fy, self.cx, self.cy]:
            return

        color_msg = self.latest_color
        depth_msg = self.latest_depth
        frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough').astype(np.float64) / 1000.0  # mm → m

        corners, ids, _ = self.detector.detectMarkers(frame)
        players_msg = Players()

        if ids is not None:
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners)

            # Draw marker IDs
            for i, marker_id in enumerate(ids):
                c = corners[i][0]
                center_px = tuple(c.mean(axis=0).astype(int))
                cv2.putText(frame, str(marker_id), center_px,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # Detect board corners (markers 0-3)
            board_points = self.get_board_corners(ids, corners, depth_image)
            board_points2D = self.get_board_corners2D(ids, corners)
            if len(board_points) == 4:
                # Broadcast dynamic board_frame
                self.broadcast_board_frame(board_points, color_msg.header.stamp)

                # self.get_logger().info(f"Board detected with markers 0-3. {board_points}")
                # Warp image for visualization
                self.publish_warped_board(frame, board_points2D, color_msg.header.stamp)

                # Process non-board markers as players
                for i, marker_id in enumerate(ids):
                    if marker_id not in [0, 1, 2, 3]:
                        player_msg = Player()
                        player_msg.player_id = str(marker_id)
                        player_msg.position = 1
                        players_msg.players.append(player_msg)

        # Publish players
        self.players_pub.publish(players_msg)

        if self.show_image:
            cv2.imshow('Aruco Detection', frame)
            cv2.waitKey(1)

    # ------------------------------
    def get_board_corners(self, ids, corners, depth_image):
        """Compute 3D camera-frame positions of the board markers (0-3)."""
        board_pts = {}
        for i, marker_id in enumerate(ids):
            if marker_id in [0, 1, 2, 3]:
                pts_3d = self.get_marker_3d_position(corners[i][0], depth_image)
                if pts_3d is not None:
                    board_pts[marker_id] = pts_3d

        if len(board_pts) != 4:
            return []

        # Order: top-left, top-right, bottom-right, bottom-left
        return [board_pts[i] for i in [0, 1, 2, 3]]

    def get_board_corners2D(self, ids, corners):
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

    # ------------------------------
    def get_marker_3d_position(self, marker_corners, depth_image):
        """Compute 3D camera-frame position of a marker by averaging its corners."""
        points_3d = []
        for corner in marker_corners:
            u, v = int(corner[0]), int(corner[1])
            z = depth_image[v, u]
            if z == 0.0:
                continue
            x = (u - self.cx) * z / self.fx
            y = (v - self.cy) * z / self.fy
            points_3d.append((x, y, z))
        if not points_3d:
            return None
        x, y, z = np.mean(points_3d, axis=0)
        return Point(x=x, y=y, z=z)

    # ------------------------------
    def broadcast_board_frame(self, board_points, stamp):
        """Broadcast board_frame using averaged board marker positions."""
        src = np.array([[p.x, p.y, p.z] for p in board_points])
        center = np.mean(src, axis=0)

        # Compute approximate yaw (rotation around Z)
        vec_x = src[1] - src[0]  # top-left → top-right
        yaw = np.arctan2(vec_x[1], vec_x[0])

        q = quaternion_from_euler(0, 0, yaw)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'camera_frame'
        t.child_frame_id = 'board_frame'
        t.transform.translation.x = float(center[0])
        t.transform.translation.y = float(center[1])
        t.transform.translation.z = float(center[2]) * -1.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    # ------------------------------
    def publish_warped_board(self, frame, board_points, stamp):
        """Warp board to top-down using detected ArUco corners."""
        try:
            board_h_mm, board_w_mm = 390, 765
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
