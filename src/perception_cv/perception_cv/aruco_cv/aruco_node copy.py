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
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion

# Import the reusable static broadcaster
from perception_cv import broadcast_camera_to_world, pixel_to_world_pose, CAM_HEIGHT_M


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
        self.marker_pub = self.create_publisher(MarkerArray, 'aruco_markers', 10)

        # ---- TF broadcasters ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
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

            # Publish ArUco markers as RViz markers
            self.publish_aruco_markers(ids, corners, frame, color_msg.header.stamp)

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
            z = CAM_HEIGHT_M
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
        """Broadcast board_frame assuming camera is directly above (downward facing)."""
        self.get_logger().info("Broadcasting board_frame TF.")

        # Convert points to numpy (each is geometry_msgs/Point)
        src = np.array([[p.x, p.y] for p in board_points], dtype=float)
        center = np.mean(src, axis=0)

        # --- Compute yaw from board corner positions ---
        # vec_x: top-left → top-right (defines board X axis in image plane)
        vec_x = src[1] - src[0]
        yaw = np.arctan2(vec_x[1], vec_x[0])

        # --- Quaternion: only yaw rotation ---
        q = quaternion_from_euler(0.0, 0.0, 0.0)

        # --- Fill transform ---
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_frame'
        t.child_frame_id = 'board_frame'

        # Board position relative to camera
        t.transform.translation.x = float(center[0])
        t.transform.translation.y = float(-center[1])
        t.transform.translation.z = float(-CAM_HEIGHT_M)

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

    # ----------------------------------------------------------
    def publish_aruco_markers(self, ids, corners, frame, stamp):
        """Publish ArUco detections as RViz markers in the world frame."""
        if ids is None:
            return

        marker_array = MarkerArray()

        for i, marker_id in enumerate(ids):
            c = corners[i][0]
            center_px = tuple(c.mean(axis=0).astype(int))

            # --- Convert pixel to world ---
            point_world = pixel_to_world_pose(
                self.tf_buffer,
                x_px=center_px[0],
                y_px=center_px[1],
                z_offset=0.0,
                img_w=frame.shape[1],
                img_h=frame.shape[0],
                node=self
            )

            # --- CUBE marker ---
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = stamp
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

            # --- TEXT marker for ID ---
            text_marker = Marker()
            text_marker.header.frame_id = "world"
            text_marker.header.stamp = stamp
            text_marker.ns = "aruco_text"
            text_marker.id = int(marker_id) + 1000  # avoid ID collision
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position = point_world.point
            text_marker.pose.position.z += 0.05  # lift above cube
            text_marker.scale.z = 0.06  # text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = str(marker_id)
            marker_array.markers.append(text_marker)

        # --- Publish all markers ---
        self.marker_pub.publish(marker_array)



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
