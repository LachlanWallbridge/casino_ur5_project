import rclpy
from rclpy.node import Node
import tf2_ros
import tf_transformations
import numpy as np
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PointStamped
import math
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.time import Time


CAM_HEIGHT_M = 0.92-0.07  # 0.9 meters height of camera above world frame

# Previous, x=0.85,0.19,0.92-0.07

# New z = 895mm, x = 840mm, y= 176mm
def broadcast_camera_to_world(node: Node,
                              translation=(0.840, 0.176, 0.895),
                              euler_rotation=(math.pi, 0, math.pi),
                              parent_frame='world',
                              child_frame='camera_frame'):
    """
    Broadcast a static transform from world → camera_frame.
    All nodes in the package can use this frame.
    """
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster(node)

    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame

    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]

    q = tf_transformations.quaternion_from_euler(*euler_rotation)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    tf_broadcaster.sendTransform(t)
    return tf_broadcaster

# 1 pixel = 1 mm = 0.001 m
PIXEL_TO_METERS = 0.001
BOARD_H_MM = 390
BOARD_W_MM = 765

def pixel_to_board_coords(x_px: float, y_px: float, img_w: int, img_h: int, z_offset: float):
    """
    Convert pixel coordinates (x_px, y_px) in the warped board image to board frame (meters).
    The board frame origin is at the image center, with:
      +X → right, +Y → up (note inverted OpenCV Y), +Z → normal to board.
    """
    x_board_m = (x_px - img_w / 2.0) * PIXEL_TO_METERS
    y_board_m = -(y_px - img_h / 2.0) * PIXEL_TO_METERS  # Flip Y to make up positive
    z_board_m = z_offset

    return x_board_m, y_board_m, z_board_m


from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import rclpy.time
import rclpy.duration


def pixel_to_world_pose(
    tf_buffer: tf2_ros.Buffer,
    x_px: float,
    y_px: float,
    z_offset: float,
    img_w: int,
    img_h: int,
    yaw_rad: float = 0.0,
    source_frame: str = "board_frame",
    target_frame: str = "world",
    timeout_s: float = 2.0,
    node=None
):
    """
    Convert pixel coordinates + yaw into a 6-DoF PoseStamped in the world frame.

    - Computes (x, y, z) in board_frame using pixel_to_board_coords().
    - Creates a PoseStamped in board_frame.
    - Applies orientation: roll=0, pitch=0, yaw=yaw_rad.
    - Transforms full pose to target_frame via TF.
    """

    # ---------------------------------------------------------
    # 1) Convert pixel -> board coordinates (position only)
    # ---------------------------------------------------------
    bx, by, bz = pixel_to_board_coords(x_px, y_px, img_w, img_h, z_offset)

    # ---------------------------------------------------------
    # 2) Build PoseStamped in board_frame
    # ---------------------------------------------------------
    pose_board = PoseStamped()
    pose_board.header.frame_id = source_frame
    pose_board.header.stamp = rclpy.time.Time().to_msg()

    pose_board.pose.position.x = bx
    pose_board.pose.position.y = by
    pose_board.pose.position.z = bz

    # Orientation: roll=0, pitch=0, yaw=yaw_rad (cup orientation)
    q = quaternion_from_euler(0.0, 0.0, yaw_rad)
    pose_board.pose.orientation.x = float(q[0])
    pose_board.pose.orientation.y = float(q[1])
    pose_board.pose.orientation.z = float(q[2])
    pose_board.pose.orientation.w = float(q[3])

    # ---------------------------------------------------------
    # 3) Transform entire pose to world frame
    # ---------------------------------------------------------
    try:
        pose_world = tf_buffer.transform(
            pose_board,
            target_frame,
            timeout=rclpy.duration.Duration(seconds=timeout_s),
            new_type=PoseStamped,
        )
        return pose_world

    except Exception as e:
        if node:
            node.get_logger().warn(f"⚠️ TF transform PoseStamped failed: {e}")
        return None

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import rclpy.time
import rclpy.duration

def visualise_pose_in_rviz(
    node,
    poses,
    marker_pub,
    scale=0.1,
    lifetime=0.0,
    ns="ee_goal",
    start_id=0
):
    """
    Visualise one or many PoseStamped poses in RViz as coordinate frames.

    Args:
        node: rclpy node (logging)
        poses: PoseStamped OR list of PoseStamped
        marker_pub: Publisher(MarkerArray)
        scale: axis length
        lifetime: marker lifetime seconds (0 = forever)
        ns: namespace for RViz
        start_id: starting marker ID (unique per call)
    """

    # Normalise input to list
    if not isinstance(poses, (list, tuple)):
        poses = [poses]

    arr = MarkerArray()
    next_id = start_id

    # -------------------------------------------
    # For each PoseStamped in the list
    # -------------------------------------------
    for pose_stamped in poses:

        base_pose = pose_stamped.pose

        # Define local axis rotations
        axis_defs = [
            ("x_axis", (1.0, 0.0, 0.0, 1.0), quaternion_from_euler(0, 0, 0)),             # X axis
            ("y_axis", (0.0, 1.0, 0.0, 1.0), quaternion_from_euler(0, 0, math.pi/2)),     # Y axis
            ("z_axis", (0.0, 0.0, 1.0, 1.0), quaternion_from_euler(0, -math.pi/2, 0)),    # Z axis
        ]

        # For X, Y, Z arrows
        for axis_name, color, q_rot in axis_defs:

            m = Marker()
            m.header = pose_stamped.header
            m.ns = ns
            m.id = next_id
            next_id += 1

            m.type = Marker.ARROW
            m.action = Marker.ADD

            # Position
            m.pose.position = base_pose.position

            # Orientation = EE orientation * axis-rotation
            q_ee = [
                base_pose.orientation.x,
                base_pose.orientation.y,
                base_pose.orientation.z,
                base_pose.orientation.w,
            ]
            q_final = tf_transformations.quaternion_multiply(q_ee, q_rot)

            m.pose.orientation.x = q_final[0]
            m.pose.orientation.y = q_final[1]
            m.pose.orientation.z = q_final[2]
            m.pose.orientation.w = q_final[3]

            # Arrow dimensions
            m.scale.x = scale
            m.scale.y = scale * 0.1
            m.scale.z = scale * 0.1

            # Color
            m.color.r = color[0]
            m.color.g = color[1]
            m.color.b = color[2]
            m.color.a = color[3]

            arr.markers.append(m)

        # Logging for each pose
        node.get_logger().info(
            f"[RViz] Pose axes at ({base_pose.position.x:.3f}, "
            f"{base_pose.position.y:.3f}, {base_pose.position.z:.3f})"
        )

    # Publish whole array
    marker_pub.publish(arr)
