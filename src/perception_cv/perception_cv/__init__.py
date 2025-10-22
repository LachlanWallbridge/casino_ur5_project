import rclpy
from rclpy.node import Node
import numpy as np
import tf2_ros
import geometry_msgs.msg
import tf_transformations


def broadcast_camera_to_world(node: Node,
                              translation=(1.0, 0.6, 0.9),
                              euler_rotation=(-np.pi/2, 0, 0),
                              parent_frame='world',
                              child_frame='camera_frame'):
    """
    Broadcast a static transform from world → camera_frame.
    All nodes in the package can use this frame.
    """
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster(node)

    t = geometry_msgs.msg.TransformStamped()
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
