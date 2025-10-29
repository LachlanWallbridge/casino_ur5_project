#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray
from custom_interface.srv import Cup  # Temporary, we’ll make custom srv later
from cv_bridge import CvBridge
import cv2
import numpy as np


class CupDetector(Node):
    def __init__(self):
        super().__init__('cup_detector')

        # --- Subscribers ---
        self.image_sub = self.create_subscription(
            Image,
            "board/warped_image",
            self.image_callback,
            10
        )

        # --- Service Server ---
        self.srv = self.create_service(Cup, 'cup_detector/cupService', self.service_callback)

        # --- Publisher ---
        self.marker_pub = self.create_publisher(MarkerArray, 'cup_detector/markers', 10)

        # --- Variables ---
        self.bridge = CvBridge()
        self.raw_warp = None
        self.image_active = True

        self.get_logger().info("CupDetector node initialized.")

    def image_callback(self, msg):
        """Callback for image topic; saves latest image if allowed."""
        if self.image_active:
            try:
                self.raw_warp = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f"CV Bridge error: {e}")

    def service_callback(self, request, response):
        """When service is called, stop image updates, process image, and send response."""
        self.get_logger().info("Service request received. Pausing image updates...")

        # Pause image updates
        self.image_active = False

        # Check image availability
        if self.raw_warp is None:
            self.get_logger().warn("No image received yet.")
            response.success = False
            response.message = "No image available."
            self.image_active = True
            return response

        # --- Placeholder for cup detection ---
        x, y, width, height, pose = self.cupdetection(self.raw_warp)

        # --- Publish marker array once ---
        self.publish_marker_array(x, y, width, height, pose)

        # --- Resume image updates ---
        self.image_active = True

        # --- Response ---
        response.success = True
        response.message = f"Cup detected at x={x}, y={y}"
        self.get_logger().info("Response sent.")
        return response

    def cupdetection(self, image):
        """Placeholder for your cup detection algorithm."""
        # TODO: Replace with real detection logic
        h, w, _ = image.shape
        x, y = int(w / 3), int(h / 3)
        width, height = 100, 120

        # Dummy pose
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.2
        pose.position.z = 0.0
        pose.orientation.w = 1.0



        # width = self.raw_warp.shape[1]
        # warped = self.raw_warp[0:190,100:(width-100)]

        # cv2.imshow("Warped", warped)
        # hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

        # # Define range for yellow
        # lower_yellow = np.array([40, 50, 100])
        # upper_yellow = np.array([60, 100, 255])

        # # Create a mask for yellow
        # mask = cv2.inRange(hsv, lower_yellow, upper_yellow)


        return x, y, width, height, pose

    def publish_marker_array(self, x, y, width, height, pose):
        """Publish one-time marker array to visualize detection box."""
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cup_detector"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = width / 1000.0  # scale to meters if needed
        marker.scale.y = height / 1000.0
        marker.scale.z = 0.1
        marker.color.a = 0.8
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        self.get_logger().info("Marker published to RViz.")


def main(args=None):
    rclpy.init(args=args)
    node = CupDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down CupDetector node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
