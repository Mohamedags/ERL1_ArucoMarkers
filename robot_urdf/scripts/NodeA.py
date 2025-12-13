#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from ros2_aruco_interfaces.msg import ArucoMarkers

from cv_bridge import CvBridge
import cv2


class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('node_a')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_pub = self.create_publisher(Image, '/results_images', 10)

        # Subscriptions
        self.create_subscription(
            ArucoMarkers, '/aruco_markers', self.marker_callback, 10
        )
        self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # State
        self.scanning = True
        self.finished = False

        self.latest_image = None
        self.bridge = CvBridge()

        self.get_logger().info("NodeA started")

    # --------------------------------------------------

    def control_loop(self):
        if self.finished:
            self.cmd_pub.publish(Twist())
            return

        if self.scanning:
            cmd = Twist()
            cmd.angular.z = 0.3
            self.cmd_pub.publish(cmd)

    # --------------------------------------------------

    def marker_callback(self, msg: ArucoMarkers):

        if self.latest_image is None:
            return

        if len(msg.marker_ids) == 0:
            return

        self.get_logger().info(f"Marker {msg.marker_ids[0]} checked")

        frame = self.bridge.imgmsg_to_cv2(
            self.latest_image, desired_encoding='bgr8'
        )

        h, w, _ = frame.shape
        center = (w // 2, h // 2)

        # Visual confirmation
        cv2.circle(frame, center, 80, (0, 0, 255), 5)
        cv2.putText(
            frame,
            "ARUCO DETECTED",
            (30, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2
        )

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_msg.header = self.latest_image.header
        self.image_pub.publish(out_msg)

    # --------------------------------------------------

    def image_callback(self, msg: Image):
        self.latest_image = msg


def main():
    rclpy.init()
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
