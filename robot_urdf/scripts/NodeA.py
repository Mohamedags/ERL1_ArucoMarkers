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

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_pub = self.create_publisher(Image, '/results_images', 10)

        self.create_subscription(
            ArucoMarkers, '/aruco_markers', self.marker_callback, 10
        )
        self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        # STATE
        self.mode = "SCAN"      # SCAN → ALIGN → DONE
        self.detected_ids = set()
        self.target_id = None

        self.latest_image = None
        self.bridge = CvBridge()

        self.get_logger().info("NodeA started")

    # --------------------------------------------------

    def control_loop(self):
        cmd = Twist()
        if self.mode == "SCAN":
            cmd.angular.z = 0.3
        self.cmd_pub.publish(cmd)

    # --------------------------------------------------

    def marker_callback(self, msg: ArucoMarkers):

        if self.latest_image is None or not msg.marker_ids:
            return

        frame = self.bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
        h, w, _ = frame.shape
        center = (w // 2, h // 2)

        # ================= SCAN =================
        if self.mode == "SCAN":
            for mid in msg.marker_ids:
                self.detected_ids.add(mid)

            self.get_logger().info(
                f"Scanning... detected IDs: {sorted(self.detected_ids)}"
            )

            # Visual feedback
            cv2.circle(frame, center, 40, (0, 0, 255), 3)
            cv2.putText(
                frame, "SCANNING",
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (0, 0, 255), 2
            )

            if len(self.detected_ids) >= 5:
                self.target_id = min(self.detected_ids)
                self.mode = "ALIGN"
                self.cmd_pub.publish(Twist())
                self.get_logger().info(
                    f"ALL markers found! Aligning to ID {self.target_id}"
                )

        # ================= ALIGN =================
        elif self.mode == "ALIGN":

            if self.target_id not in msg.marker_ids:
                cmd = Twist()
                cmd.angular.z = 0.2
                self.cmd_pub.publish(cmd)
            else:
                idx = msg.marker_ids.index(self.target_id)
                pose = msg.poses[idx]

                error = pose.position.x   # meters

                cmd = Twist()
                cmd.angular.z = -1.5 * error
                self.cmd_pub.publish(cmd)

                cv2.circle(frame, center, 50, (0, 255, 0), 4)
                cv2.putText(
                    frame,
                    f"ALIGNING ID {self.target_id}",
                    (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (0, 255, 0), 2
                )

                if abs(error) < 0.02:
                    self.mode = "DONE"
                    self.cmd_pub.publish(Twist())
                    self.get_logger().info(
                        f"Marker {self.target_id} centered. DONE."
                    )

        out = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out.header = self.latest_image.header
        self.image_pub.publish(out)

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
