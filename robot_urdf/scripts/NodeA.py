#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from ros2_aruco_interfaces.msg import ArucoMarkers

from cv_bridge import CvBridge
import cv2


class MarkerNavigator(Node):

    def __init__(self):
        super().__init__('marker_navigator')

        # ---------------- Publishers ----------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_pub = self.create_publisher(Image, '/results_images', 10)

        # ---------------- Subscribers ----------------
        self.create_subscription(
            ArucoMarkers, '/aruco_markers', self.marker_callback, 10
        )
        self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # ---------------- Timer ----------------
        self.timer = self.create_timer(0.1, self.control_loop)

        # ---------------- State ----------------
        self.mode = "SCAN"                  
        self.detected_ids = set()
        self.marker_sequence = []
        self.current_index = 0
        self.target_id = None

        self.align_counter = 0               # stability counter

        self.latest_image = None
        self.latest_markers = None

        self.bridge = CvBridge()

        self.get_logger().info("Marker Navigator started")

    
    # CONTROL LOOP
    
    def control_loop(self):
        cmd = Twist()

        if self.mode == "SCAN":
            cmd.angular.z = 0.20

        elif self.mode == "ALIGN":
            self.align_to_marker(cmd)

        elif self.mode == "DONE":
            cmd = Twist()

        self.cmd_pub.publish(cmd)

    
    # MARKER CALLBACK
    
    def marker_callback(self, msg: ArucoMarkers):
        self.latest_markers = msg

        if self.mode == "SCAN" and msg.marker_ids:
            for mid in msg.marker_ids:
                self.detected_ids.add(mid)

            self.get_logger().info(
                f"Scanning... detected IDs: {sorted(self.detected_ids)}"
            )

            EXPECTED_MARKERS = 5
            if len(self.detected_ids) >= EXPECTED_MARKERS:
                self.marker_sequence = sorted(self.detected_ids)
                self.current_index = 0
                self.target_id = self.marker_sequence[self.current_index]
                self.mode = "ALIGN"

                self.get_logger().info(
                    f"ALL markers found! Starting alignment sequence: "
                    f"{self.marker_sequence}"
                )

    
    # ALIGN LOGIC
    
    def align_to_marker(self, cmd: Twist):

        if self.latest_markers is None:
            return

        if self.target_id not in self.latest_markers.marker_ids:
            cmd.angular.z = 0.18
            return

        idx = self.latest_markers.marker_ids.index(self.target_id)
        pose = self.latest_markers.poses[idx]

        # --- Errors ---
        error_x = pose.position.x          # left / right
        error_z = pose.position.z - 0.25    # desired distance

        # --- Control gains ---
        KP_ANG = 1.2
        KP_LIN = 0.6

        cmd.angular.z = -KP_ANG * error_x
        cmd.linear.x  =  KP_LIN * error_z

        # Clamp angular speed
        cmd.angular.z = max(min(cmd.angular.z, 0.08), -0.08)

        # --- Alignment confirmation ---
        if abs(error_x) < 0.03 and abs(error_z) < 0.10:
            self.align_counter += 1
        else:
            self.align_counter = 0

        REQUIRED_FRAMES = 8

        if self.align_counter >= REQUIRED_FRAMES:
            self.align_counter = 0
            self.get_logger().info(
                f"Marker {self.target_id} aligned."
            )

            self.current_index += 1

            if self.current_index >= len(self.marker_sequence):
                self.mode = "DONE"
                self.get_logger().info("ALL markers processed. DONE.")
                return
            else:
                self.target_id = self.marker_sequence[self.current_index]
                self.get_logger().info(
                    f"Moving to next marker ID {self.target_id}"
                )

    
    # IMAGE CALLBACK
    
    def image_callback(self, msg: Image):
        self.latest_image = msg

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = frame.shape

        # Slight upward offset for better visual centering
        vertical_offset = int(0.05 * h)
        image_center = (w // 2, h // 2 - vertical_offset)

        cv2.circle(frame, image_center, 35, (0, 0, 255), 3)

        cv2.putText(
            frame,
            f"MODE: {self.mode}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2
        )

        if self.target_id is not None:
            cv2.putText(
                frame,
                f"TARGET ID: {self.target_id}",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 0, 255),
                2
            )

        out = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out.header = msg.header
        self.image_pub.publish(out)



# MAIN

def main():
    rclpy.init()
    node = MarkerNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
