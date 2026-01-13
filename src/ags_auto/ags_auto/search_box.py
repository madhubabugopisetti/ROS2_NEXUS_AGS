#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np

class AGSSearchBox(Node):
    def __init__(self):
        super().__init__("ags_search_box")
        self.sub = self.create_subscription(Image, "/camera/image", self.image_cb, 10)
        self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.bridge = CvBridge()
        self.shoulder = 0.0
        self.box_found = False
        self.timer = self.create_timer(0.3, self.control_loop)
        self.get_logger().info("AGS search box started")

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower1 = (0, 120, 70)
        upper1 = (10, 255, 255)
        lower2 = (170, 120, 70)
        upper2 = (180, 255, 255)
        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.box_found = len(contours) > 0

    def send_joints(self, shoulder):
        msg = JointTrajectory()
        msg.joint_names = [
            "shoulder_joint",
            "elbow_joint",
            "forearm_joint",
            "wrist_pitch_joint",
            "wrist_roll_joint",
            "left_finger_joint",
            "right_finger_joint",
        ]
        p = JointTrajectoryPoint()
        p.positions = [
            shoulder,
            -0.5,    # elbow
            -1.2,    # forearm
            -1.2,   # wrist_pitch DOWN
            0.0,    # wrist_roll
            0.01,
            0.01
        ]
        p.time_from_start.sec = 1

        msg.points.append(p)
        self.pub.publish(msg)

    def control_loop(self):
        if not self.box_found:
            self.shoulder += 0.1
            if self.shoulder > 3.14:
                self.shoulder = -3.14

            self.get_logger().info(f"Searching... shoulder = {self.shoulder:.2f}")
            self.send_joints(self.shoulder)
        else:
            self.get_logger().info("BOX FOUND! Stopping.")
            self.timer.cancel()

def main():
    rclpy.init()
    node = AGSSearchBox()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
