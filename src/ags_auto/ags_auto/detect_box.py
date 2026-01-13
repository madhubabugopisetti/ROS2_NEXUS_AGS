#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import cv2
import numpy as np

class AlignShoulder(Node):
    def __init__(self):
        super().__init__("align_shoulder")

        self.bridge = CvBridge()

        self.create_subscription(Image, "/camera/image", self.image_cb, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_cb, 10)

        self.joint_positions = {}

        self.get_logger().info("align_shoulder started (STEP 1: vision + joints only)")

    def joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = img.shape

        # --- detect red box ---
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower1 = (0, 120, 70)
        upper1 = (10, 255, 255)
        lower2 = (170, 120, 70)
        upper2 = (180, 255, 255)

        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cx = cy = None

        if contours:
            c = max(contours, key=cv2.contourArea)
            x, y, bw, bh = cv2.boundingRect(c)

            # draw green box
            cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 255, 0), 2)

            cx = x + bw // 2
            cy = y + bh // 2

            cv2.circle(img, (cx, cy), 5, (0, 255, 0), -1)

        # --- draw axes ---
        cv2.line(img, (w // 2, 0), (w // 2, h), (0, 255, 0), 2)   # Y axis (green)
        cv2.line(img, (0, h // 2), (w, h // 2), (0, 0, 255), 2)   # X axis (red)

        # --- print info ---
        if cx is not None:
            err_x = cx - (w // 2)
            err_y = cy - (h // 2)

            shoulder = self.joint_positions.get("shoulder_joint", 0.0)

            self.get_logger().info(
                f"Box: ({cx},{cy})  Err: ({err_x},{err_y})  Shoulder: {shoulder:.3f}"
            )

        cv2.imshow("align_shoulder", img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = AlignShoulder()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
