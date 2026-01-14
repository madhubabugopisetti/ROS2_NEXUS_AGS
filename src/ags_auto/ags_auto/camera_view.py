#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionAlignUI(Node):
    def __init__(self):
        super().__init__("vision_align_ui")
        self.sub = self.create_subscription(Image, "/camera/image", self.image_cb, 10)
        self.bridge = CvBridge()
        self.get_logger().info("Vision Align UI started")

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower1 = (0, 120, 70)
        upper1 = (10, 255, 255)
        lower2 = (170, 120, 70)
        upper2 = (180, 255, 255)

        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        H, W, _ = img.shape
        icx = W // 2
        icy = H // 2

        # draw image center axes
        cv2.line(img, (icx, 0), (icx, H), (0, 0, 255), 2)   # vertical
        cv2.line(img, (0, icy), (W, icy), (0, 255, 0), 2)   # horizontal

        if contours:
            cnt = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w // 2
            cy = y + h // 2

            # draw box
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # draw center dot
            cv2.circle(img, (cx, cy), 6, (0, 255, 0), -1)

            err_x = cx - icx
            err_y = cy - icy

            cv2.putText(
                img,
                f"err_x: {err_x}  err_y: {err_y}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

        cv2.imshow("Vision Align", img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = VisionAlignUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
