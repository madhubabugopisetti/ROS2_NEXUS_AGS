#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class AGSDetectBox(Node):
    def __init__(self):
        super().__init__("ags_box_vision")
        self.sub = self.create_subscription(Image, "/camera/image", self.cb, 10)
        self.bridge = CvBridge()
        self.get_logger().info("AGS box vision started")

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Red mask (same logic as your AMR)
        lower1 = (0, 120, 70)
        upper1 = (10, 255, 255)
        lower2 = (170, 120, 70)
        upper2 = (180, 255, 255)

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = mask1 | mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            cv2.imshow("ags_box", img)
            cv2.waitKey(1)
            return

        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)

        # Draw green box
        cv2.rectangle(img, (x, y), (x+w, y+h), (0,255,0), 2)

        # Center lines
        cx = x + w//2
        img_center = img.shape[1]//2
        cv2.line(img, (cx, 0), (cx, img.shape[0]), (255,0,0), 2)
        cv2.line(img, (img_center, 0), (img_center, img.shape[0]), (0,255,255), 2)

        cv2.imshow("ags_box", img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = AGSDetectBox()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
