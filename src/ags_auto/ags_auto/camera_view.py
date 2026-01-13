#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class AGSCameraView(Node):
    def __init__(self):
        super().__init__("ags_camera_view")
        self.sub = self.create_subscription(Image, "/camera/image", self.cb, 10)
        self.bridge = CvBridge()
        self.get_logger().info("AGS camera view started")

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("ags_camera", img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = AGSCameraView()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
