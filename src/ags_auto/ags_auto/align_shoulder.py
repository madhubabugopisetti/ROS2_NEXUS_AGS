#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np

class AGSAlignShoulder(Node):
    def __init__(self):
        super().__init__("ags_align_shoulder")

        # PID
        self.Kp = 0.001
        self.Ki = 0.0000
        self.Kd = 0.0005

        self.integral = 0.0
        self.prev_err = 0.0

        self.max_step = 0.08

        # Subs
        self.sub_img = self.create_subscription(Image, "/camera/image", self.image_cb, 10)
        self.sub_js = self.create_subscription(JointState, "/joint_states", self.joint_cb, 10)

        # Pub
        self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)

        self.bridge = CvBridge()

        # Joint values
        self.shoulder = 0.0
        self.elbow = 0.0
        self.forearm = 0.0
        self.wrist_pitch = -1.2
        self.wrist_roll = 0.0
        self.left_finger = 0.01
        self.right_finger = 0.01

        self.joint_ready = False

        # Control
        self.step = 0.05
        self.deadzone = 5
        self.prev_error = None
        self.dir = 1

        self.get_logger().info("AGS align SHOULDER ONLY started")

    def joint_cb(self, msg):
        if "shoulder_joint" in msg.name:
            self.shoulder = msg.position[msg.name.index("shoulder_joint")]
        if "elbow_joint" in msg.name:
            self.elbow = msg.position[msg.name.index("elbow_joint")]
        if "forearm_joint" in msg.name:
            self.forearm = msg.position[msg.name.index("forearm_joint")]

        self.joint_ready = True

    def image_cb(self, msg):
        if not self.joint_ready:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        H, W, _ = img.shape
        icx = W // 2
        icy = H // 2

        # Draw image center axes
        cv2.line(img, (icx, 0), (icx, H), (0, 0, 255), 2)   # vertical (X)
        cv2.line(img, (0, icy), (W, icy), (0, 255, 0), 2)  # horizontal (Y)

        # Detect red box
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower1 = (0, 120, 70)
        upper1 = (10, 255, 255)
        lower2 = (170, 120, 70)
        upper2 = (180, 255, 255)

        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            cv2.imshow("align_shoulder", img)
            cv2.waitKey(1)
            return

        # Biggest contour
        c = max(contours, key=cv2.contourArea)
        x, y, bw, bh = cv2.boundingRect(c)

        # Draw box
        cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 255, 0), 2)

        box_cx = x + bw // 2
        box_cy = y + bh // 2

        # Draw center dot
        cv2.circle(img, (box_cx, box_cy), 5, (0, 255, 0), -1)

        # Errors
        err_x = box_cx - icx
        err_y = box_cy - icy

        # Show errors
        cv2.putText(
            img,
            f"err_x: {err_x}  err_y: {err_y}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        # ========== YOUR CONTROL USES Y ERROR ==========
        err = err_y

        # Check centered
        if abs(err) < self.deadzone:
            self.get_logger().info("Y CENTERED. Stopping shoulder.")
            cv2.imshow("align_shoulder", img)
            cv2.waitKey(1)
            return

        # PID
        self.integral += err
        derivative = err - self.prev_err
        self.prev_err = err

        u = self.Kp * err + self.Ki * self.integral + self.Kd * derivative

        # Clamp
        if u > self.max_step:
            u = self.max_step
        if u < -self.max_step:
            u = -self.max_step

        # Direction safety (your old logic)
        if self.prev_error is not None:
            if abs(err) > self.prev_error:
                self.dir *= -1

        self.prev_error = abs(err)

        # Apply
        self.shoulder += u * self.dir

        self.send_joints()


        cv2.imshow("align_shoulder", img)
        cv2.waitKey(1)

    def send_joints(self):
        # ONLY SHOULDER MOVES

        traj = JointTrajectory()
        traj.joint_names = [
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
            self.shoulder,
            self.elbow,
            self.forearm,
            self.wrist_pitch,
            self.wrist_roll,
            self.left_finger,
            self.right_finger
        ]

        p.time_from_start.sec = 1
        traj.points.append(p)
        self.pub.publish(traj)

def main():
    rclpy.init()
    node = AGSAlignShoulder()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
