#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class AGSScanAlign(Node):
    def __init__(self):
        super().__init__("ags_scan_align")

        # === SAME GAINS AS YOUR WORKING SERVO ===
        self.Kp = 0.0015
        self.Kd = 0.0008
        self.prev_err = 0.0
        self.max_step = 0.05

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
        self.deadzone = 2   # pixels

        # === SCAN STATE MACHINE ===
        self.phase = "align"
        self.iteration = 0
        self.max_iterations = 10

        self.get_logger().info("AGS Scan + Align started")

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

        if self.iteration >= self.max_iterations:
            return

        # === IF WE ARE IN STEP PHASE, DO IT AND RETURN ===
        if self.phase == "step":
            self.step_elbow()
            self.phase = "align"
            time.sleep(1.0)
            return

        # ===============================
        # ===== YOUR ALIGN CODE =========
        # ===============================

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        vis = img.copy()
        H, W, _ = img.shape
        icx = W // 2
        icy = H // 2

        # Draw axes
        cv2.line(vis, (icx, 0), (icx, H), (0, 0, 255), 2)
        cv2.line(vis, (0, icy), (W, icy), (0, 255, 0), 2)

        # Detect red
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower1 = (0, 120, 70)
        upper1 = (10, 255, 255)
        lower2 = (170, 120, 70)
        upper2 = (180, 255, 255)

        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours = [c for c in contours if cv2.contourArea(c) > 500]

        if not contours:
            cv2.imshow("scan_align", vis)
            cv2.waitKey(1)
            return

        c = max(contours, key=cv2.contourArea)

        x, y, bw, bh = cv2.boundingRect(c)
        box_cx = x + bw // 2
        box_cy = y + bh // 2

        cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0,255,0), 2)
        cv2.circle(vis, (box_cx, box_cy), 5, (0,255,0), -1)

        err_x = box_cx - icx

        cv2.putText(vis, f"iter={self.iteration} ex={err_x}", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        err = err_x

        # === DEADZONE ===
        if abs(err) < self.deadzone:
            self.prev_err = 0.0
            self.get_logger().info(f"Aligned at iteration {self.iteration}")

            # Switch to elbow step phase
            self.phase = "step"
            return
        else:
            derr = err - self.prev_err
            self.prev_err = err
            u = self.Kp * err + self.Kd * derr

        # === CLAMP ===
        u = max(-self.max_step, min(self.max_step, u))

        # Apply
        self.forearm += u
        self.forearm = max(-3.14, min(3.14, self.forearm))

        self.send_joints()

        cv2.imshow("scan_align", vis)
        cv2.waitKey(1)

    def step_elbow(self):
        self.get_logger().info(f"Stepping elbow at iteration {self.iteration}")

        self.elbow -= 0.05
        self.elbow = max(-1.57, min(1.57, self.elbow))

        self.send_joints()

        self.iteration += 1

    def send_joints(self):
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
    node = AGSScanAlign()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
