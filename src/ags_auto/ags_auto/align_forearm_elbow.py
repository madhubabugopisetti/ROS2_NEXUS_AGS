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
        # === GAINS ===
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
        # State machine
        self.phase = "align_forearm"   # align_forearm -> align_wrist -> step
        # Z stop threshold
        self.z_stop_threshold = 120
        self.get_logger().info("AGS Scan + Align + Wrist started")
    def joint_cb(self, msg):
        if "shoulder_joint" in msg.name:
            self.shoulder = msg.position[msg.name.index("shoulder_joint")]
        if "elbow_joint" in msg.name:
            self.elbow = msg.position[msg.name.index("elbow_joint")]
        if "forearm_joint" in msg.name:
            self.forearm = msg.position[msg.name.index("forearm_joint")]
        if "wrist_roll_joint" in msg.name:
            self.wrist_roll = msg.position[msg.name.index("wrist_roll_joint")]
        self.joint_ready = True
    def image_cb(self, msg):
        if not self.joint_ready:
            return
        
        
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
        # Z proxy
        z = bh
        # === STEP PHASE ===
        if self.phase == "step":
            # === Z STOP (only after alignment) ===
            if z > self.z_stop_threshold:
                cv2.putText(vis, f"Z STOP z={z}", (10,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                cv2.imshow("scan_align", vis)
                cv2.waitKey(1)
                self.approach_and_grasp()
                return
            self.step_elbow()
            self.phase = "align_forearm"
            time.sleep(1.0)
            return
        # ===============================
        # ===== VISION ==================
        # ===============================
        box_cx = x + bw // 2
        box_cy = y + bh // 2
        cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0,255,0), 2)
        cv2.circle(vis, (box_cx, box_cy), 5, (0,255,0), -1)
        err_x = box_cx - icx
        err_y = box_cy - icy
        cv2.putText(vis, f"x={err_x} y={err_y} z={z}", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        err = err_x
        # === DEADZONE CHECK ===
        if abs(err) < self.deadzone:
            self.prev_err = 0.0
            if self.phase == "align_forearm":
                self.phase = "align_wrist"
            elif self.phase == "align_wrist":
                self.phase = "step"
            return
        # === PD CONTROL ===
        derr = err - self.prev_err
        self.prev_err = err
        u = self.Kp * err + self.Kd * derr
        u = max(-self.max_step, min(self.max_step, u))
        # === APPLY CONTROL ===
        if self.phase == "align_forearm":
            self.forearm += u
            self.forearm = max(-3.14, min(3.14, self.forearm))
        elif self.phase == "align_wrist":
            self.wrist_roll += u
            self.wrist_roll = max(-3.14, min(3.14, self.wrist_roll))
        self.send_joints()
        cv2.imshow("scan_align", vis)
        cv2.waitKey(1)
    def step_elbow(self):
        self.get_logger().info("Stepping elbow")
        self.elbow -= 0.2
        self.elbow = max(-1.57, min(1.57, self.elbow))
        self.send_joints()
    def approach_and_grasp(self):
        self.get_logger().info("Opening gripper")
        # 1) Open gripper
        self.left_finger = 0.06
        self.right_finger = 0.06
        self.send_joints()
        time.sleep(1.0)
        self.get_logger().info("Approaching (2 iterations)")
        # 2) Move elbow + wrist forward (2 iterations)
        for i in range(4):
            self.elbow -= 0.04
            self.wrist_pitch += 0.04
            self.elbow = max(-1.57, min(1.57, self.elbow))
            self.wrist_pitch = max(-2.5, min(0.0, self.wrist_pitch))
            self.send_joints()
            time.sleep(0.7)
        self.get_logger().info("Closing gripper")
        # 3) Close gripper
        self.left_finger = 0.0
        self.right_finger = 0.0
        self.send_joints()
        self.get_logger().info("Lifting object")
        # 4) Lift up
        for i in range(4):
            self.elbow += 0.05
            self.send_joints()
            time.sleep(0.5)
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
