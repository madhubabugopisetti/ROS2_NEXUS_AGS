#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2, time
import numpy as np

JOINTS = [
    "shoulder_joint",
    "elbow_joint",
    "forearm_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "left_finger_joint",
    "right_finger_joint"
]

SCAN   = 0
ALIGN  = 1

class AGS(Node):
    def __init__(self):
        super().__init__("ags")
        # ===== MODE =====
        self.mode = SCAN
        # ===== SCAN PARAMS =====
        self.step = 0.1
        self.slow_step = 0.02
        self.max = 3.14
        self.min = -3.14
        # ===== ALIGN (PD) PARAMS =====
        self.Kp = 0.0012
        self.Kd = 0.0010
        self.deadzone = 2
        self.max_step = 0.05
        self.prev_err = 0.0
        # ===== STATE =====
        self.box_found = False
        self.stable_count = 0
        self.STABLE_REQ = 5
        # ===== ROS =====
        self.pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.sub_img = self.create_subscription(
            Image, "/camera/image", self.image_cb, 10
        )
        self.sub_js = self.create_subscription(
            JointState, "/joint_states", self.joint_cb, 10
        )
        self.bridge = CvBridge()
        self.pose = [0.0, -0.5, -1.2, -1.2, 0.0, 0.01, 0.01]
        self.shoulder = 0.0
        self.joint_ready = False
        self.send_pose(self.pose, 2.0)
        time.sleep(2.5)
        self.timer = self.create_timer(0.3, self.scan_rotate)

    # ================= JOINT STATE =================
    def joint_cb(self, msg):
        if "shoulder_joint" in msg.name:
            self.shoulder = msg.position[msg.name.index("shoulder_joint")]
            self.pose[0] = self.shoulder
            self.joint_ready = True

    # ================= COMMAND =================
    def send_pose(self, pos, t):
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        p = JointTrajectoryPoint()
        p.positions = pos
        p.time_from_start.sec = int(t)
        traj.points = [p]
        self.pub.publish(traj)

    # ================= SCAN =================
    def scan_rotate(self):
        if self.mode != SCAN:
            return
        step = self.slow_step if self.box_found else self.step
        self.pose[0] += step
        if self.pose[0] >= self.max:
            self.step = -abs(self.step)
        elif self.pose[0] <= self.min:
            self.step = abs(self.step)
        self.send_pose(self.pose, 1.0)

    # ================= IMAGE =================
    def image_cb(self, msg):
        if not self.joint_ready:
            return
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        vis = img.copy()
        h, w, _ = img.shape
        cx, cy = w // 2, h // 2
        # axes
        cv2.line(vis, (0, cy), (w, cy), (0, 0, 255), 1)
        cv2.line(vis, (cx, 0), (cx, h), (0, 255, 0), 1)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = (
            cv2.inRange(hsv, (0,120,70), (10,255,255)) |
            cv2.inRange(hsv, (170,120,70), (180,255,255))
        )
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = [c for c in cnts if cv2.contourArea(c) > 500]
        if not cnts:
            self.box_found = False
            self.stable_count = 0
            cv2.imshow("cam", vis)
            cv2.waitKey(1)
            return
        c = max(cnts, key=cv2.contourArea)
        x,y,wc,hc = cv2.boundingRect(c)
        bx = x + wc//2
        by = y + hc//2
        err_x = bx - cx
        err_y = by - cy
        cv2.putText( vis, f"err_x={err_x}  err_y={err_y}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.rectangle(vis,(x,y),(x+wc,y+hc),(0,255,0),2)
        cv2.circle(vis,(bx,by),4,(0,255,0),-1)
        self.box_found = True
        self.stable_count += 1
        # ===== MODE SWITCH =====
        if self.mode == SCAN and self.stable_count >= self.STABLE_REQ:
            self.mode = ALIGN
            self.prev_err = 0.0
        # ===== ALIGN =====
        if self.mode == ALIGN:
            err = by - cy
            if abs(err) < self.deadzone:
                u = 0.0
                self.prev_err = 0.0
            else:
                derr = err - self.prev_err
                self.prev_err = err
                u = self.Kp * err + self.Kd * derr
            u = max(-self.max_step, min(self.max_step, u))
            self.pose[0] -= u
            self.pose[0] = max(self.min, min(self.max, self.pose[0]))
            self.send_pose(self.pose, 1.0)
        cv2.imshow("cam", vis)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = AGS()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
