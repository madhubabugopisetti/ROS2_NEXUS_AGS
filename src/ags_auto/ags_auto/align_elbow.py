#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import subprocess

class AGSStepElbow(Node):
    def __init__(self):
        super().__init__("ags_step_elbow")
        self.sub_js = self.create_subscription(JointState, "/joint_states", self.joint_cb, 10)
        self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.shoulder = 0.0
        self.elbow = 0.0
        self.forearm = 0.0
        self.wrist_pitch = -1.2
        self.wrist_roll = 0.0
        self.left_finger = 0.01
        self.right_finger = 0.01
        self.joint_ready = False
        self.done = False
        self.get_logger().info("AGS Step Elbow started")
    def joint_cb(self, msg):
        if self.done:
            return
        if "shoulder_joint" in msg.name:
            self.shoulder = msg.position[msg.name.index("shoulder_joint")]
        if "elbow_joint" in msg.name:
            self.elbow = msg.position[msg.name.index("elbow_joint")]
        if "forearm_joint" in msg.name:
            self.forearm = msg.position[msg.name.index("forearm_joint")]
        self.joint_ready = True
        # === DO ONLY ONCE ===
        self.step_and_exit()
    def step_and_exit(self):
        self.done = True
        # Step elbow
        self.elbow -= 0.05
        # Safety clamp
        self.elbow = max(-1.57, min(1.57, self.elbow))
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
            self.right_finger,
        ]
        p.time_from_start.sec = 1
        traj.points.append(p)
        self.pub.publish(traj)
        self.get_logger().info("Elbow stepped. Launching forearm align...")
        # Launch forearm align
        subprocess.Popen(["ros2", "run", "ags_auto", "align_forearm"])
        # Exit this node
        rclpy.shutdown()


def main():
    rclpy.init()
    node = AGSStepElbow()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
