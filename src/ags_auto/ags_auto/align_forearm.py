#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2


class AGSAlignForearmX(Node):
    def __init__(self):
        super().__init__("ags_align_forearm_x")
        self.prev_error = None
        self.dir = 1

        # Subscribers
        self.sub_img = self.create_subscription(
            Image, "/camera/image", self.image_cb, 10
        )
        self.sub_js = self.create_subscription(
            JointState, "/joint_states", self.joint_cb, 10
        )

        # Publisher
        self.pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )

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

        # Control params
        self.forearm_step = 0.04
        self.deadzone = 5

        self.get_logger().info("AGS FOREARM X-axis alignment started")

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
        h, w, _ = img.shape

        cx = w // 2
        cy = h // 2

        # Draw center cross
        cv2.line(img, (0, cy), (w, cy), (0, 255, 0), 2)
        cv2.line(img, (cx, 0), (cx, h), (0, 255, 0), 2)

        # Detect red box
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower1 = (0, 120, 70)
        upper1 = (10, 255, 255)
        lower2 = (170, 120, 70)
        upper2 = (180, 255, 255)

        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            cv2.imshow("align_forearm_x", img)
            cv2.waitKey(1)
            return

        # Largest contour
        c = max(contours, key=cv2.contourArea)
        x, y, bw, bh = cv2.boundingRect(c)

        # Draw box + center dot
        cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        box_cx = x + bw // 2
        box_cy = y + bh // 2
        cv2.circle(img, (box_cx, box_cy), 5, (0, 255, 0), -1)

        # X-axis error ONLY
        err = box_cx - cx

        abs_err = abs(err)

        if self.prev_error is None:
            self.prev_error = abs_err
            self.forearm += self.dir * self.forearm_step
            self.send_joints()
            return

        # Direction validation
        if abs_err > self.prev_error:
            self.dir *= -1
            self.get_logger().warn("FOREARM DIR FLIPPED")

        self.prev_error = abs_err

        if abs_err > self.deadzone:
            self.forearm += self.dir * self.forearm_step
            self.send_joints()
            self.get_logger().info(
                f"FOREARM MOVE | x_err={err:.1f}px | forearm={self.forearm:.3f} | dir={self.dir}"
            )
        else:
            self.get_logger().info("X CENTERED — FOREARM STOP")


        cv2.imshow("align_forearm_x", img)
        cv2.waitKey(1)

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
            self.right_finger,
        ]

        p.time_from_start.sec = 1
        traj.points.append(p)
        self.pub.publish(traj)


def main():
    rclpy.init()
    node = AGSAlignForearmX()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
