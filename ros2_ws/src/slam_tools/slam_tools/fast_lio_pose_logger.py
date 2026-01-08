#!/usr/bin/env python3
"""
FAST-LIO Pose Logger für ROS2

Abonniert:
    /Odometry  (nav_msgs/Odometry)

Speichert:
    Zeit, Position, Quaternion, Linear- & Angular Velocity

Output:
    /data/YYYY-MM-DD_HH-MM_fastlio_odometry.csv
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os
from datetime import datetime

class FastLioPoseLogger(Node):
    def __init__(self):
        super().__init__("fastlio_pose_logger")

        # Ensure /data exists
        os.makedirs("/data", exist_ok=True)

        # File name with timestamp
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M")
        self.csv_path = f"/data/{timestamp}_fastlio_odometry.csv"

        self.get_logger().info(f"Logging FAST-LIO odometry to: {self.csv_path}")

        # Open CSV file
        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)

        # Write header
        self.writer.writerow([
            "time_sec",
            "px", "py", "pz",
            "qx", "qy", "qz", "qw",
            "vx", "vy", "vz",
            "wx", "wy", "wz"
        ])

        # Subscribe FAST-LIO odometry
        self.sub = self.create_subscription(
            Odometry,
            "/Odometry",
            self.cb,
            100   # FAST-LIO publishes at high rate
        )

    def cb(self, msg: Odometry):
        # Time
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Pose
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # Velocities
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular

        self.writer.writerow([
            t,
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w,
            v.x, v.y, v.z,
            w.x, w.y, w.z
        ])

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FastLioPoseLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()