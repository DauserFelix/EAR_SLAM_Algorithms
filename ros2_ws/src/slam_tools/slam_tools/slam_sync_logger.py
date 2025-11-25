#!/usr/bin/env python3
"""
SLAM Sync Logger
----------------
Loggt gleichzeitig:
  - /pose   (PoseStamped von SLAM Toolbox)
  - /odom   (Odometry für Twist)

Synchronisiert per ApproximateTimeSynchronizer.

CSV-Format:
MM-DD_hh-mm_slam_sync_output.csv
Gespeichert unter: /data
"""

import rclpy
from rclpy.node import Node
import csv
from datetime import datetime
import os

from message_filters import Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class SlamSyncLogger(Node):
    def __init__(self):
        super().__init__("slam_sync_logger")

        # ==============================
        # CSV-Dateiname + Pfad erzeugen
        # ==============================
        timestamp = datetime.now().strftime("%m-%d_%H-%M")
        os.makedirs("/data", exist_ok=True)
        output_path = f"/data/{timestamp}_slam_sync_output.csv"

        self.csv_file = open(output_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)

        self.writer.writerow([
            "time_sec",
            "px", "py", "pz",
            "qx", "qy", "qz", "qw",
            "vx", "vy", "vz",
            "wx", "wy", "wz"
        ])

        self.get_logger().info(f"Logge synchronisiert /pose + /odom → {output_path}")

        # ==============================
        # Subscriber
        # ==============================
        self.pose_sub = Subscriber(self, PoseStamped, "/pose")
        self.odom_sub = Subscriber(self, Odometry, "/odom")

        # ==============================
        # Synchronisation
        # ==============================
        self.sync = ApproximateTimeSynchronizer(
            [self.pose_sub, self.odom_sub],
            queue_size=50,
            slop=0.05
        )
        self.sync.registerCallback(self.callback)

    def callback(self, pose_msg, odom_msg):
        # Zeit
        t = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9

        # Pose
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation

        # Twist aus /odom
        v = odom_msg.twist.twist.linear
        w = odom_msg.twist.twist.angular

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


def main():
    rclpy.init()
    node = SlamSyncLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
