#!/usr/bin/env python3
"""
Dieses Skript loggt die Odometry-Ausgaben von DLIO in eine CSV-Datei.
Es abonniert das Topic /dlio/odom_node/odom (nav_msgs/Odometry) und
speichert jede eingehende Pose samt Geschwindigkeit.

Für jeden Datensatz werden folgende Werte aufgezeichnet:
    - time_sec : Zeitstempel
    - px, py, pz : Position
    - qx, qy, qz, qw : Orientierung (Quaternion)
    - vx, vy, vz : Lineare Geschwindigkeit
    - wx, wy, wz : Winkelgeschwindigkeit

Die Ausgabe wird mit einem Zeitstempel benannt:
    MM-DD_hh-mm_pose_logger_output.csv
und unter /data abgelegt.

Zweck:
Erzeugt eine vollständige Zeitreihe der DLIO-Pose für spätere
Trajektorienvergleiche (2D/3D), Fehlerberechnungen und Plots.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os
from datetime import datetime

class PoseLogger(Node):
    def __init__(self):
        super().__init__("pose_logger")

        # Timestamped filename
        timestamp = datetime.now().strftime("%m-%d_%H-%M")
        self.csv_path = f"/data/{timestamp}_pose_logger_output.csv"

        self.get_logger().info(f"Logging Pose to: {self.csv_path}")

        # Create CSV file
        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)

        # Write header
        self.writer.writerow([
            "time_sec",
            # Position
            "px", "py", "pz",
            # Orientation
            "qx", "qy", "qz", "qw",
            # Linear velocity
            "vx", "vy", "vz",
            # Angular velocity
            "wx", "wy", "wz"
        ])

        # Subscribe to DLIO odometry
        self.sub = self.create_subscription(
            Odometry,
            "/dlio/odom_node/odom",
            self.cb,
            10
        )

    def cb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
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
    node = PoseLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
