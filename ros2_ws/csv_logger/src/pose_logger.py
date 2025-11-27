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

class LegoLoamPoseLogger(Node):
    def __init__(self):
        super().__init__("lego_loam_pose_logger")

        # Optional: Zeitgestempelter Name
        timestamp = datetime.now().strftime("%m-%d_%H-%M")
        out_dir = os.path.join(os.getcwd(), "data_logs")
        os.makedirs(out_dir, exist_ok=True)
        self.csv_path = os.path.join(out_dir, f"{timestamp}_lego_loam_pose.csv")

        self.get_logger().info(f"Logging Pose to: {self.csv_path}")

        # CSV vorbereiten
        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)

        # Genau dein gewünschtes Format:
        # time, px, py, pz, dx, dy, dz, dw
        self.writer.writerow(["time", "px", "py", "pz", "dx", "dy", "dz", "dw"])

        # LeGO-LOAM-Pose-Topic: meist /aft_mapped_to_init
        self.sub = self.create_subscription(
            Odometry,
            "/aft_mapped_to_init",  # ggf. /laser_odom_to_init oder /odom
            self.cb,
            10
        )

    def cb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.writer.writerow([
            t,
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w   # dx, dy, dz, dw
        ])

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LegoLoamPoseLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
