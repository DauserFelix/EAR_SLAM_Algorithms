#!/usr/bin/env python3
"""
Dieses Skript loggt die 2D-Pose der slam_toolbox während der Laufzeit.
Es abonniert das Topic /pose (PoseWithCovarianceStamped) und speichert
jede eingehende Pose in eine CSV-Datei unter /data.

Für jeden Pose-Eintrag werden folgende Werte protokolliert:
    - time_sec : Zeitstempel (ROS-Zeit)
    - px       : x-Position
    - py       : y-Position
    - yaw      : Yaw-Winkel (aus dem Quaternion extrahiert)
    - qz, qw   : relevante Quaternion-Komponenten (für 2D-Rotation)

Die Ausgabe-Datei wird automatisch mit einem Zeitstempel benannt:
    MM-DD_hh-mm_slam_toolbox_logger.csv

Einsatz:
Dieses Skript wird parallel zu slam_toolbox ausgeführt, um daraus
eine dichte 2D-Trajektorie für spätere Analysen und Vergleiche
(DLIO, LIO-SAM, Mapping-Qualität, Drift) zu erzeugen.
"""

import rclpy
from rclpy.node import Node
import csv, os
from datetime import datetime

from geometry_msgs.msg import PoseWithCovarianceStamped
import math


class SlamPoseLogger(Node):
    def __init__(self):
        super().__init__("slam_pose_logger")

        # Output path
        now = datetime.now().strftime("%m-%d_%H-%M")
        out_dir = "/data"
        os.makedirs(out_dir, exist_ok=True)

        self.filename = f"{now}_slam_toolbox_logger.csv"
        self.out_path = os.path.join(out_dir, self.filename)

        self.csv = open(self.out_path, "w", newline="")
        self.writer = csv.writer(self.csv)

        self.writer.writerow(["time_sec", "px", "py", "yaw", "qz", "qw"])

        # Only one subscriber → correct type
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/pose",
            self.cb,
            10
        )

        self.get_logger().info(f"Logging /pose → {self.out_path}")

    def cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        yaw = math.atan2(2*(q.w*q.z), 1 - 2*(q.z*q.z))
        self.writer.writerow([t, p.x, p.y, yaw, q.z, q.w])


def main():
    rclpy.init()
    node = SlamPoseLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.csv.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
