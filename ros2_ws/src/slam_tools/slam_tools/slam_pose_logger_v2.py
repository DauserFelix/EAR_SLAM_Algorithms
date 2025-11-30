#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import os
from datetime import datetime
from scipy.spatial.transform import Rotation as R


class SlamPoseLogger(Node):
    def __init__(self):
        super().__init__("slam_pose_logger")

        # Output
        now = datetime.now().strftime("%m-%d_%H-%M")
        out_dir = "/data"
        os.makedirs(out_dir, exist_ok=True)

        self.out_path = f"{out_dir}/{now}_slam_toolbox.csv"

        self.csv = open(self.out_path, "w", newline="")
        self.writer = csv.writer(self.csv)

        # Header
        self.writer.writerow(["time_sec", "px", "py", "yaw"])

        # Richtiges Topic!
        self.sub = self.create_subscription(
            PoseStamped,
            "/slam_toolbox/pose",
            self.cb,
            10
        )

        self.get_logger().info(f"Logging SLAM toolbox → {self.out_path}")

    def cb(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation

        # quaternion → yaw
        rot = R.from_quat([q.x, q.y, q.z, q.w])
        yaw = rot.as_euler('xyz', degrees=False)[2]   # Z-Achse

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.writer.writerow([t, p.x, p.y, yaw])


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
