# imu_logger.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv
import os
from datetime import datetime

class ImuLogger(Node):
    def __init__(self):
        super().__init__("imu_logger")

        # Timestamp für Dateiname
        ts = datetime.now().strftime("%m-%d_%H-%M")
        self.csv_path = f"/data/{ts}_imu_logger_output.csv"

        self.get_logger().info(f"Logging IMU to: {self.csv_path}")

        # Datei öffnen + Header
        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)

        self.writer.writerow([
            "time_sec",
            "ax", "ay", "az",
            "gx", "gy", "gz",
            "qw", "qx", "qy", "qz"
        ])

        # Subscriber
        self.sub = self.create_subscription(
            Imu,
            "/imu/data",
            self.cb,
            100
        )

    def cb(self, msg: Imu):
        self.writer.writerow([
            float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ])

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
