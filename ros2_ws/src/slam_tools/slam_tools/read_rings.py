#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class Reader(Node):
    def __init__(self):
        super().__init__("ring_reader")
        self.sub = self.create_subscription(PointCloud2, "/velodyne_points", self.cb, 10)

    def cb(self, msg):
        rings = set()

        for p in pc2.read_points(msg, field_names=("ring",), skip_nans=True):
            rings.add(int(p[0]))

        print("Rings found:", rings)
        rclpy.shutdown()

def main():
    rclpy.init()
    node = Reader()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
