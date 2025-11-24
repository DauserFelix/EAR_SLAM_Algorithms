#!/usr/bin/env python3
"""
===============================================================================
Universeller Pose-Logger für ROS2 (DLIO, LIO-SAM, Odom, TF, rosbag)

Dieses Skript kann zwei Modi:

1) LIVE-MODUS (ROS2 Node):
   - Loggt jedes beliebige Odometry-Topic (z.B. /dlio/odom_node/odom)
   - Start:
       ros2 run <package> universeller_pose_logger.py --mode topic \
           --topic /dlio/odom_node/odom --out /data/dlio.csv

2) OFFLINE-MODUS (rosbag TF Parser):
   - Liest eine Bag-Datei und extrahiert Pose aus TF-Frames
   - Beispiel für LIO-SAM:
       ./universeller_pose_logger.py \
           --mode tf \
           --bag /data/run.mcap \
           --source liosam_base_link \
           --target map \
           --out /data/liosam.csv

Ausgabe-CSV immer im Format:
    time_sec, px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz

===============================================================================
"""

import argparse
import csv
import os
from datetime import datetime
import numpy as np

# ROS2 (für topic-mode)
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# rosbag2_py (für tf-mode)
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped



# =============================================================================
# Hilfsfunktionen: Quaternion / Matrizen
# =============================================================================

def quaternion_matrix(q):
    x, y, z, w = q
    n = x*x + y*y + z*z + w*w
    if n < np.finfo(float).eps:
        return np.eye(4)

    s = 2.0 / n
    X, Y, Z = x*s, y*s, z*s
    wX, wY, wZ = w*X, w*Y, w*Z
    xX, xY, xZ = x*X, x*Y, x*Z
    yY, yZ = y*Y, y*Z
    zZ = z*Z

    M = np.eye(4)
    M[0,0] = 1.0 - (yY + zZ)
    M[0,1] = xY - wZ
    M[0,2] = xZ + wY
    M[1,0] = xY + wZ
    M[1,1] = 1.0 - (xX + zZ)
    M[1,2] = yZ - wX
    M[2,0] = xZ - wY
    M[2,1] = yZ + wX
    M[2,2] = 1.0 - (xX + yY)
    return M


def quaternion_from_matrix(M):
    m = M[:3, :3]
    t = np.trace(m)

    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        w = 0.25 / s
        x = (m[2,1] - m[1,2]) * s
        y = (m[0,2] - m[2,0]) * s
        z = (m[1,0] - m[0,1]) * s
    else:
        i = np.argmax(np.diagonal(m))
        if i == 0:
            s = 2.0 * np.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2])
            w = (m[2,1] - m[1,2]) / s
            x = 0.25 * s
            y = (m[0,1] + m[1,0]) / s
            z = (m[0,2] + m[2,0]) / s
        elif i == 1:
            s = 2.0 * np.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2])
            w = (m[0,2] - m[2,0]) / s
            x = (m[0,1] + m[1,0]) / s
            y = 0.25 * s
            z = (m[1,2] + m[2,1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1])
            w = (m[1,0] - m[0,1]) / s
            x = (m[0,2] + m[2,0]) / s
            y = (m[1,2] + m[2,1]) / s
            z = 0.25 * s
    return np.array([x, y, z, w])


def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])


def quat_log(q):
    v = np.array([q[0], q[1], q[2]])
    nv = np.linalg.norm(v)
    if nv < 1e-12:
        return np.zeros(3)
    theta = np.arctan2(nv, q[3])
    return theta * v / nv



# =============================================================================
# Mode A: LIVE-LOGGER für Odometry-Topic
# =============================================================================

class TopicPoseLogger(Node):
    def __init__(self, topic_name, output_path):
        super().__init__("topic_pose_logger")
        self.topic_name = topic_name
        self.output_path = output_path or make_output_filename("topic")

        self.csv = open(self.output_path, "w", newline="")
        self.writer = csv.writer(self.csv)

        self.writer.writerow([
            "time_sec", "px", "py", "pz",
            "qx", "qy", "qz", "qw",
            "vx", "vy", "vz",
            "wx", "wy", "wz"
        ])

        self.sub = self.create_subscription(
            Odometry, self.topic_name, self.cb, 20
        )

        self.get_logger().info(f"Logging Odometry topic '{topic_name}' → {self.output_path}")

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular

        self.writer.writerow([
            t, p.x, p.y, p.z,
            q.x, q.y, q.z, q.w,
            v.x, v.y, v.z,
            w.x, w.y, w.z
        ])

    def destroy_node(self):
        self.csv.close()
        super().destroy_node()



# =============================================================================
# Mode B: OFFLINE-LOGGER für TF in rosbag
# =============================================================================

def log_tf_from_bag(bag_path, source, target, output):
    print(f"[TF Logger] Loading TF from bag: {bag_path}")
    print(f"[TF Logger] Chain: {target} ← ... ← {source}")

    storage = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}

    # TF buffers
    tf_tree = {}
    tf_static = {}

    def add_tf(tf, static):
        (tf_static if static else tf_tree)[tf.child_frame_id] = (
            tf.header.frame_id, tf
        )

    def find_parent(child):
        return tf_tree.get(child) or tf_static.get(child)

    def compute_chain(child_frame):
        chain = []
        current = child_frame
        while current != target:
            result = find_parent(current)
            if result is None:
                return None
            parent, tf = result
            chain.append(tf)
            current = parent

        T = np.eye(4)
        for tf in reversed(chain):
            tr, rot = tf.transform.translation, tf.transform.rotation
            R = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
            R[:3, 3] = [tr.x, tr.y, tr.z]
            T = T @ R
        return T

    # CSV file
    output = output or make_output_filename("tf")
    os.makedirs(os.path.dirname(output), exist_ok=True)
    csv_file = open(output, "w", newline="")
    print(f"[TF Logger] Output → {output}")
    writer = csv.writer(csv_file)
    writer.writerow([
        "time_sec", "px", "py", "pz",
        "qx", "qy", "qz", "qw",
        "vx", "vy", "vz",
        "wx", "wy", "wz"
    ])

    last_p = None
    last_q = None
    last_t = None

    # Main loop
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topics[topic] != "tf2_msgs/msg/TFMessage":
            continue

        msg = deserialize_message(data, TFMessage)
        static = (topic == "/tf_static")

        for tf in msg.transforms:
            add_tf(tf, static)

            if tf.child_frame_id != source:
                continue

            T = compute_chain(source)
            if T is None:
                continue

            px, py, pz = T[0, 3], T[1, 3], T[2, 3]
            qx, qy, qz, qw = quaternion_from_matrix(T)
            q = np.array([qx, qy, qz, qw])

            t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9

            # Velocities
            vx = vy = vz = wx = wy = wz = 0.0
            if last_p is not None:
                dt = t - last_t
                if dt > 0:
                    vx = (px - last_p[0]) / dt
                    vy = (py - last_p[1]) / dt
                    vz = (pz - last_p[2]) / dt
                    dq = quat_mul(
                        np.array([-last_q[0], -last_q[1], -last_q[2], last_q[3]]),
                        q
                    )
                    wv = (2.0 * quat_log(dq)) / dt
                    wx, wy, wz = wv

            writer.writerow([t, px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz])

            last_p = np.array([px, py, pz])
            last_q = q
            last_t = t

    csv_file.close()
    print(f"[TF Logger] Done → {output}")

# =============================================================================
# Create File Names
# =============================================================================

def make_output_filename(mode, base_dir="/data"):
    timestamp = datetime.now().strftime("%m-%d_%H-%M")
    filename = f"{timestamp}_upl_{mode}_output.csv"
    return os.path.join(base_dir, filename)


# =============================================================================
# Argument Parsing
# =============================================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["topic", "tf"], required=True)
    parser.add_argument("--topic", help="Odometry topic for topic-mode")
    parser.add_argument("--bag", help="Path to .mcap bag file")
    parser.add_argument("--source", help="TF child frame (e.g. liosam_base_link)")
    parser.add_argument("--target", default="map", help="TF target frame")
    parser.add_argument("--out", help="Optional: override output CSV path")

    args = parser.parse_args()

    # Mode A: LIVE ODOMETRY LOGGING
    if args.mode == "topic":
        rclpy.init()
        node = TopicPoseLogger(args.topic, args.out)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.shutdown()

    # Mode B: TF FROM BAG
    elif args.mode == "tf":
        log_tf_from_bag(args.bag, args.source, args.target, args.out)



if __name__ == "__main__":
    main()
