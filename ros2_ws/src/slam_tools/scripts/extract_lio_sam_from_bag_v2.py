#!/usr/bin/env python3
"""
Extrahiert die vollständige 3D-Pose von LIO-SAM aus einer ROS2-Bagdatei,
indem alle TF-Transformationen ausgewertet und die Transformation vom
SOURCE_FRAME zum TARGET_FRAME rekonstruiert wird.

Funktionen des Skripts:

1) Öffnet eine MCAP-Bagdatei und liest alle /tf- und /tf_static-Nachrichten.
2) Baut daraus eine TF-Kette zwischen SOURCE_FRAME → TARGET_FRAME auf.
3) Berechnet für jeden Zeitstempel:
      - Position (px, py, pz)
      - Orientierung als Quaternion (qx, qy, qz, qw)
4) Berechnet zusätzlich:
      - Lineare Geschwindigkeit (vx, vy, vz) per Differenzenquotient
      - Exakte Rotationsgeschwindigkeit (wx, wy, wz) über die
        Quaternion-Logarithmus-Methode
5) Schreibt alle Werte geordnet in eine CSV-Datei:
      time_sec, position, quaternion, linear velocity, angular velocity

Ergebnis: Eine vollständige, zeitlich sortierte Pose-Zeitreihe von LIO-SAM,
perfekt geeignet für Trajektorienvergleich, Fehleranalyse und Plotting.
"""

import rosbag2_py
import numpy as np
import csv
import os
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

# ----------------------------------------------------
# Konfiguration
# ----------------------------------------------------
BAG_PATH = "/data/halltest4_small_ros2_mcap/halltest4_small_ros2_mcap.mcap"
TARGET_FRAME = "map"
SOURCE_FRAME = "liosam_base_link"
OUTPUT_CSV = "/data/lio_sam.csv"


# ----------------------------------------------------
# Quaternion helper Funktionen
# ----------------------------------------------------
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


# ----------------------------------------------------
# Quaternion Logarithmus
# ----------------------------------------------------
def quat_log(q):
    x, y, z, w = q
    v = np.array([x, y, z])
    nv = np.linalg.norm(v)

    if nv < 1e-12:
        return np.zeros(3)

    theta = np.arctan2(nv, w)
    return theta * v / nv


# ----------------------------------------------------
# TF buffer
# ----------------------------------------------------
tf_tree = {}
tf_static_tree = {}

def add_tf(tf: TransformStamped, static=False):
    d = tf_static_tree if static else tf_tree
    child = tf.child_frame_id
    parent = tf.header.frame_id
    d[child] = (parent, tf)


def find_parent(child):
    if child in tf_tree:
        return tf_tree[child]
    if child in tf_static_tree:
        return tf_static_tree[child]
    return None


def compute_transform_to_target(child_frame):
    chain = []
    current = child_frame

    while current != TARGET_FRAME:
        result = find_parent(current)
        if result is None:
            return None
        parent, tf = result
        chain.append(tf)
        current = parent

    T = np.eye(4)
    for tf in reversed(chain):
        tr = tf.transform.translation
        rot = tf.transform.rotation
        R = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        R[0, 3] = tr.x
        R[1, 3] = tr.y
        R[2, 3] = tr.z
        T = T @ R

    return T


# ----------------------------------------------------
# MAIN
# ----------------------------------------------------
def main():
    print("Opening bag:", BAG_PATH)

    storage_options = rosbag2_py.StorageOptions(
        uri=BAG_PATH,
        storage_id="mcap"
    )
    converter_options = rosbag2_py.ConverterOptions("", "")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics = reader.get_all_topics_and_types()
    topic_types = {t.name: t.type for t in topics}

    os.makedirs(os.path.dirname(OUTPUT_CSV), exist_ok=True)
    csv_file = open(OUTPUT_CSV, "w")
    writer = csv.writer(csv_file)
    writer.writerow([
        "time_sec", "px", "py", "pz",
        "qx", "qy", "qz", "qw",
        "vx", "vy", "vz",
        "wx", "wy", "wz"
    ])

    last_pos = None
    last_q = None
    last_time = None

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic_types[topic] != "tf2_msgs/msg/TFMessage":
            continue

        msg = deserialize_message(data, TFMessage)
        is_static = (topic == "/tf_static")

        for tf in msg.transforms:
            add_tf(tf, static=is_static)

            if tf.child_frame_id != SOURCE_FRAME:
                continue

            T = compute_transform_to_target(SOURCE_FRAME)
            if T is None:
                continue

            px, py, pz = T[0, 3], T[1, 3], T[2, 3]
            qx, qy, qz, qw = quaternion_from_matrix(T)
            q_curr = np.array([qx, qy, qz, qw])

            timestamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9

            vx = vy = vz = 0.0
            wx = wy = wz = 0.0

            if last_pos is not None:
                dt = timestamp - last_time
                if dt > 0:

                    # linear vel
                    vx = (px - last_pos[0]) / dt
                    vy = (py - last_pos[1]) / dt
                    vz = (pz - last_pos[2]) / dt

                    # exact angular vel
                    q_inv = np.array([-last_q[0], -last_q[1], -last_q[2], last_q[3]])
                    q_delta = quat_mul(q_inv, q_curr)

                    omega = (2.0 * quat_log(q_delta)) / dt
                    wx, wy, wz = omega

            writer.writerow([
                timestamp, px, py, pz,
                qx, qy, qz, qw,
                vx, vy, vz,
                wx, wy, wz
            ])

            last_pos = np.array([px, py, pz])
            last_q = q_curr
            last_time = timestamp

    csv_file.close()
    print("DONE →", OUTPUT_CSV)


if __name__ == "__main__":
    main()
