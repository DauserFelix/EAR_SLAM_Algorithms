#!/usr/bin/env python3
import rosbag2_py
import numpy as np
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from collections import defaultdict

# ------------------ CONFIG ------------------
BAG_PATH = "/data/halltest4_small_ros2_mcap/halltest4_small_ros2_mcap.mcap"
# --------------------------------------------


def load_bag_info(bag_path):
    info = rosbag2_py.Info()
    metadata = info.read_metadata(bag_path, "mcap")
    return metadata


def pretty_print_bag_info(metadata):
    print("\n================ BAG INFO ================")
    print(f"Files:             {metadata.relative_file_paths}")
    print(f"Bag size:          {metadata.bag_size} bytes")
    print(f"Storage id:        {metadata.storage_identifier}")

    # Convert nanosecond fields properly
    duration_sec = metadata.duration.nanoseconds / 1e9
    start_time_sec = metadata.starting_time.nanoseconds / 1e9

    print(f"Duration:          {duration_sec:.6f} s")
    print(f"Start:             {start_time_sec} (unix timestamp)")
    print(f"Messages:          {metadata.message_count}")

    print("\nTopic information:")
    for topic in metadata.topics_with_message_count:
        print(f"  Topic: {topic.topic_metadata.name:<18} | "
              f"Type: {topic.topic_metadata.type:<30} | "
              f"Count: {topic.message_count}")



def analyze_tf_frames(bag_path):
    print("\n================ TF TREE ================")

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    tf_frames = {}

    total_tf = 0

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in ["/tf", "/tf_static"]:
            continue

        msg = deserialize_message(data, TFMessage)

        for tf in msg.transforms:
            total_tf += 1
            child = tf.child_frame_id
            parent = tf.header.frame_id
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9

            if child not in tf_frames:
                tf_frames[child] = {
                    "parent": parent,
                    "count": 1,
                    "first_stamp": stamp,
                    "last_stamp": stamp,
                }
            else:
                f = tf_frames[child]
                f["count"] += 1
                if stamp < f["first_stamp"]:
                    f["first_stamp"] = stamp
                if stamp > f["last_stamp"]:
                    f["last_stamp"] = stamp

        # Progress updates
        if total_tf % 20000 == 0:
            print(f"Processed {total_tf} transforms...")

    # print summary
    print(f"\nTotal TF transforms processed: {total_tf}\n")

    for frame, info in tf_frames.items():
        span = info["last_stamp"] - info["first_stamp"]
        freq = info["count"] / span if span > 0 else 0.0

        print(f"\nFrame: {frame}")
        print(f"  parent:          {info['parent']}")
        print(f"  count:           {info['count']}")
        print(f"  first_stamp:     {info['first_stamp']}")
        print(f"  last_stamp:      {info['last_stamp']}")
        print(f"  duration:        {span:.3f} s")
        print(f"  frequency:       {freq:.3f} Hz")



def main():
    metadata = load_bag_info(BAG_PATH)
    pretty_print_bag_info(metadata)
    analyze_tf_frames(BAG_PATH)


if __name__ == "__main__":
    main()
