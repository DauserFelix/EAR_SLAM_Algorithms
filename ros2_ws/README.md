# ROS2 Diverse Befehle

ros1 bag in ros2 bag konvertieren
```bash
rosbags-convert \
  --src /data/halltest4_small.bag \
  --dst /data/halltest4_small_ros2_mcap \
  --dst-storage mcap \
  --dst-typestore ros2_humble
```
ros2 bag info
```bash
ros2 bag info -s mcap /data/halltest4_small_ros2_mcap/halltest4_small_ros2_mcap.mcap
```

output:
```bash
Files:             /data/halltest4_small_ros2_mcap/halltest4_small_ros2_mcap.mcap
Bag size:          5.4 GiB
Storage id:        mcap
Duration:          919.432676656s
Start:             Aug  2 2023 12:37:39.668494163 (1690979859.668494163)
End:               Aug  2 2023 12:52:59.101170819 (1690980779.101170819)
Messages:          582076
Topic information: 
    Topic: /cmd_vel         | Type: geometry_msgs/msg/Twist     | Count: 4933   | Serialization Format: cdr
    Topic: /odometry/imu    | Type: nav_msgs/msg/Odometry       | Count: 91948  | Serialization Format: cdr
    Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 9119   | Serialization Format: cdr
    Topic: /imu/data        | Type: sensor_msgs/msg/Imu         | Count: 91974  | Serialization Format: cdr
    Topic: /tf_static       | Type: tf2_msgs/msg/TFMessage      | Count: 1      | Serialization Format: cdr
    Topic: /tf              | Type: tf2_msgs/msg/TFMessage      | Count: 361097 | Serialization Format: cdr
    Topic: /odom            | Type: nav_msgs/msg/Odometry       | Count: 23004  | Serialization Format: cdr
``

ros2 package erzeugen
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake mein_package
ros2 pkg create --build-type ament_python my_python_pkg
```

package löschen
```bash
cd ~/ros2_ws
rm -rf src/<package_name>
rm -rf build install log
```

sourcen:
```bash
source install/setup.bash
```

bauen:
```bash
cd ~/ros2_ws
colcon build

oder nur ein bestimmtes
colcon build --packages-select slam_tools
```

package (python) starten:
```bash
ros2 run my_pkg my_pkg
```

topics:
```bash
ros2 topic list
```
Mit zusätzlichen Datentypen
```bash
ros2 topic list -t
```

topic zuhören:
```bash
ros2 topic echo /topic --once
ros2 topic echo /scan --once
```

transformation anschauen zur simulierten Zeit
```bash
ros2 run tf2_ros tf2_echo map liosam_base_link --ros-args -p use_sim_time:=true


```

Struktur der PointCloud:
Die PointCloud2-Struktur ist wie folgt:
 ```bash
 ubuntu@nils-B360M-DS3H:~/ros2_ws/src/direct_lidar_inertial_odometry$ sed -n '1,200p' /tmp/pcd.yaml header: stamp: sec: 1690979865 nanosec: 674972057 frame_id: velodyne height: 1 width: 26748 fields: - name: x offset: 0 datatype: 7 count: 1 - name: y offset: 4 datatype: 7 count: 1 - name: z offset: 8 datatype: 7 count: 1 - name: intensity offset: 12 datatype: 7 count: 1 - name: ring offset: 16 datatype: 4 count: 1 - name: time offset: 18 datatype: 7 count: 1 is_bigendian: false point_step: 22 row_step: 588456
 ```
 | Feld      | Typ      |
|-----------|----------|
| x         | float32  |
| y         | float32  |
| z         | float32  |
| intensity | float32  |
| ring      | uint16   |
| time      | float32  |

# Diverse Ausgaben von topics

## /tf_static
```bash
ros2 topic echo /tf_static
Some, but not all, publishers are offering QoSDurabilityPolicy.TRANSIENT_LOCAL. Falling back to QoSDurabilityPolicy.VOLATILE as it will connect to all publishers
transforms:
- header:
    stamp:
      sec: 1690979774
      nanosec: 655983038
    frame_id: L515
  child_frame_id: L515_scan
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.4999999999966269
      y: 0.4999999999966269
      z: 0.5000018366025517
      w: -0.49999816339744835
- header:
    stamp:
      sec: 1690979774
      nanosec: 655993457
    frame_id: halterung
  child_frame_id: L515
  transform:
    translation:
      x: 0.0
      y: 0.0231
      z: 0.0445
    rotation:
      x: -2.597353007557415e-06
      y: 0.7071080798547033
      z: 0.707105482506466
      w: -2.5973434669646147e-06
- header:
    stamp:
      sec: 1690979774
      nanosec: 655994448
    frame_id: chassis_link
  child_frame_id: base_arm
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: -4.3297802811774664e-17
      y: -0.7071067811865475
      z: 0.7071067811865476
      w: 4.329780281177467e-17
- header:
    stamp:
      sec: 1690979774
      nanosec: 655995672
    frame_id: base_link
  child_frame_id: chassis_link
  transform:
    translation:
      x: 0.23
      y: 0.0
      z: 0.193
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
- header:
    stamp:
      sec: 1690979774
      nanosec: 655996071
    frame_id: chassis_link
  child_frame_id: boxlink
  transform:
    translation:
      x: -0.57
      y: -0.03
      z: 0.1
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
- header:
    stamp:
      sec: 1690979774
      nanosec: 655997357
    frame_id: chassis_link
  child_frame_id: kamerabox
  transform:
    translation:
      x: 0.235
      y: 0.0
      z: 0.0
    rotation:
      x: -0.7071080798594738
      y: 0.0
      z: 0.0
      w: 0.7071054825112363
- header:
    stamp:
      sec: 1690979774
      nanosec: 655997729
    frame_id: kamerabox
  child_frame_id: halterung
  transform:
    translation:
      x: 0.065
      y: -0.1
      z: 0.0
    rotation:
      x: -0.4999999999966269
      y: -0.4999999999966269
      z: 0.5000018366025517
      w: -0.49999816339744835
- header:
    stamp:
      sec: 1690979774
      nanosec: 655998166
    frame_id: velodyne
  child_frame_id: imu
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.04
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
- header:
    stamp:
      sec: 1690979774
      nanosec: 655998602
    frame_id: velodyne_base_link
  child_frame_id: velodyne
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0377
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
- header:
    stamp:
      sec: 1690979774
      nanosec: 655999846
    frame_id: chassis_link
  child_frame_id: velodyne_base_link
  transform:
    translation:
      x: -0.655
      y: 0.0
      z: 0.2
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
---
```