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
