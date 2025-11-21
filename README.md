# EAR SLAM Algorithms

## rosbag konvertieren

```bash
rosbags-convert \
  --src /data/halltest4_small.bag \
  --dst /data/halltest4_small_ros2_mcap \
  --dst-storage mcap \
  --dst-typestore ros2_humble
```

```bash
sudo apt install ros-humble-rosbag2-storage-mcap 

ros2 bag info -s mcap /data/halltest4_small_ros2_mcap/halltest4_small_ros2_mcap.mcap
```

```bash
Files:             /data/halltest4_small_ros2_mcap/halltest4_small_ros2_mcap.mcap
Bag size:          5.4 GiB
Storage id:        mcap
Duration:          919.432676656s
Start:             Aug  2 2023 12:37:39.668494163 (1690979859.668494163)
End:               Aug  2 2023 12:52:59.101170819 (1690980779.101170819)
Messages:          582076
Topic information: Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 4933 | Serialization Format: cdr
                   Topic: /odometry/imu | Type: nav_msgs/msg/Odometry | Count: 91948 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 9119 | Serialization Format: cdr
                   Topic: /imu/data | Type: sensor_msgs/msg/Imu | Count: 91974 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 361097 | Serialization Format: cdr
                   Topic: /odom | Type: nav_msgs/msg/Odometry | Count: 23004 | Serialization Format: cdr
```

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

# Increase FastDDS message size limit

Ursache:
```bash
[dlio_odom_node-1] terminate called after throwing an instance of 'eprosima::fastcdr::exception::NotEnoughMemoryException' [dlio_odom_node-1] what(): Not enough memory in the buffer stream
```

Wir erhöhen die FastDDS mit einer XML-Konfiguration:
```bash
touch ~/.ros/fastdds.xml
```
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_transport</transport_id>
        <type>UDPv4</type>
        <!-- Erhöhe die maximale Paketgröße -->
        <maxMessageSize>65536000</maxMessageSize> <!-- 64 MB -->
        <sendBufferSize>65536000</sendBufferSize> <!-- 64 MB -->
        <receiveBufferSize>65536000</receiveBufferSize> <!-- 64 MB -->
      </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="dlio_participant" is_default_profile="true">
      <rtps>
        <builtin>
          <metatrafficUnicastLocatorList>
            <locator>
              <transport_id>udp_transport</transport_id>
            </locator>
          </metatrafficUnicastLocatorList>
        </builtin>
      </rtps>
    </participant>
  </profiles>
</dds>
```
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds.xml
``

[dlio_odom_node-1] +-------------------------------------------------------------------+
[dlio_odom_node-1] |               Direct LiDAR-Inertial Odometry v1.1.1               |
[dlio_odom_node-1] +-------------------------------------------------------------------+
[dlio_odom_node-1] | Wed Aug  2 12:52:59 2023             Elapsed Time: 919.39 seconds |
[dlio_odom_node-1] | Intel(R) Core(TM) i5-9600K CPU @ 3.70GHz x 6                      |
[dlio_odom_node-1] | Sensor Rates: Velodyne @ 9.92 Hz, IMU @ 100.07 Hz                 |
[dlio_odom_node-1] |===================================================================|
[dlio_odom_node-1] | Position     {W}  [xyz] :: -5.9830 -7.4832 -69.5334               |
[dlio_odom_node-1] | Orientation  {W} [wxyz] :: -0.9795 0.1126 -0.1188 -0.1174         |
[dlio_odom_node-1] | Lin Velocity {B}  [xyz] :: 0.0172 0.4549 -6.5060                  |
[dlio_odom_node-1] | Ang Velocity {B}  [xyz] :: -0.0041 -0.0015 -0.1400                |
[dlio_odom_node-1] | Accel Bias        [xyz] :: 2.07832885 0.97086650 -5.00000000      |
[dlio_odom_node-1] | Gyro Bias         [xyz] :: 0.00289364 0.00021916 0.13907044       |
[dlio_odom_node-1] |                                                                   |
[dlio_odom_node-1] | Distance Traveled  :: 295.0185 meters                             |
[dlio_odom_node-1] | Distance to Origin :: 70.1904 meters                              |
[dlio_odom_node-1] | Registration       :: keyframes: 48, deskewed points: 24447       |
[dlio_odom_node-1] |                                                                   |
[dlio_odom_node-1] | Computation Time ::   0.00 ms    // Avg:   0.00 / Max:   0.00     |
[dlio_odom_node-1] | Cores Utilized   ::   0.42 cores // Avg:   0.31 / Max:   1.25     |
[dlio_odom_node-1] | CPU Load         ::   6.94 %     // Avg:   5.24 / Max:  20.83     |
[dlio_odom_node-1] | RAM Allocation   :: 268.63 MB                                     |
[dlio_odom_node-1] +-------------------------------------------------------------------+

# Ausführung
Terminal 1:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run slam_tools pose_logger
```

Terminal 2:
```bash
source install/setup.bash
ros2 launch direct_lidar_inertial_odometry dlio.launch.py   pointcloud_topic:=/velodyne_points   imu_topic:=/imu/data   rviz:=false
```

Terminal 3:
```bash
ros2 bag play /data/halltest4_small_ros2_mcap/halltest4_small_ros2_mcap.mcap --clock -r 2
```



# ros2 Diverse Befehle

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

bauen:
```bash
cd ~/ros2_ws
colcon build

oder nur ein bestimmtes
colcon build --packages-select slam_tools
```

# Evaluierung im Vergleich mit LIO-SAM
