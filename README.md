# EAR SLAM Algorithms

# Getting started
## Schritt 1: Docker Container starten
![ROS: Humble](https://img.shields.io/badge/ROS-Humble-blueviolet)
Wechsel in `docker` und führe nachstehenden Befehl aus:
```bash
./setup_and_run.sh
```
Das Bash-Skript erstellt den Docker-Container. Dieser ist dann unter `docker exec -it ros2_turtlebot3 bash` erreichbar. Der Workspace liegt unter `~/git/EAR_SLAM_Algorithms/ros2_ws`  
## Schritt 2: ros2 packages bauen
```bash
colcon build
```

# Pose von DLIO aufnehmen und in .csv speichern
![status: stable](https://img.shields.io/badge/status-stable-green)
## Wir starten drei Terminals und führen nachstehende Befehle aus
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
ros2 bag play /data/halltest4_small_ros2_mcap/halltest4_small_ros2_mcap.mcap --clock --start-paused --qos-profile-overrides-path /data/halltest4_small_ros2_mcap/qos_tf.yaml 
```

Ausgabe Terminal 2, nachdem das ros2 bag gestartet wurde
```bash
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
```

# Ergebnis plotten
```bash
ros2 run plot_manager plot_manager 
```

# Pose von LeGO-LOAM aufnehmen und in .csv speichern
![status: stable](https://img.shields.io/badge/status-stable-green)
## Wir starten drei Terminals und führen nachstehende Befehle aus
Terminal 1:
```bash
cd ~/ros2_ws
source install/setup.bash
/ros2_ws/src/slam_tools/slam_tools$ ./lego_pose_logger.py 
```

Terminal 2:
```bash
source install/setup.bash
ros2 launch lego_loam_sr run.launch.py lidar_topic:=/velodyne_points rviz:=false
```

Terminal 3:
```bash
source install/setup.bash
ros2 bag play /data/halltest4_small_ros2_mcap/halltest4_small_ros2_mcap.mcap --clock --start-paused --qos-profile-overrides-path /data/halltest4_small_ros2_mcap/qos_tf.yaml 
```
Terminal 3 Alternative
```bash
ros2 bag play /data/halltest4_small_ros2_mcap/halltest4_small_ros2_mcap.mcap   --clock   --start-paused   --remap /tf:=/tf_disabled /tf_static:=/tf_static_disabled
```
Die Ergebnisse sind leider nicht gut. Vielleicht sollte die `config.yaml` näher begutachtet werden.

# FAST-LIO aufnehmen
# ![status: experimental](https://img.shields.io/badge/status-experimental-orange)
## Wir starten drei Terminals und führen nachstehende Befehle aus
Terminal 1:
```bash
source install/setup.bash
ros2 bag play /data/halltest4_small_ros2_mcap/halltest4_small_ros2_mcap.mcap --clock --start-paused --qos-profile-overrides-path /data/halltest4_small_ros2_mcap/qos_tf.yaml 
```
Terminal 2:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run slam_tools fast_lio_pose_logger
```
Terminal 3:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=velodyne.yaml
```


# Li-SLAM aufnehmen
# ![status: experimental](https://img.shields.io/badge/status-experimental-orange)

## Schritt 1: Node starten
Terminal 1
```bash
ros2 launch scanmatcher lio_bigloop.launch.py \
```
## To be continued
# ![status: experimental](https://img.shields.io/badge/status-experimental-orange)
Ausgabe:
```bash
[INFO] [1764634132.428558577] [graph_based_slam]: initialization start
registration_method:NDT
voxel_leaf_size[m]:0.2
ndt_resolution[m]:5
ndt_num_threads:0
loop_detection_period[Hz]:1000
threshold_loop_closure_score:1
distance_loop_closure[m]:20
range_of_searching_loop_closure[m]:20
search_submap_num:3
num_adjacent_pose_cnstraints:5
use_save_map_in_loop:true
------------------
[INFO] [1764634132.430734066] [graph_based_slam]: initialize Publishers and Subscribers
[INFO] [1764634132.432184474] [graph_based_slam]: initialization end
```
Terminal 2:
```bash
ros2 bag play /data/halltest4_small_ros2_mcap/halltest4_small_ros2_mcap.mcap \
  --clock \
  --start-paused \
  --qos-profile-overrides-path /data/halltest4_small_ros2_mcap/qos_tf.yaml \
  --topics /velodyne_points /imu/data /odom /tf /tf_static \
  --remap /imu/data:=/imu_correct /velodyne_points:=/points_raw /odom:=/odometry

ros2 bag play /data/halltest4_small_ros2_mcap/halltest4_small_ros2_mcap.mcap \
  --clock \
  --start-paused \
  --qos-profile-overrides-path /data/halltest4_small_ros2_mcap/qos_tf.yaml \
  --topics /velodyne_points /imu/data \
  --remap /imu/data:=/imu_correct /velodyne_points:=/points_raw
```

ros2 bag play /data/halltest4_small_ros2_mcap/halltest4_small_ros2_mcap.mcap \
  --clock \
  --start-paused \
  --qos-profile-overrides-path /data/halltest4_small_ros2_mcap/qos_tf.yaml \
  --topics /velodyne_points /imu/data /odom \
  --remap /imu/data:=/imu_correct /velodyne_points:=/points_raw /odom:=/odometry

ros2 bag play /data/halltest4_small_ros2_mcap/halltest4_small_ros2_mcap.mcap \
  --clock \
  --start-paused \
  --qos-profile-overrides-path /data/halltest4_small_ros2_mcap/qos_tf.yaml
























## Evaluierung im Vergleich mit LIO-SAM
# ![status: experimental](https://img.shields.io/badge/status-experimental-orange)
## Pose von SLAM-TOOLBOX (2D) aufnehmen und in .csv speichern

Terminal 1:
```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node   --ros-args   -r cloud_in:=/velodyne_points   -r scan:=/scan   -p target_frame:=liosam_base_link   -p min_height:=-1.0   -p max_height:=1.0
```

Terminal 2:
```bash
ros2 bag play /data/halltest4_small_ros2_mcap/halltest4_small_ros2_mcap.mcap --clock --start-paused --qos-profile-overrides-path /data/halltest4_small_ros2_mcap/qos_tf.yaml
```


Terminal 3:
```bash
ros2 run slam_toolbox sync_slam_toolbox_node \
--ros-args \
-p use_sim_time:=true \
-p scan_topic:=/scan \
-p base_frame:=liosam_base_link \
-p map_frame:=map \
-p provide_odom_frame:=false \
-p publish_tf:=false \
-p use_scan_matching:=true \
-p use_online_correlative_scan_matching:=true \
-p mode:=mapping
```
`provide_odom_frame:=false` → `slam_toolbox` versucht NICHT, `odom` zu erzeugen
`publish_tf:=false` → sie veröffentlicht KEIN `tf`, weil `tf` bei dir sowieso 3D ist
`use_scan_matching:=true` → Laser-ICP aktiviert
`online_correlative` → hoher Qualitätsmodus
hierdurch wird `/pose` publiziert, auch ohne Odometrie

Terminal 4:
```bash
./slam_pose_logger.py
```

ros2 run slam_toolbox sync_slam_toolbox_node \
--ros-args \
-p use_sim_time:=true \
-p scan_topic:=/scan \
-p base_frame:=liosam_base_link \
-p map_frame:=map \
-p provide_odom_frame:=false \
-p publish_tf:=false \
-p use_scan_matching:=false \
-p use_online_correlative_scan_matching:=false \
-p mode:=mapping
