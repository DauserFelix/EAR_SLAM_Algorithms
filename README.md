# EAR SLAM Algorithms
Unsere Projektarbeit wurde unter Ubuntu 22.04.5 LTS (Jammy Jellyfish) mit Intel(R) Core(TM) i5-9600K CPU @ 3.70GHz und NVIDIA GeForce RTX 2060 aufgesetzt und getestet.

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

# FAST-LIO aufnehmen
![status: stable](https://img.shields.io/badge/status-stable-green)
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
