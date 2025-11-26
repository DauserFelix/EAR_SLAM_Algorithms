# ROS2 Diverse Befehle

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