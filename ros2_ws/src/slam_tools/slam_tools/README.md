# Grundlagen SLAM Algorithmik

## 1) Wie funktioniert SLAM in ROS?

SLAM = gleichzeitig Karte aufbauen + eigene Pose schätzen.
In ROS besteht ein SLAM-System immer aus vier Teilen:

- Sensor-Input (LiDAR oder Kamera)
- Front-End → extrahiert Merkmale / Scan-Matching
- Back-End → Optimierung (Graph-SLAM, EKF, Factor Graph)
- Map-Output → Occupancy-Grid, Pointcloud-Map oder Pose-Graph

SLAM ist somit ein *Pipeline*-*Prozess*, kein einzelner Befehl.

## 2) Welche Art von “Standard-SLAM” gibt es in ROS2?

- slam_toolbox (2D, LiDAR, modern, online + offline)
- cartographer_ros
- rtabmap_ros

## a) ROS2 slam_toolbox (2D) – kompakte Erklärung

### 1) Eingangs-Topics

/scan  
Typ: `sensor_msgs/msg/LaserScan`  
Basisdaten für das Scan-Matching (distanzbasierte Strahlenmessungen).Beispiel:
```bash
header.stamp
header.frame_id = "laser"
angle_min = -1.57
angle_max =  1.57
angle_increment = 0.005
ranges = [2.10, 2.05, 2.00, 3.50, ...]
```
Jeder Winkel zwischen `angle_min` und `angle_max` besitzt genau einen Eintrag in `ranges`.

/tf (Transformations-Frames)  
Wesentliche Frames:
```bash
laser_frame → base_link
```
Position des Lasers (laser_frame) relativ zum Roboter (base_link).
```bash
base_link → odom
```
Pose des Roboters (base_link) relativ zum Odometrie-Ursprung (odom).  
Eine stabile TF-Kette ist zwingend erforderlich.

/odom (optional, empfohlen)  
Typ: `nav_msgs/msg/Odometry`
Dient als zusätzliche Bewegungsschätzung für stabileres Scan-Matching.  
pose.pose – Position + Orientierung
```bash
pose.pose.position.x
pose.pose.position.y
pose.pose.position.z        #Für 2D-Roboter: z = 0
pose.pose.orientation.x
pose.pose.orientation.y
pose.pose.orientation.z
pose.pose.orientation.w
```
twist.twist – lineare und rotatorische Geschwindigkeiten
```bash
twist.twist.linear.x        #Vor-/Rückwärtsbewegung
twist.twist.linear.y        #y=0!, Differential Drive kann nicht seitlich rutschen.
twist.twist.linear.z        #z=0!, auf und ab schweben tut kein Bodenroboter.
twist.twist.angular.x       #x=0!, Rolle um Längsachse → macht kein Bodenroboter
twist.twist.angular.y       #y=0!, Nicken/Pitch → macht kein Bodenroboter
twist.twist.angular.z       #Drehgeschwindigkeit um die Hochachse (Yaw), Einheit rad/s
```
### 2) Interne Verarbeitung (Pipeline)
Front-End
- Umwandlung LaserScan → 2D-Punkte
- Korrelatives Scan-Matching
- Schätzung von Δx, Δy, Δθ zwischen zwei Scans

Pose-Graph Aufbau
- Jeder Roboterzustand = Knoten
- Odometrie + Scan-Matching = Kanten (Constraints)

Loop Closure
- Wiedererkannte Orte erzeugen zusätzliche Constraints
- Stabilisieren die globale Karte

Optimierung
- Ceres Solver / Graph Optimizer
- Ausgabe: konsistente globale Pose + Karte

### 3) Output-Topics von slam_toolbox
/map  
Typ: `nav_msgs/msg/OccupancyGrid`  
→ Laufend aktualisierte Karte.

/map_metadata  
→ Auflösung, Ursprung, Größe.  

TF: map → odom  
→ Korrektur der driftenden Odometrie.  
→ Wesentlicher Bestandteil jeder Navigation.  

Optionale SLAM-Toolbox-Services:

`/slam_toolbox/serialize_map`  
`/slam_toolbox/deserialize_map`  
`/slam_toolbox/loop_closure`  

### 4) Wichtige Parameter (Auswahl)
`resolution (z. B. 0.05 m)`  
`map_update_interval`  
`max_laser_range`  
`scan_topic`  
`map_frame`, `odom_frame`, `base_frame`  
`use_scan_matching`  
`use_loop_closure`  