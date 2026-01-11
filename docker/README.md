# Docker – ROS 2 Humble Environment
Dieses Verzeichnis enthält das vollständige Docker-Setup für die ROS 2-basierte EAR-SLAM-Entwicklung, inkl. TurtleBot3, GTSAM, g2o, RViz/Gazebo, rosbag-Replay.

## Ordnerstruktur
```bash
docker/
├── Dockerfile              # ROS 2 Humble Image inkl. GTSAM & g2o
├── docker-compose.yaml     # Container-Definition (Host-Network, X11, Volumes)
├── apt-packages.txt        # Systemabhängigkeiten
├── requirements.txt        # Python-Abhängigkeiten
├── setup_and_run.sh        # Setup- & Startskript
```
## Setup & Start (empfohlen)
Vom docker/-Ordner aus:
```bash
./setup_and_run.sh
```
Das Skript:

1. aktiviert den Docker-Dienst
2. fügt den Nutzer zur docker-Gruppe hinzu (falls nötig)
3. baut das Image
4. startet den Container im Hintergrund

## Manuelle Alternative
```bash
docker compose build
docker compose up -d
```
Shell im Container öffnen:
```bash
docker exec -it ros2_turtlebot3 bash
```
## Container Eigenschaften

- Base Image: ros:humble-ros-base
- User: ubuntu (kein Root-Workflow)
- Network: host (stabile DDS-Discovery)
- Middleware: CycloneDDS
- Libraries: GTSAM (PPA), g2o (CMake-Fix Branch)
- GUI: RViz / Gazebo via X11
- TurtleBot3 Modell: burger

### Volumes (Bind Mounts)
| Host (Repo-Root) | Container              | Zweck         |
| ---------------- | ---------------------- | ------------- |
| `ros2_ws/`       | `/home/ubuntu/ros2_ws` | ROS-Workspace |
| `data/`          | `/data`                | Eingabedaten  |
| `output/`        | `/output`              | Ergebnisse    |

## ROS-Workspace im Container
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```
ROS-Setups werden automatisch über `.bashrc` gesourct.