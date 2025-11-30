#!/bin/bash
set -e  # stop on first actual error

echo "=== [EAR_SLAM] Starte Setup für ROS2 / Docker Umgebung ==="

# -------------------------------------------------------------
# 1. Docker-Service aktivieren
# -------------------------------------------------------------
echo "[Docker] Aktiviere Docker-Dienst..."
sudo systemctl enable --now docker

# Nutzer zur docker-Gruppe hinzufügen, damit kein sudo nötig ist
if ! groups "$USER" | grep -q "\bdocker\b"; then
    echo "[Docker] Füge Benutzer '$USER' der docker-Gruppe hinzu..."
    sudo usermod -aG docker "$USER"
    echo " >> Bitte TERMINAL NEU STARTEN, damit die Gruppenrechte aktiv werden!"
fi


# -------------------------------------------------------------
# 2. Image bauen
# -------------------------------------------------------------
echo "[Docker] Baue Container-Image..."
docker compose build


# -------------------------------------------------------------
# 3. Container starten
# -------------------------------------------------------------
echo "[Docker] Starte Container..."
docker compose up -d

echo "=== [EAR_SLAM] Setup abgeschlossen! ==="
echo "Container läuft nun im Hintergrund."

echo
echo "Öffne den Container mit:"
echo "    docker exec -it ros2_turtlebot3 bash"
echo
echo "Workspace liegt unter:"
echo "    ~/git/EAR_SLAM_Algorithms/ros2_ws"
