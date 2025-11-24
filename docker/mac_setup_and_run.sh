#!/bin/bash
set -e

echo "=== [EAR_SLAM] Starte Setup für ROS2 / Docker Umgebung (macOS) ==="

# Check Docker Desktop
if ! docker info >/dev/null 2>&1; then
    echo "[ERROR] Docker Desktop läuft nicht! Bitte starten."
    exit 1
fi

echo "[Docker] Baue Image..."
docker compose build

echo "[Docker] Starte Container..."
docker compose up -d

echo "=== Fertig! ==="
echo "Container läuft im Hintergrund."
echo ""
echo "Öffne den Container mit:"
echo "    docker exec -it ros2_turtlebot3 bash"