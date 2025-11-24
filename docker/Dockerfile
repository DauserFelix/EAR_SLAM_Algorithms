# Basis: ROS 2 Humble (leichtgewichtige ros-base Variante, ohne Desktop)
FROM ros:humble-ros-base

# Keine interaktiven apt-Prompts & fixe Zeitzone
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# ============================================================
# === Abschnitt: Benutzer anlegen                          ===
# Hier wird ein Nicht-Root-User erstellt (ubuntu), damit
# du im Container nicht als root arbeiten musst.
# ============================================================
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME} \
    \
    # apt-get vorbereiten
    && apt-get update \
    \
    # ========================================================
    # === Abschnitt: Ubuntu- und ROS-Pakete installieren   ===
    # Hier installierst du alles, was systemweit benötigt wird:
    #   - Tools (git, curl, pip, colcon, compiler)
    #   - RViz
    #   - Gazebo + ROS-Bridge
    #   - TurtleBot3 Pakete
    #   - Nav2, SLAM Toolbox etc.
    #
    # Wenn du weitere ROS- oder apt-Pakete brauchst:
    #   einfach unten weitere Zeilen hinzufügen:
    #   ros-humble-mein-paket \
    # ========================================================
    && apt-get install -y --no-install-recommends \
       sudo \
       git \
       curl \
       ca-certificates \
       gdb gdbserver \
       python3-pip \
       python3-colcon-common-extensions \
       build-essential \
       \
       # ---- DLIO Dependencies ----
       libomp-dev \
       libpcl-dev \
       libeigen3-dev \
       ros-humble-pcl-ros \
       ros-humble-sensor-msgs \
       \
       # Visualisierung
       ros-humble-rviz2 \
       \
       # Robot-Description / Joint-State
       ros-humble-xacro \
       ros-humble-robot-state-publisher \
       ros-humble-joint-state-publisher \
       ros-humble-joint-state-publisher-gui \
       \
       # Gazebo + ROS-Brücke
       ros-humble-gazebo-ros-pkgs \
       \
       # TurtleBot3 + Navigation + SLAM
       ros-humble-turtlebot3 \
       ros-humble-turtlebot3-gazebo \
       ros-humble-turtlebot3-navigation2 \
       ros-humble-slam-toolbox \
       ros-humble-pointcloud-to-laserscan \
       ros-humble-rqt-graph \
       ros-humble-rqt-gui \
       ros-humble-rqt-gui-py \
       \
       # ROS2 Control + Controller
       ros-humble-ros2-control \
       ros-humble-ros2-controllers \
       ros-humble-controller-manager \
       ros-humble-diff-drive-controller \
       ros-humble-joint-trajectory-controller \
       \
       # ROS2 Bag
       ros-humble-rosbag2-storage-mcap  \
       # --- Developer Tools ---
       tree \
    \
    # Benutzer sudo-Rechte geben
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    \
    # apt-Cache aufräumen (spart viel Speicher)
    && rm -rf /var/lib/apt/lists/*

# TurtleBot3 Modell als Standard setzen
ENV TURTLEBOT3_MODEL=burger

# ============================================================
# === Abschnitt: Python-Pakete als root installieren       ===
# Hier kannst du beliebige pip-Pakete installieren:
#   pip install numpy scipy matplotlib ...
# Das läuft als root, daher systemweite Installation.
# ============================================================
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# ============================================================
# === Abschnitt: Nutzer wechseln                          ===
# Ab hier läuft der Container als ubuntu-User.
# ============================================================
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# ============================================================
# === Abschnitt: Python-Pakete als user installieren       ===
# Hier kannst du beliebige pip-Pakete installieren:
#   pip install numpy scipy matplotlib ...
# Das läuft als root, daher systemweite Installation.
# ============================================================
RUN pip install --no-cache-dir --user rosbags \
    && echo 'export PATH=/home/ubuntu/.local/bin:$PATH' >> /home/ubuntu/.bashrc


# ============================================================
# === Abschnitt: ROS-Umgebung & Workspace einrichten      ===
# 1) ROS 2 Setup in die .bashrc schreiben
# 2) Workspace initialisieren
# 3) Lokales Workspace-Setup beim Start automatisch sourcen
# ============================================================
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && mkdir -p /home/${USERNAME}/ros2_ws/src \
    && echo 'if [ -f /home/'${USERNAME}'/ros2_ws/install/setup.bash ]; then source /home/'${USERNAME}'/ros2_ws/install/setup.bash; fi' \
       >> /home/${USERNAME}/.bashrc

# Standardverzeichnis beim Start
WORKDIR /home/${USERNAME}/ros2_ws

# ============================================================
# === Abschnitt: Standardbefehl                           ===
# Der Container startet in einer interaktiven Bash-Shell.
# ============================================================
CMD ["/bin/bash"]
