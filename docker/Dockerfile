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

# ============================================================
# === Benutzer anlegen + sudo-Rechte =========================
# ============================================================
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# ============================================================
# === APT-Pakete installieren aus apt-packages.txt ===========
# ============================================================
COPY apt-packages.txt /tmp/apt-packages.txt

RUN apt-get update && \
    grep -Ev '^\s*#' /tmp/apt-packages.txt | grep -Ev '^\s*$' | \
    xargs apt-get install -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# ============================================================
# === GTSAM aus Source installieren (für LeGO-LOAM)        ===
# ============================================================
# RUN wget -O /tmp/gtsam.zip https://github.com/borglab/gtsam/archive/4.2.0.zip \
#     && unzip /tmp/gtsam.zip -d /tmp/ \
#     && cd /tmp/gtsam-4.2.0 \
#     && mkdir build && cd build \
#     && cmake -DCMAKE_BUILD_TYPE=Release \
#              -DGTSAM_USE_SYSTEM_EIGEN=ON \
#              -DEIGEN_INCLUDE_DIR=/usr/include/eigen3 \
#              .. \
#     && make -j$(nproc) \
#     && make install \
#     && ldconfig \
#     && rm -rf /tmp/gtsam*

# ============================================================
# === GTSAM installieren (über offizielles Borglab PPA) =======
# ============================================================
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository ppa:borglab/gtsam-release-4.1 -y && \
    apt-get update && \
    apt-get install -y libgtsam-dev libgtsam-unstable-dev && \
    rm -rf /var/lib/apt/lists/*

# ============================================================
# === g2o (fixed CMake version, ROS2 compatible) ============
# ============================================================
RUN git clone --branch has_use_cmake_fixes https://github.com/RainerKuemmerle/g2o.git /tmp/g2o \
    && cd /tmp/g2o \
    && mkdir build && cd build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=ON \
        -DG2O_BUILD_APPS=OFF \
        -DG2O_BUILD_EXAMPLES=OFF \
        -DG2O_USE_OPENGL=OFF \
        -DG2O_USE_CHOLMOD=OFF \
        -DG2O_USE_CSPARSE=ON \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && rm -rf /tmp/g2o

ENV CMAKE_PREFIX_PATH="/usr/local:${CMAKE_PREFIX_PATH}"

# TurtleBot3 Modell als Standard setzen
ENV TURTLEBOT3_MODEL=burger

# ============================================================
# === Abschnitt: Python-Pakete als Root installieren       ===
# Installation zusätzlicher Python-Pakete im Systemkontext.
# ============================================================
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# ============================================================
# === CycloneDDS AKTIVIEREN                                ===
# ============================================================
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ============================================================
# === Abschnitt: Nutzer wechseln                          ===
# Ab hier läuft der Container als ubuntu-User.
# ============================================================
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# ============================================================
# === Abschnitt: Python-Pakete als User installieren       ===
# Hier werden zusätzliche Pakete im User-Kontext installiert.
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

RUN echo 'export CMAKE_PREFIX_PATH="/usr/local:${CMAKE_PREFIX_PATH}"' >> /home/ubuntu/.bashrc

# ============================================================
# === rosdep install (OFFICIAL STEP FROM README)           ===
# ============================================================
RUN sudo apt-get update && rosdep update && \
    rosdep install --from-paths /home/ubuntu/ros2_ws/src --ignore-src -yr

# Standardverzeichnis beim Start
WORKDIR /home/${USERNAME}/ros2_ws

# ============================================================
# === Abschnitt: Standardbefehl                           ===
# Der Container startet in einer interaktiven Bash-Shell.
# ============================================================
CMD ["/bin/bash"]