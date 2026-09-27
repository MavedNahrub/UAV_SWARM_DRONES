###############################################################################
# PX4 Multi-UAV SITL + ROS 2 Humble + Gazebo Harmonic
# Base: Ubuntu 22.04 | GPU: NVIDIA (via nvidia-container-toolkit)
###############################################################################
FROM ubuntu:22.04 AS base

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Locale setup (required by ROS 2)
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ─── Base development tools ──────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    curl \
    git \
    gnupg \
    lsb-release \
    python3 \
    python3-pip \
    python3-venv \
    software-properties-common \
    sudo \
    wget \
    vim \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# ─── ROS 2 Humble ───────────────────────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y \
        ros-humble-desktop \
        ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# ─── ROS 2 Python dependencies ──────────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    empy==3.3.4 \
    pyros-genmsg \
    setuptools \
    networkx \
    pyyaml \
    jsonschema \
    scipy \
    numpy

# ─── PX4 build dependencies (from ubuntu.sh, pre-seeded) ────────────────────
RUN apt-get update && apt-get install -y \
    astyle \
    gdb \
    libharfbuzz-dev \
    libxml2-dev \
    libxml2-utils \
    ninja-build \
    openjdk-11-jre \
    rsync \
    shellcheck \
    unzip \
    xsltproc \
    zip \
    && rm -rf /var/lib/apt/lists/*

# ─── Micro XRCE-DDS Agent (pre-built binary) ───────────────────────────────
# Building from source is fragile (upstream Fast DDS branches get deleted).
# Instead, install the latest stable release binary.
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent \
    && cd /tmp/agent \
    && git checkout v3.0.2 \
    && git submodule update --init --recursive \
    && mkdir -p build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && ldconfig /usr/local/lib/ \
    && rm -rf /tmp/agent

# ─── Create non-root user ───────────────────────────────────────────────────
ARG USERNAME=px4user
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /home/$USERNAME

# ─── PX4-Autopilot ──────────────────────────────────────────────────────────
RUN git clone --recursive --branch v1.17.0 \
        https://github.com/PX4/PX4-Autopilot.git /home/$USERNAME/PX4-Autopilot \
    && cd /home/$USERNAME/PX4-Autopilot \
    && bash ./Tools/setup/ubuntu.sh --no-nuttx \
    && make px4_sitl

# ─── ROS 2 workspace with px4_msgs ──────────────────────────────────────────
RUN mkdir -p /home/$USERNAME/ros2_ws/src \
    && cd /home/$USERNAME/ros2_ws/src \
    && git clone --branch release/1.17 https://github.com/PX4/px4_msgs.git \
    && git clone https://github.com/PX4/px4_ros_com.git \
    && /bin/bash -c "source /opt/ros/humble/setup.bash && cd /home/$USERNAME/ros2_ws && colcon build --symlink-install"

# ─── Shell setup ─────────────────────────────────────────────────────────────
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
    && echo 'source ~/ros2_ws/install/setup.bash 2>/dev/null' >> ~/.bashrc \
    && echo 'source ~/uav_swarm/ros2_ws/install/setup.bash 2>/dev/null' >> ~/.bashrc \
    && echo 'export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds' >> ~/.bashrc

# ─── NVIDIA runtime support ─────────────────────────────────────────────────
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

# ─── Default entrypoint ─────────────────────────────────────────────────────
CMD ["/bin/bash"]
