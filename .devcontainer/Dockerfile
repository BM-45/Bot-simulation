# Start from the official ROS 2 Humble base
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Base tools (one layer)
# Install basic development tools 
RUN apt-get update && \ 
    apt-get upgrade -y && \ 
    apt-get install -y --no-install-recommends \ 
        build-essential \ 
        git \ 
        wget \ 
        curl \ 
        nano \ 
        python3-pip \ 
        tmux \ 
        lsb-release \ 
        sudo \ 
        locales \ 
        apt-utils \
        && rm -rf /var/lib/apt/lists/*

# ---- Add OSRF Gazebo repo with keyring (no apt-key)
RUN wget -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg https://packages.osrfoundation.org/gazebo.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# ---- ROS desktop-ish CLI tools you listed
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-joint-state-publisher-gui \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-xacro \
    libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-render-util0 \
    libxcb-xinerama0 libxkbcommon-x11-0 x11-apps \
    && rm -rf /var/lib/apt/lists/*

# ---- Gazebo Fortress (Ignition) + ROS ↔ Gazebo bridge
RUN apt-get update && apt-get install -y --no-install-recommends \
    ignition-fortress \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# ---- Controllers + manager + teleop (your "must-haves")
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Workspace
WORKDIR /rover_ws/src

# Source ROS in every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["bash"]
