# =========================================================================
# Base Image
# =========================================================================
# - https://hub.docker.com/r/hhanoo/ros/tags
FROM hhanoo/ros:humble-desktop

# - Build dependencies
ARG DEBIAN_FRONTEND=noninteractive

# =========================================================================
# Locale and Font
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    fonts-noto-color-emoji \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# =========================================================================
# Essential Packages
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    curl \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# ROS2
# =========================================================================
# - ROS2 setup
ARG ROS_VERSION=humble
ENV ROS_DISTRO=${ROS_VERSION}

# - ROS2 Control
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# - ROS2 Universal Robots Driver
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ur \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# C++ Build Dependencies
# =========================================================================
# - libmodbus-dev : tool controllers (Modbus RTU/TCP)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libmodbus-dev \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# Development Tools
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    bear \
    clang-format \
    clangd \
    gdb \
    gdbserver \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# Workspace Setup
# =========================================================================
# - Workspace
WORKDIR /ros2_ws

# - Commands
COPY ./commands.sh /etc/profile.d/commands.sh
RUN chmod 644 /etc/profile.d/commands.sh && \
    echo 'source /etc/profile.d/commands.sh' >> /etc/bash.bashrc

# - Entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# - Default command
CMD ["bash"]