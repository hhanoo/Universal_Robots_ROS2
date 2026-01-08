# ROS 2 Humble Desktop base -------------------------------------------------
# https://hub.docker.com/_/ros/tags
FROM hhanoo/ros:humble-desktop

# Locale and Font Settings for Emoji Support ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    fonts-noto-color-emoji \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# Build Dependencies ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# ROS2 Control Dependencies ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Universal Robots ROS2 Driver ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ur \
    && rm -rf /var/lib/apt/lists/*

# Tool Dependencies ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    libmodbus-dev \
    && rm -rf /var/lib/apt/lists/*

# Development Tools ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    bear \
    clang-format \
    clangd \
    && rm -rf /var/lib/apt/lists/*

# Additional Dependencies ------------------------------------------------------------
# RUN uv pip install --system \
#     numpy \
#     setuptools==69.5.1 \ 
#     opencv-python \
#     pyrealsense2 \
#     matplotlib

# Setup workspace ------------------------------------------------------------
WORKDIR /ros2_ws

# ROS environment auto source -----------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Default command ------------------------------------------------------------
CMD ["bash"]