#!/bin/bash

# ===============================
# Get workspace root directory
# 스크립트 위치를 기준으로 워크스페이스 루트 계산
# ===============================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ===============================
# User options
# ===============================
ROBOT_IP="127.0.0.1"
TARGET_FILENAME="$WORKSPACE_ROOT/KETI_Universal_Robots_ROS2/ur_description_wrapper/config/calibration_kinematics.yaml"

# ===============================
# ROS2 env
# ===============================
# Source ROS2 Humble environment (ROS2 환경 소싱)
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "[INFO] Sourcing ROS2 Humble environment"
    source /opt/ros/humble/setup.bash
else
    echo "[ERROR] ROS2 Humble not found"
    exit 1
fi

# Source ROS2 workspace (ROS2 워크스페이스 소싱)
if [ -f /ros2_ws/install/setup.bash ]; then
    echo "[INFO] Sourcing ROS2 workspace"
    source /ros2_ws/install/setup.bash
else
    echo "[ERROR] ROS2 workspace not found"
    exit 1
fi

# ===============================
# Extract robot calibration_kinematics
# ===============================
echo "[INFO] Extracting robot calibration_kinematics"
echo "[INFO] Target file: $TARGET_FILENAME"
ros2 launch ur_calibration calibration_correction.launch.py \
    robot_ip:=$ROBOT_IP \
    target_filename:="$TARGET_FILENAME"


