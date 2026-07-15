# ===== ROS env =====
[ -f /opt/ros/humble/setup.bash ]    && source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ]   && source /ros2_ws/install/setup.bash
[ -f /ros2_ws/src/docker/config.sh ] && source /ros2_ws/src/docker/config.sh

# ===== Common helpers =====
source-ros-ws() {
    [ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
}

source-config() {
    [ -f /ros2_ws/src/docker/config.sh ] && source /ros2_ws/src/docker/config.sh
}

# ===== Build =====
build() {
    cd /ros2_ws || return 1
    colcon build \
        --symlink-install \
        --cmake-args \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            -DCMAKE_CXX_STANDARD=17 \
            -DCMAKE_CXX_STANDARD_REQUIRED=ON \
            -DCMAKE_BUILD_TYPE=Release "$@"
    source-ros-ws
    source-config
}

build-debug() {
    # Build with debug symbols, keep optimization (RelWithDebInfo)
    build -DCMAKE_BUILD_TYPE=RelWithDebInfo "$@"
}

# ===== Debug =====
# Run a node under gdbserver so host VSCode (cppdbg) can attach on :3000.
# Only one gdbserver can bind :3000 at a time — pick one node per session.
debug-motion() {
    source-ros-ws
    gdbserver :3000 install/ur_motion/lib/ur_motion/motion_action_server "$@"
}

# ===== Robot calibration =====
extract-calib() {
    source-ros-ws
    source-config
    ros2 launch ur_calibration calibration_correction.launch.py \
        robot_ip:="${ROBOT_IP}" \
        target_filename:=/ros2_ws/src/ur_description_wrapper/config/calibration_kinematics.yaml \
        "$@"
}

# ===== Granular launchers =====
run-ur() {
    source-ros-ws
    source-config
    ros2 launch /ros2_ws/src/1_ur_driver.launch.py \
        robot_ip:="${ROBOT_IP}" \
        ur_type:="${UR_TYPE}" \
        use_fake_hardware:="${USE_FAKE_HARDWARE}" \
        "$@"
}

run-moveit() {
    source-ros-ws
    source-config
    ros2 launch /ros2_ws/src/2_ur_moveit.launch.py \
        ur_type:="${UR_TYPE}" \
        launch_moveit_rviz:="${LAUNCH_RVIZ}" \
        "$@"
}

run-motion() {
    source-ros-ws
    ros2 launch /ros2_ws/src/3_motion_server.launch.py "$@"
}

# ===== Combined =====
run-all() {
    source-ros-ws
    source-config
    ros2 launch /ros2_ws/src/all.launch.py \
        robot_ip:="${ROBOT_IP}" \
        ur_type:="${UR_TYPE}" \
        use_fake_hardware:="${USE_FAKE_HARDWARE}" \
        launch_moveit_rviz:="${LAUNCH_RVIZ}" \
        "$@"
}

# ===== Examples =====
example-movej() {
    source-ros-ws
    ros2 run ur_robot_client example_movej "$@"
}

example-io() {
    source-ros-ws
    ros2 run ur_robot_client example_io_speed "$@"
}

example-complete() {
    source-ros-ws
    ros2 run ur_robot_client example_complete "$@"
}

example-state() {
    source-ros-ws
    ros2 run ur_robot_client example_state "$@"
}

# ===== Help =====
cmd-help() {
    printf "\n[Universal_Robots_ROS2] Commands:\n\n"

    printf "  Build:\n"
    printf "    %-18s - %s\n" "build"            "colcon build --symlink-install + source overlay"
    printf "    %-18s - %s\n" "build-debug"      "build with debug symbols (RelWithDebInfo)"
    printf "\n"

    printf "  Debug (gdbserver :3000, host VSCode F5 attach):\n"
    printf "    %-18s - %s\n" "debug-motion"     "Run motion_action_server under gdbserver"
    printf "\n"

    printf "  Robot calibration:\n"
    printf "    %-18s - %s\n" "extract-calib"    "Extract UR calibration kinematics  [ROBOT_IP]"
    printf "\n"

    printf "  Granular launchers (Individual nodes, for debugging):\n"
    printf "    %-18s - %s\n" "run-ur"           "UR robot driver                    [ROBOT_IP, UR_TYPE, USE_FAKE_HARDWARE]"
    printf "    %-18s - %s\n" "run-moveit"       "MoveIt                             [UR_TYPE, LAUNCH_RVIZ]"
    printf "    %-18s - %s\n" "run-motion"       "Motion action server"
    printf "\n"

    printf "  Combined (Full system):\n"
    printf "    %-18s - %s\n" "run-all"          "UR driver + MoveIt + Motion        [ROBOT_IP, UR_TYPE, USE_FAKE_HARDWARE, LAUNCH_RVIZ]"
    printf "\n"

    printf "  Examples (ur_robot_client):\n"
    printf "    %-18s - %s\n" "example-movej"    "MoveJ motion example"
    printf "    %-18s - %s\n" "example-io"       "Digital I/O + speed slider example"
    printf "    %-18s - %s\n" "example-complete" "Pick & Place full example"
    printf "    %-18s - %s\n" "example-state"    "Read-only state monitoring"
    printf "\n"

    printf "  Config / Help:\n"
    printf "    %-18s - %s\n" "source-config"    "Reload /ros2_ws/src/docker/config.sh"
    printf "    %-18s - %s\n" "cmd-help"         "Show this help"
    printf "\n"

    printf "  Current config (from /ros2_ws/src/docker/config.sh):\n"
    printf "    ROBOT_IP=%s\n"          "${ROBOT_IP}"
    printf "    UR_TYPE=%s\n"           "${UR_TYPE}"
    printf "    USE_FAKE_HARDWARE=%s\n" "${USE_FAKE_HARDWARE}"
    printf "    LAUNCH_RVIZ=%s\n"       "${LAUNCH_RVIZ}"
    echo
}

# ===== Show help on interactive shell =====
case $- in
    *i*) cmd-help ;;
esac
