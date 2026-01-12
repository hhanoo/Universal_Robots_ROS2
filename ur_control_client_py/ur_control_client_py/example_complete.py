#!/usr/bin/env python3
"""
Complete Example (Python)
==========================
Demonstrates all features:
- MoveJ (joint space motion)
- MoveL (Cartesian linear motion)
- Speed slider control
- Digital I/O control
- State monitoring

Usage:
    ros2 run ur_control_client_py example_complete
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from ur_control_client_py import URRobotController


def create_tmatrix(x, y, z, rx, ry, rz):
    """
    Create 4x4 transformation matrix from position and rotation

    Args:
        x, y, z: Position in meters
        rx, ry, rz: Rotation around X, Y, Z axes in radians (Euler angles)

    Returns:
        list: 16-element list representing 4x4 transformation matrix (row-major)
    """
    # Rotation matrices
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    # Combined rotation matrix (ZYX order)
    T = [0.0] * 16

    T[0] = cy * cz
    T[1] = cz * sx * sy - cx * sz
    T[2] = sx * sz + cx * cz * sy
    T[3] = x

    T[4] = cy * sz
    T[5] = cx * cz + sx * sy * sz
    T[6] = cx * sy * sz - cz * sx
    T[7] = y

    T[8] = -sy
    T[9] = cy * sx
    T[10] = cx * cy
    T[11] = z

    T[12] = 0.0
    T[13] = 0.0
    T[14] = 0.0
    T[15] = 1.0

    return T


def main(args=None):
    rclpy.init(args=args)

    # Create ROS2 node
    node = Node("ur_control_example_complete")

    # Create robot controller
    robot = URRobotController(node)

    # Wait for robot connection
    node.get_logger().info("Waiting for robot connection...")
    while rclpy.ok() and not robot.is_connected():
        rclpy.spin_once(node, timeout_sec=0.1)

    if not robot.is_connected():
        node.get_logger().error("Failed to connect to robot")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("✅ Robot connected!")
    node.get_logger().info("")
    node.get_logger().info("=" * 60)
    node.get_logger().info("  Complete UR Control Example (Python)")
    node.get_logger().info("=" * 60)
    node.get_logger().info("")

    time.sleep(1)

    # ========== 1. Speed Control ==========
    node.get_logger().info("=" * 40)
    node.get_logger().info("1. Speed Control")
    node.get_logger().info("=" * 40)
    node.get_logger().info("Setting speed to 30% for safe operation...")
    robot.set_speed_slider(0.3)
    time.sleep(1)

    # ========== 2. MoveJ to Home ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("2. MoveJ to Home")
    node.get_logger().info("=" * 40)
    home = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

    if robot.move_j(home, 0.3, True):
        node.get_logger().info("✅ Reached HOME position")

        # Get current joint positions
        current_joints = robot.get_joint_positions()
        if current_joints:
            node.get_logger().info("Current joint positions:")
            for i, j in enumerate(current_joints):
                node.get_logger().info(f"  Joint[{i}]: {j:.4f} rad")
    else:
        node.get_logger().error("❌ Failed to reach HOME")

    time.sleep(2)

    # ========== 3. Digital Output Control ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("3. Digital Output Control")
    node.get_logger().info("=" * 40)
    node.get_logger().info("Turning ON DO[0] (Standard output)...")
    robot.set_digital_out(0, True)
    time.sleep(1)

    # ========== 4. MoveJ to Pre-Pick Position ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("4. MoveJ to Pre-Pick")
    node.get_logger().info("=" * 40)
    pre_pick = [0.5, -1.2, 1.0, -1.5, -1.57, 0.5]

    if robot.move_j(pre_pick, 0.3, True):
        node.get_logger().info("✅ Reached Pre-Pick position")
    else:
        node.get_logger().error("❌ Failed to reach Pre-Pick")

    time.sleep(1)

    # ========== 5. MoveL Down (Simulated Pick) ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("5. MoveL Down (Pick)")
    node.get_logger().info("=" * 40)

    # Create T-matrix for downward motion
    tmatrix_down = create_tmatrix(
        -0.4,
        -0.2,
        0.2,  # Position (x, y, z) in meters
        math.pi,
        0.0,
        0.0,  # Rotation (rx, ry, rz) - tool pointing down
    )

    node.get_logger().info("Moving down to pick position...")
    if robot.move_l(tmatrix_down, 0.2, True):
        node.get_logger().info("✅ Reached pick position")
    else:
        node.get_logger().warn(
            "⚠️ MoveL might have failed (check if position is reachable)"
        )

    time.sleep(1)

    # ========== 6. Gripper Control (Simulated with DO) ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("6. Gripper Control (DO[1])")
    node.get_logger().info("=" * 40)
    node.get_logger().info("Closing gripper (DO[1] = HIGH)...")
    robot.set_digital_out(1, True)
    time.sleep(1)

    # ========== 7. MoveL Up ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("7. MoveL Up")
    node.get_logger().info("=" * 40)

    tmatrix_up = create_tmatrix(
        -0.4, -0.2, 0.4, math.pi, 0.0, 0.0  # Position (10cm higher)
    )

    node.get_logger().info("Moving up with object...")
    if robot.move_l(tmatrix_up, 0.2, True):
        node.get_logger().info("✅ Moved up successfully")
    else:
        node.get_logger().warn("⚠️ MoveL might have failed")

    time.sleep(1)

    # ========== 8. MoveJ to Place Position ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("8. MoveJ to Place Position")
    node.get_logger().info("=" * 40)
    place = [-0.5, -1.2, 1.0, -1.5, -1.57, -0.5]

    if robot.move_j(place, 0.3, True):
        node.get_logger().info("✅ Reached Place position")
    else:
        node.get_logger().error("❌ Failed to reach Place position")

    time.sleep(1)

    # ========== 9. Release Object ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("9. Release Object")
    node.get_logger().info("=" * 40)
    node.get_logger().info("Opening gripper (DO[1] = LOW)...")
    robot.set_digital_out(1, False)
    time.sleep(1)

    # ========== 10. Return to Home ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("10. Return to Home")
    node.get_logger().info("=" * 40)

    if robot.move_j(home, 0.3, True):
        node.get_logger().info("✅ Returned to HOME")
    else:
        node.get_logger().error("❌ Failed to return to HOME")

    # ========== 11. Cleanup ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("11. Cleanup")
    node.get_logger().info("=" * 40)
    node.get_logger().info("Turning OFF all outputs...")
    robot.set_digital_out(0, False)
    robot.set_digital_out(1, False)

    # ========== 12. Reset Speed ==========
    node.get_logger().info("Resetting speed to 100%...")
    robot.set_speed_slider(1.0)

    time.sleep(1)

    node.get_logger().info("")
    node.get_logger().info("=" * 60)
    node.get_logger().info("  ✅ Complete Example Finished!")
    node.get_logger().info("=" * 60)
    node.get_logger().info("")
    node.get_logger().info("Demonstrated features:")
    node.get_logger().info("  ✓ Speed slider control")
    node.get_logger().info("  ✓ MoveJ (joint space motion)")
    node.get_logger().info("  ✓ MoveL (Cartesian linear motion)")
    node.get_logger().info("  ✓ Digital I/O control")
    node.get_logger().info("  ✓ State monitoring")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
