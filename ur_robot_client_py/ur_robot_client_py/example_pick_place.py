#!/usr/bin/env python3
"""
Pick & Place Example (Python)
==========================
Demonstrates all features:
- MoveJ (joint space motion)
- MoveL (Cartesian linear motion)
- Speed slider control
- Digital I/O control
- State monitoring

Usage:
    ros2 run ur_robot_client_py example_pick_place
"""

import asyncio
import math

import rclpy
from rclpy.node import Node
from ur_robot_client_py import URRobotClient


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


async def spin_node(node):
    """Spin ROS node in async loop"""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        await asyncio.sleep(0.01)


async def main_async():
    rclpy.init()

    # Create ROS2 node
    node = Node("ur_control_example_pick_place")

    # Create robot controller
    robot = URRobotClient(node)

    spin_task = asyncio.create_task(spin_node(node))

    # Wait for robot ready (IO included)
    node.get_logger().info("Waiting for robot connection and to be ready...")
    ready = await robot.wait_robot_ready(timeout=5.0, require_io=True)
    if not ready:
        node.get_logger().error(
            "❌ Robot not ready (timeout waiting for joint/speed/tcp/io)"
        )
        spin_task.cancel()
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("✅ Robot connected and ready!")
    node.get_logger().info("")
    node.get_logger().info("=" * 60)
    node.get_logger().info("  Pick & Place Example (Python)")
    node.get_logger().info("=" * 60)
    node.get_logger().info("")

    await asyncio.sleep(0.5)

    # ========== 1. Speed Control ==========
    node.get_logger().info("=" * 40)
    node.get_logger().info("1. Speed Control")
    node.get_logger().info("=" * 40)
    node.get_logger().info("Setting speed to 30% for safe operation...")

    ok, msg = await robot.set_speed_slider(0.3)
    if not ok:
        node.get_logger().warn(f"⚠️ Failed to set speed slider: {msg}")

    await asyncio.sleep(1.0)

    # ========== 2. MoveJ to Home ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("2. MoveJ to Home")
    node.get_logger().info("=" * 40)

    home = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    ok, msg = await robot.move_j(home, velocity=0.3)
    if ok:
        node.get_logger().info("✅ Reached HOME position")

        # Get current joint positions
        current_joints = robot.get_joint_positions()
        if current_joints:
            node.get_logger().info("Current joint positions:")
            for i, j in enumerate(current_joints):
                node.get_logger().info(f"  Joint[{i}]: {j:.4f} rad")
    else:
        node.get_logger().error(f"❌ Failed to reach HOME: {msg}")

    await asyncio.sleep(2.0)

    # ========== 3. Digital Output Control ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("3. Digital Output Control")
    node.get_logger().info("=" * 40)
    node.get_logger().info("Turning ON DO[0] (Standard output)...")

    ok, msg = await robot.set_digital_out(0, True)
    if not ok:
        node.get_logger().warn(f"⚠️ Failed to set DO[0]: {msg}")

    await asyncio.sleep(1.0)

    # ========== 4. MoveJ to Pre-Pick Position ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("4. MoveJ to Pre-Pick")
    node.get_logger().info("=" * 40)

    pre_pick = [0.5, -1.2, 1.0, -1.5, -1.57, 0.5]
    ok, msg = await robot.move_j(pre_pick, velocity=0.3)
    if ok:
        node.get_logger().info("✅ Reached Pre-Pick position")
    else:
        node.get_logger().error(f"❌ Failed to reach Pre-Pick: {msg}")

    await asyncio.sleep(1.0)

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
    ok, msg = await robot.move_l(tmatrix_down, velocity=0.2)
    if ok:
        node.get_logger().info("✅ Reached pick position")
    else:
        node.get_logger().warn(f"⚠️ MoveL failed: {msg}")

    await asyncio.sleep(1.0)

    # ========== 6. Gripper Control (Simulated with DO) ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("6. Gripper Control (DO[1])")
    node.get_logger().info("=" * 40)
    node.get_logger().info("Closing gripper (DO[1] = HIGH)...")

    ok, msg = await robot.set_digital_out(1, True)
    if not ok:
        node.get_logger().warn(f"⚠️ Failed to set DO[1]: {msg}")

    await asyncio.sleep(1.0)

    # ========== 7. MoveL Up ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("7. MoveL Up")
    node.get_logger().info("=" * 40)

    tmatrix_up = create_tmatrix(
        -0.4,
        -0.2,
        0.4,
        math.pi,
        0.0,
        0.0,
    )

    node.get_logger().info("Moving up with object...")
    ok, msg = await robot.move_l(tmatrix_up, velocity=0.2)
    if ok:
        node.get_logger().info("✅ Moved up successfully")
    else:
        node.get_logger().warn(f"⚠️ MoveL failed: {msg}")

    await asyncio.sleep(1.0)

    # ========== 8. MoveJ to Place Position ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("8. MoveJ to Place Position")
    node.get_logger().info("=" * 40)

    place = [-0.5, -1.2, 1.0, -1.5, -1.57, -0.5]
    ok, msg = await robot.move_j(place, velocity=0.3)
    if ok:
        node.get_logger().info("✅ Reached Place position")
    else:
        node.get_logger().error(f"❌ Failed to reach Place position: {msg}")

    await asyncio.sleep(1.0)

    # ========== 9. Release Object ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("9. Release Object")
    node.get_logger().info("=" * 40)

    node.get_logger().info("Opening gripper (DO[1] = LOW)...")
    ok, msg = await robot.set_digital_out(1, False)
    if not ok:
        node.get_logger().warn(f"⚠️ Failed to clear DO[1]: {msg}")

    await asyncio.sleep(1.0)

    # ========== 10. Return to Home ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("10. Return to Home")
    node.get_logger().info("=" * 40)

    ok, msg = await robot.move_j(home, velocity=0.3)
    if ok:
        node.get_logger().info("✅ Returned to HOME")
    else:
        node.get_logger().error(f"❌ Failed to return to HOME: {msg}")

    # ========== 11. Cleanup ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("11. Cleanup")
    node.get_logger().info("=" * 40)

    node.get_logger().info("Turning OFF all outputs...")
    await robot.set_digital_out(0, False)
    await robot.set_digital_out(1, False)

    # ========== 12. Reset Speed ==========
    node.get_logger().info("Resetting speed to 100%...")
    await robot.set_speed_slider(1.0)

    await asyncio.sleep(0.5)

    node.get_logger().info("")
    node.get_logger().info("=" * 60)
    node.get_logger().info("  ✅ Pick & Place Example Finished!")
    node.get_logger().info("=" * 60)
    node.get_logger().info("")
    node.get_logger().info("Demonstrated features:")
    node.get_logger().info("  ✓ Speed slider control")
    node.get_logger().info("  ✓ MoveJ (joint space motion)")
    node.get_logger().info("  ✓ MoveL (Cartesian linear motion)")
    node.get_logger().info("  ✓ Digital I/O control")
    node.get_logger().info("  ✓ State monitoring")

    spin_task.cancel()
    node.destroy_node()
    rclpy.shutdown()


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
