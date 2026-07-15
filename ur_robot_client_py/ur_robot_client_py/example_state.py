#!/usr/bin/env python3
"""
Robot State Monitoring Example (Python)
========================================
Read-only — safe to run on a real robot (no motion, no output changes).
Prints every second:
- Joint positions
- TCP pose (via TF)
- Speed slider / speed scaling
- Digital I/O states
- Program running (external control) state

Usage:
    ros2 run ur_robot_client_py example_state
"""

import asyncio

import rclpy
from rclpy.node import Node

from ur_robot_client_py import URRobotClient


async def spin_node(node):
    """Spin ROS node in async loop"""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        await asyncio.sleep(0.01)


async def main_async():
    rclpy.init()

    # Create ROS2 node
    node = Node("ur_control_example_state")

    # Create robot controller
    robot = URRobotClient(node)

    # Start ROS spinning task
    spin_task = asyncio.create_task(spin_node(node))

    # Wait for robot connection
    node.get_logger().info("Waiting for robot connection...")
    ready = await robot.wait_robot_ready(timeout=10.0)

    if not ready:
        node.get_logger().error("❌ Robot not ready (timeout)")
        spin_task.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    node.get_logger().info("✅ Robot connected!")
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("  Robot State Monitoring (read-only)")
    node.get_logger().info("  Press Ctrl+C to stop")
    node.get_logger().info("=" * 40)

    while rclpy.ok():
        # Joint positions [rad]
        joints = robot.get_joint_positions()
        if joints:
            joints_str = ", ".join(f"{j:.3f}" for j in joints)
            node.get_logger().info(f"Joints [rad]: [{joints_str}]")
        else:
            node.get_logger().info("Joints [rad]: not available")

        # TCP pose (4x4 T-matrix, translation = column 3)
        if robot.is_tcp_pose_available():
            tcp = robot.get_tcp_pose()
            node.get_logger().info(
                f"TCP [m]     : x={tcp[0, 3]:.3f}, y={tcp[1, 3]:.3f}, z={tcp[2, 3]:.3f}"
            )
        else:
            node.get_logger().info("TCP [m]     : not available (TF)")

        # Speed slider (user-set) vs speed scaling (actual robot speed)
        node.get_logger().info(
            f"Speed       : slider={robot.get_speed_slider() * 100:.0f}%, "
            f"scaling={robot.get_speed_scaling() * 100:.0f}%"
        )

        # Digital I/O (standard pins 0-7, printed as bit string)
        di = "".join("1" if robot.get_digital_in(p) else "0" for p in range(8))
        do = "".join("1" if robot.get_digital_out(p) else "0" for p in range(8))
        node.get_logger().info(f"I/O [0-7]   : DI={di}, DO={do}")

        # Program (external control) state — False means control lost
        # (e-stop / Local mode). Not published on fake hardware.
        if robot.is_program_running():
            state = "RUNNING (control OK)"
        elif not robot.program_state_received:
            state = "unknown (not published — fake hardware?)"
        else:
            state = "STOPPED (control lost)"
        node.get_logger().info(f"Program     : {state}")

        node.get_logger().info("-" * 40)
        await asyncio.sleep(1.0)

    # Loop exits when rclpy.ok() turns False (Ctrl+C already shut the
    # context down) — only shut down explicitly if still up
    spin_task.cancel()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def main():
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
