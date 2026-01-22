#!/usr/bin/env python3
"""
MoveJ Motion Control Example (Python)
======================================
Demonstrates how to control UR robot using joint space motion (MoveJ)
"""

import asyncio

import rclpy
from rclpy.node import Node
from ur_control_client_py import URRobotClient


async def spin_node(node):
    """Spin ROS node in async loop"""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        await asyncio.sleep(0.01)


async def main_async():
    rclpy.init()

    # Create ROS2 node
    node = Node("ur_control_example_movej")

    # Create robot controller
    client = URRobotClient(node)

    # Start ROS spinning task
    spin_task = asyncio.create_task(spin_node(node))

    # Wait for robot connection
    node.get_logger().info("Waiting for robot connection and to be ready...")
    ready = await client.wait_robot_ready(timeout=5.0)

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
    node.get_logger().info("  MoveJ Example: Joint Space Motion (Python)")
    node.get_logger().info("=" * 60)
    node.get_logger().info("")

    # Example 1: Move to home position
    node.get_logger().info("Example 1: Moving to HOME position")
    home = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

    success, msg = await client.move_j(home, velocity=0.3)
    if success:
        node.get_logger().info("✅ Reached HOME position")
    else:
        node.get_logger().error(f"❌ Failed: {msg}")

    await client.wait(1.0)

    # Example 2: Move to target position
    node.get_logger().info("")
    node.get_logger().info("Example 2: Moving to TARGET position")
    target = [0.5, -1.2, 1.0, -1.5, -1.57, 0.5]

    success, msg = await client.move_j(target, velocity=0.5)
    if success:
        node.get_logger().info("✅ Reached TARGET position")
    else:
        node.get_logger().error(f"❌ Failed: {msg}")

    await client.wait(1.0)

    # Example 3: Return to home
    node.get_logger().info("")
    node.get_logger().info("Example 3: Returning to HOME")

    success, msg = await client.move_j(home, velocity=0.3)
    if success:
        node.get_logger().info("✅ Returned to HOME position")
    else:
        node.get_logger().error(f"❌ Failed: {msg}")

    node.get_logger().info("")
    node.get_logger().info("=" * 60)
    node.get_logger().info("  MoveJ Example Completed!")
    node.get_logger().info("=" * 60)

    spin_task.cancel()
    node.destroy_node()
    rclpy.shutdown()


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
