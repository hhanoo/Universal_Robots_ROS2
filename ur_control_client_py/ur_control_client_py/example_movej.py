#!/usr/bin/env python3
"""
MoveJ Motion Control Example (Python)
======================================
Demonstrates how to control UR robot using joint space motion (MoveJ)

Usage:
    ros2 run ur_control_client_py example_movej
"""

import time

import rclpy
from rclpy.node import Node
from ur_control_client_py import URRobotController


def main(args=None):
    rclpy.init(args=args)

    # Create ROS2 node
    node = Node("ur_control_example_movej")

    # Create robot controller
    client = URRobotController(node)

    # Wait for robot connection
    node.get_logger().info("Waiting for robot connection...")
    while rclpy.ok() and not client.is_connected():
        rclpy.spin_once(node, timeout_sec=0.1)

    if not client.is_connected():
        node.get_logger().error("Failed to connect to robot")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("✅ Robot connected!")
    node.get_logger().info("")
    node.get_logger().info("=" * 60)
    node.get_logger().info("  MoveJ Example: Joint Space Motion (Python)")
    node.get_logger().info("=" * 60)
    node.get_logger().info("")

    # Example 1: Move to home position
    node.get_logger().info("Example 1: Moving to HOME position")
    home = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

    if client.move_j(home, 0.3, True):
        node.get_logger().info("✅ Reached HOME position")
    else:
        node.get_logger().error("❌ Failed to reach HOME position")

    time.sleep(1)

    # Example 2: Move to target position
    node.get_logger().info("")
    node.get_logger().info("Example 2: Moving to TARGET position")
    target = [0.5, -1.2, 1.0, -1.5, -1.57, 0.5]

    if client.move_j(target, 0.5, True):
        node.get_logger().info("✅ Reached TARGET position")
    else:
        node.get_logger().error("❌ Failed to reach TARGET position")

    time.sleep(1)

    # Example 3: Return to home
    node.get_logger().info("")
    node.get_logger().info("Example 3: Returning to HOME")

    if client.move_j(home, 0.3, True):
        node.get_logger().info("✅ Returned to HOME position")
    else:
        node.get_logger().error("❌ Failed to return to HOME")

    node.get_logger().info("")
    node.get_logger().info("=" * 60)
    node.get_logger().info("  MoveJ Example Completed!")
    node.get_logger().info("=" * 60)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
