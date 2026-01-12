#!/usr/bin/env python3
"""
I/O and Speed Control Example (Python)
=======================================
Demonstrates how to control:
- Digital I/O (input/output)
- Speed slider

Usage:
    ros2 run ur_control_client_py example_io_speed
"""

import time

import rclpy
from rclpy.node import Node
from ur_control_client_py import URRobotController


def main(args=None):
    rclpy.init(args=args)

    # Create ROS2 node
    node = Node("ur_control_example_io_speed")

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
    node.get_logger().info("  I/O and Speed Control Example (Python)")
    node.get_logger().info("=" * 60)
    node.get_logger().info("")

    # Wait for I/O states to be received
    time.sleep(1)

    # ========== Speed Slider Control ==========
    node.get_logger().info("=" * 40)
    node.get_logger().info("Speed Slider Control")
    node.get_logger().info("=" * 40)

    # Get current speed scaling
    current_speed = robot.get_speed_scaling()
    node.get_logger().info(f"Current speed scaling: {current_speed * 100:.1f}%")

    # Set speed to 50%
    node.get_logger().info("Setting speed slider to 50%...")
    if robot.set_speed_slider(0.5):
        node.get_logger().info("✅ Speed slider set successfully")
    else:
        node.get_logger().warn("❌ Failed to set speed slider")

    time.sleep(2)

    # Set speed to 100%
    node.get_logger().info("Setting speed slider to 100%...")
    if robot.set_speed_slider(1.0):
        node.get_logger().info("✅ Speed slider set successfully")
    else:
        node.get_logger().warn("❌ Failed to set speed slider")

    time.sleep(2)

    # ========== Digital I/O Control ==========
    node.get_logger().info("")
    node.get_logger().info("=" * 40)
    node.get_logger().info("Digital I/O Control")
    node.get_logger().info("=" * 40)

    # Read digital inputs
    node.get_logger().info("Reading digital inputs...")
    for pin in range(18):
        state = robot.get_digital_in(pin)
        if pin <= 7:
            pin_type = "Standard"
        elif pin <= 15:
            pin_type = "Configurable"
        else:
            pin_type = "Tool"

        node.get_logger().info(
            f'  DI[{pin:2d}] ({pin_type:13s}): {"HIGH" if state else "LOW"}'
        )

    time.sleep(1)

    # Set digital outputs
    node.get_logger().info("")
    node.get_logger().info("Setting digital outputs...")

    # Example: Set DO[0] to HIGH
    node.get_logger().info("Setting DO[0] (Standard) to HIGH...")
    if robot.set_digital_out(0, True):
        node.get_logger().info("✅ DO[0] set to HIGH")
    else:
        node.get_logger().warn("❌ Failed to set DO[0]")

    time.sleep(2)

    # Set DO[0] to LOW
    node.get_logger().info("Setting DO[0] to LOW...")
    if robot.set_digital_out(0, False):
        node.get_logger().info("✅ DO[0] set to LOW")
    else:
        node.get_logger().warn("❌ Failed to set DO[0]")

    time.sleep(1)

    # Example: Toggle multiple outputs
    node.get_logger().info("")
    node.get_logger().info("Toggling DO[0-3] (Standard outputs)...")

    for i in range(3):
        node.get_logger().info(f"Cycle {i + 1}/3:")

        # Turn ON
        for pin in range(4):
            robot.set_digital_out(pin, True)
            time.sleep(0.2)

        # Turn OFF
        for pin in range(4):
            robot.set_digital_out(pin, False)
            time.sleep(0.2)

    # Read digital outputs
    node.get_logger().info("")
    node.get_logger().info("Reading digital outputs...")
    for pin in range(18):
        state = robot.get_digital_out(pin)
        if pin <= 7:
            pin_type = "Standard"
        elif pin <= 15:
            pin_type = "Configurable"
        else:
            pin_type = "Tool"

        node.get_logger().info(
            f'  DO[{pin:2d}] ({pin_type:13s}): {"HIGH" if state else "LOW"}'
        )

    node.get_logger().info("")
    node.get_logger().info("=" * 60)
    node.get_logger().info("  I/O and Speed Control Example Completed!")
    node.get_logger().info("=" * 60)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
