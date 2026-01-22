#!/usr/bin/env python3
"""
I/O and Speed Control Example (Python)
=======================================
Demonstrates how to control:
- Digital I/O (input/output)
- Speed slider

Usage:
    ros2 run ur_robot_client_py example_io_speed
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
    node = Node("ur_control_example_io_speed")

    # Create robot controller
    robot = URRobotClient(node)

    # Start ROS spinning task
    spin_task = asyncio.create_task(spin_node(node))

    # Wait for robot ready (IO 포함)
    node.get_logger().info("Waiting for robot connection and to be ready...")
    ready = await robot.wait_robot_ready(timeout=5.0, require_io=True)

    if not ready:
        node.get_logger().error("❌ Robot not ready (timeout)")
        spin_task.cancel()
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("✅ Robot connected and ready!")
    node.get_logger().info("")
    node.get_logger().info("=" * 60)
    node.get_logger().info("  I/O and Speed Control Example (Python)")
    node.get_logger().info("=" * 60)
    node.get_logger().info("")

    # ========== Speed Slider Control ==========
    node.get_logger().info("=" * 40)
    node.get_logger().info("Speed Slider Control")
    node.get_logger().info("=" * 40)

    # Get current speed scaling
    current_speed = robot.get_speed_scaling()
    node.get_logger().info(f"Current speed scaling: {current_speed * 100:.1f}%")

    # Set speed to 50%
    node.get_logger().info("Setting speed slider to 50%...")
    success, _ = await robot.set_speed_slider(0.5)
    if success:
        node.get_logger().info("✅ Speed slider set to 50%")
    else:
        node.get_logger().warn("❌ Failed to set speed slider")

    await asyncio.sleep(2.0)

    # Set speed to 100%
    node.get_logger().info("Setting speed slider to 100%...")
    success, _ = await robot.set_speed_slider(1.0)
    if success:
        node.get_logger().info("✅ Speed slider set to 100%")
    else:
        node.get_logger().warn("❌ Failed to set speed slider")

    await asyncio.sleep(1.0)

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

    await asyncio.sleep(1.0)

    # Set digital outputs
    node.get_logger().info("")
    node.get_logger().info("Setting digital outputs...")

    # Example: Set DO[0] to HIGH
    node.get_logger().info("Setting DO[0] (Standard) to HIGH...")
    success, _ = await robot.set_digital_out(0, True)
    if success:
        node.get_logger().info("✅ DO[0] set to HIGH")

    await asyncio.sleep(2.0)

    # Set DO[0] to LOW
    node.get_logger().info("Setting DO[0] to LOW...")
    success, _ = await robot.set_digital_out(0, False)
    if success:
        node.get_logger().info("✅ DO[0] set to LOW")

    await asyncio.sleep(1.0)

    # Example: Toggle multiple outputs
    node.get_logger().info("")
    node.get_logger().info("Toggling DO[0-3] (Standard outputs)...")

    for i in range(3):
        node.get_logger().info(f"Cycle {i + 1}/3")

        # Turn ON
        for pin in range(4):
            await robot.set_digital_out(pin, True)
            await asyncio.sleep(0.2)

        # Turn OFF
        for pin in range(4):
            await robot.set_digital_out(pin, False)
            await asyncio.sleep(0.2)

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

    spin_task.cancel()
    node.destroy_node()
    rclpy.shutdown()


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
