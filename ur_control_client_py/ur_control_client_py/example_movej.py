#!/usr/bin/env python3
"""
MoveJ Motion Control Example (Python)
======================================
Demonstrates how to control UR robot using joint space motion (MoveJ)

Usage:
    ros2 run ur_control_client_py example_movej
"""

import rclpy
from ur_control_client_py.ur_control_client import URControlClient
import time


def main(args=None):
    rclpy.init(args=args)
    
    client = URControlClient()
    
    # Wait for robot connection
    client.get_logger().info('Waiting for robot connection...')
    while rclpy.ok() and not client.is_connected():
        rclpy.spin_once(client, timeout_sec=0.1)
    
    if not client.is_connected():
        client.get_logger().error('Failed to connect to robot')
        rclpy.shutdown()
        return
    
    client.get_logger().info('✅ Robot connected!')
    client.get_logger().info('')
    client.get_logger().info('=' * 60)
    client.get_logger().info('  MoveJ Example: Joint Space Motion (Python)')
    client.get_logger().info('=' * 60)
    client.get_logger().info('')
    
    # Example 1: Move to home position
    client.get_logger().info('Example 1: Moving to HOME position')
    home = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    
    if client.move_j(home, 0.3, True):
        client.get_logger().info('✅ Reached HOME position')
    else:
        client.get_logger().error('❌ Failed to reach HOME position')
    
    time.sleep(1)
    
    # Example 2: Move to target position
    client.get_logger().info('')
    client.get_logger().info('Example 2: Moving to TARGET position')
    target = [0.5, -1.2, 1.0, -1.5, -1.57, 0.5]
    
    if client.move_j(target, 0.5, True):
        client.get_logger().info('✅ Reached TARGET position')
    else:
        client.get_logger().error('❌ Failed to reach TARGET position')
    
    time.sleep(1)
    
    # Example 3: Return to home
    client.get_logger().info('')
    client.get_logger().info('Example 3: Returning to HOME')
    
    if client.move_j(home, 0.3, True):
        client.get_logger().info('✅ Returned to HOME position')
    else:
        client.get_logger().error('❌ Failed to return to HOME')
    
    client.get_logger().info('')
    client.get_logger().info('=' * 60)
    client.get_logger().info('  MoveJ Example Completed!')
    client.get_logger().info('=' * 60)
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

