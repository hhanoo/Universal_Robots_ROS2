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
    client.get_logger().info('  I/O and Speed Control Example (Python)')
    client.get_logger().info('=' * 60)
    client.get_logger().info('')
    
    # Wait for I/O states to be received
    time.sleep(1)
    
    # ========== Speed Slider Control ==========
    client.get_logger().info('=' * 40)
    client.get_logger().info('Speed Slider Control')
    client.get_logger().info('=' * 40)
    
    # Get current speed scaling
    current_speed = client.get_speed_scaling()
    client.get_logger().info(f'Current speed scaling: {current_speed * 100:.1f}%')
    
    # Set speed to 50%
    client.get_logger().info('Setting speed slider to 50%...')
    if client.set_speed_slider(0.5):
        client.get_logger().info('✅ Speed slider set successfully')
    else:
        client.get_logger().warn('❌ Failed to set speed slider')
    
    time.sleep(2)
    
    # Set speed to 100%
    client.get_logger().info('Setting speed slider to 100%...')
    if client.set_speed_slider(1.0):
        client.get_logger().info('✅ Speed slider set successfully')
    else:
        client.get_logger().warn('❌ Failed to set speed slider')
    
    time.sleep(2)
    
    # ========== Digital I/O Control ==========
    client.get_logger().info('')
    client.get_logger().info('=' * 40)
    client.get_logger().info('Digital I/O Control')
    client.get_logger().info('=' * 40)
    
    # Read digital inputs
    client.get_logger().info('Reading digital inputs...')
    for pin in range(18):
        state = client.get_digital_in(pin)
        if pin <= 7:
            pin_type = 'Standard'
        elif pin <= 15:
            pin_type = 'Configurable'
        else:
            pin_type = 'Tool'
        
        client.get_logger().info(f'  DI[{pin:2d}] ({pin_type:13s}): {"HIGH" if state else "LOW"}')
    
    time.sleep(1)
    
    # Set digital outputs
    client.get_logger().info('')
    client.get_logger().info('Setting digital outputs...')
    
    # Example: Set DO[0] to HIGH
    client.get_logger().info('Setting DO[0] (Standard) to HIGH...')
    if client.set_digital_out(0, True):
        client.get_logger().info('✅ DO[0] set to HIGH')
    else:
        client.get_logger().warn('❌ Failed to set DO[0]')
    
    time.sleep(2)
    
    # Set DO[0] to LOW
    client.get_logger().info('Setting DO[0] to LOW...')
    if client.set_digital_out(0, False):
        client.get_logger().info('✅ DO[0] set to LOW')
    else:
        client.get_logger().warn('❌ Failed to set DO[0]')
    
    time.sleep(1)
    
    # Example: Toggle multiple outputs
    client.get_logger().info('')
    client.get_logger().info('Toggling DO[0-3] (Standard outputs)...')
    
    for i in range(3):
        client.get_logger().info(f'Cycle {i + 1}/3:')
        
        # Turn ON
        for pin in range(4):
            client.set_digital_out(pin, True)
            time.sleep(0.2)
        
        # Turn OFF
        for pin in range(4):
            client.set_digital_out(pin, False)
            time.sleep(0.2)
    
    # Read digital outputs
    client.get_logger().info('')
    client.get_logger().info('Reading digital outputs...')
    for pin in range(18):
        state = client.get_digital_out(pin)
        if pin <= 7:
            pin_type = 'Standard'
        elif pin <= 15:
            pin_type = 'Configurable'
        else:
            pin_type = 'Tool'
        
        client.get_logger().info(f'  DO[{pin:2d}] ({pin_type:13s}): {"HIGH" if state else "LOW"}')
    
    client.get_logger().info('')
    client.get_logger().info('=' * 60)
    client.get_logger().info('  I/O and Speed Control Example Completed!')
    client.get_logger().info('=' * 60)
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

