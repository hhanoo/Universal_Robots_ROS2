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

import rclpy
from ur_control_client_py.ur_control_client import URControlClient
import time
import math
import numpy as np


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
    client.get_logger().info('  Complete UR Control Example (Python)')
    client.get_logger().info('=' * 60)
    client.get_logger().info('')

    time.sleep(1)

    # ========== 1. Speed Control ==========
    client.get_logger().info('=' * 40)
    client.get_logger().info('1. Speed Control')
    client.get_logger().info('=' * 40)
    client.get_logger().info('Setting speed to 30% for safe operation...')
    client.set_speed_slider(0.3)
    time.sleep(1)

    # ========== 2. MoveJ to Home ==========
    client.get_logger().info('')
    client.get_logger().info('=' * 40)
    client.get_logger().info('2. MoveJ to Home')
    client.get_logger().info('=' * 40)
    home = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

    if client.move_j(home, 0.3, True):
        client.get_logger().info('✅ Reached HOME position')

        # Get current joint positions
        current_joints = client.get_joint_positions()
        if current_joints:
            client.get_logger().info('Current joint positions:')
            for i, j in enumerate(current_joints):
                client.get_logger().info(f'  Joint[{i}]: {j:.4f} rad')
    else:
        client.get_logger().error('❌ Failed to reach HOME')

    time.sleep(2)

    # ========== 3. Digital Output Control ==========
    client.get_logger().info('')
    client.get_logger().info('=' * 40)
    client.get_logger().info('3. Digital Output Control')
    client.get_logger().info('=' * 40)
    client.get_logger().info('Turning ON DO[0] (Standard output)...')
    client.set_digital_out(0, True)
    time.sleep(1)

    # ========== 4. MoveJ to Pre-Pick Position ==========
    client.get_logger().info('')
    client.get_logger().info('=' * 40)
    client.get_logger().info('4. MoveJ to Pre-Pick')
    client.get_logger().info('=' * 40)
    pre_pick = [0.5, -1.2, 1.0, -1.5, -1.57, 0.5]

    if client.move_j(pre_pick, 0.3, True):
        client.get_logger().info('✅ Reached Pre-Pick position')
    else:
        client.get_logger().error('❌ Failed to reach Pre-Pick')

    time.sleep(1)

    # ========== 5. MoveL Down (Simulated Pick) ==========
    client.get_logger().info('')
    client.get_logger().info('=' * 40)
    client.get_logger().info('5. MoveL Down (Pick)')
    client.get_logger().info('=' * 40)

    # Create T-matrix for downward motion
    tmatrix_down = create_tmatrix(
        -0.4,
        -0.2,
        0.2,  # Position (x, y, z) in meters
        math.pi,
        0.0,
        0.0  # Rotation (rx, ry, rz) - tool pointing down
    )

    client.get_logger().info('Moving down to pick position...')
    if client.move_l(tmatrix_down, 0.2, True):
        client.get_logger().info('✅ Reached pick position')
    else:
        client.get_logger().warn('⚠️ MoveL might have failed (check if position is reachable)')

    time.sleep(1)

    # ========== 6. Gripper Control (Simulated with DO) ==========
    client.get_logger().info('')
    client.get_logger().info('=' * 40)
    client.get_logger().info('6. Gripper Control (DO[1])')
    client.get_logger().info('=' * 40)
    client.get_logger().info('Closing gripper (DO[1] = HIGH)...')
    client.set_digital_out(1, True)
    time.sleep(1)

    # ========== 7. MoveL Up ==========
    client.get_logger().info('')
    client.get_logger().info('=' * 40)
    client.get_logger().info('7. MoveL Up')
    client.get_logger().info('=' * 40)

    tmatrix_up = create_tmatrix(
        -0.4,
        -0.2,
        0.4,  # Position (10cm higher)
        math.pi,
        0.0,
        0.0)

    client.get_logger().info('Moving up with object...')
    if client.move_l(tmatrix_up, 0.2, True):
        client.get_logger().info('✅ Moved up successfully')
    else:
        client.get_logger().warn('⚠️ MoveL might have failed')

    time.sleep(1)

    # ========== 8. MoveJ to Place Position ==========
    client.get_logger().info('')
    client.get_logger().info('=' * 40)
    client.get_logger().info('8. MoveJ to Place Position')
    client.get_logger().info('=' * 40)
    place = [-0.5, -1.2, 1.0, -1.5, -1.57, -0.5]

    if client.move_j(place, 0.3, True):
        client.get_logger().info('✅ Reached Place position')
    else:
        client.get_logger().error('❌ Failed to reach Place position')

    time.sleep(1)

    # ========== 9. Release Object ==========
    client.get_logger().info('')
    client.get_logger().info('=' * 40)
    client.get_logger().info('9. Release Object')
    client.get_logger().info('=' * 40)
    client.get_logger().info('Opening gripper (DO[1] = LOW)...')
    client.set_digital_out(1, False)
    time.sleep(1)

    # ========== 10. Return to Home ==========
    client.get_logger().info('')
    client.get_logger().info('=' * 40)
    client.get_logger().info('10. Return to Home')
    client.get_logger().info('=' * 40)

    if client.move_j(home, 0.3, True):
        client.get_logger().info('✅ Returned to HOME')
    else:
        client.get_logger().error('❌ Failed to return to HOME')

    # ========== 11. Cleanup ==========
    client.get_logger().info('')
    client.get_logger().info('=' * 40)
    client.get_logger().info('11. Cleanup')
    client.get_logger().info('=' * 40)
    client.get_logger().info('Turning OFF all outputs...')
    client.set_digital_out(0, False)
    client.set_digital_out(1, False)

    # ========== 12. Reset Speed ==========
    client.get_logger().info('Resetting speed to 100%...')
    client.set_speed_slider(1.0)

    time.sleep(1)

    client.get_logger().info('')
    client.get_logger().info('=' * 60)
    client.get_logger().info('  ✅ Complete Example Finished!')
    client.get_logger().info('=' * 60)
    client.get_logger().info('')
    client.get_logger().info('Demonstrated features:')
    client.get_logger().info('  ✓ Speed slider control')
    client.get_logger().info('  ✓ MoveJ (joint space motion)')
    client.get_logger().info('  ✓ MoveL (Cartesian linear motion)')
    client.get_logger().info('  ✓ Digital I/O control')
    client.get_logger().info('  ✓ State monitoring')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
