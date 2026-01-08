#!/usr/bin/env python3
"""
Launch all nodes for UR robot control system
- UR Robot Driver
- MoveIt
- Motion Action Server
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ===============================
    # Launch Arguments
    # ===============================
    robot_ip_arg = DeclareLaunchArgument('robot_ip',
                                         default_value='127.0.0.1',
                                         description='IP address of the UR robot (use 127.0.0.1 for simulation)')

    ur_type_arg = DeclareLaunchArgument('ur_type',
                                        default_value='ur10e',
                                        description='Type of UR robot (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30)')

    use_fake_hardware_arg = DeclareLaunchArgument('use_fake_hardware',
                                                  default_value='false',
                                                  description='Use fake hardware (simulation mode)')

    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true', description='Launch RViz for visualization')

    # ===============================
    # Get launch configurations
    # ===============================
    robot_ip = LaunchConfiguration('robot_ip')
    ur_type = LaunchConfiguration('ur_type')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    launch_rviz = LaunchConfiguration('launch_rviz')

    # ===============================
    # 1. UR Robot Driver
    # ===============================
    ur_driver = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [PathJoinSubstitution([FindPackageShare('ur_robot_driver_wrapper'), 'launch', 'bringup.launch.py'])]),
                                         launch_arguments={
                                             'robot_ip': robot_ip,
                                             'ur_type': ur_type,
                                             'use_fake_hardware': use_fake_hardware,
                                         }.items())

    # ===============================
    # 2. MoveIt
    # ===============================
    moveit = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [PathJoinSubstitution([FindPackageShare('ur_moveit_config_wrapper'), 'launch', 'ur_moveit.launch.py'])]),
                                      launch_arguments={
                                          'ur_type': ur_type,
                                          'launch_rviz': launch_rviz,
                                      }.items())

    # ===============================
    # 3. Motion Action Server
    # ===============================
    # Motion action server will auto-detect planning group
    motion_action_server = ExecuteProcess(cmd=['ros2', 'run', 'ur_motion', 'motion_action_server'], output='screen', shell=False)

    return LaunchDescription([
        # Launch arguments
        robot_ip_arg,
        ur_type_arg,
        use_fake_hardware_arg,
        launch_rviz_arg,

        # Launch nodes
        ur_driver,
        moveit,
        motion_action_server,
    ])
