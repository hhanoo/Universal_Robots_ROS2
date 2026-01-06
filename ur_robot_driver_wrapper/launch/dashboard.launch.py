from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_ip = LaunchConfiguration("robot_ip")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            description="UR robot or URSIM IP address",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ur_robot_driver"),
                    "launch",
                    "ur_dashboard_client.launch.py",
                ])),
            launch_arguments={
                "robot_ip": robot_ip,
            }.items(),
        ),
    ])
