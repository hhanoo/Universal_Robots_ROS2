from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    ThisLaunchFileDir,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    ThisLaunchFileDir(),
                    "common.launch.py",
                ]
            )
        )
    )

    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_robot_driver_wrapper"),
                    "launch",
                    "bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "ur_type": LaunchConfiguration("ur_type"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "kinematics_params": LaunchConfiguration("kinematics_params"),
        }.items(),
    )

    return LaunchDescription(
        [
            common,
            ur_driver,
        ]
    )
