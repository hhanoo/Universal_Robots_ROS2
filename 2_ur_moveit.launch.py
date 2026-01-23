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

    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_moveit_config_wrapper"),
                    "launch",
                    "ur_moveit.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "launch_rviz": LaunchConfiguration("launch_moveit_rviz"),
            # "namespace": LaunchConfiguration("namespace"),
            # "prefix": LaunchConfiguration("tf_prefix"),
        }.items(),
    )

    return LaunchDescription(
        [
            common,
            ur_moveit,
        ]
    )
