from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="127.0.0.1",
            description="IP address of the UR robot (use 127.0.0.1 for simulation)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="Type of UR robot (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware (simulation mode)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_description_wrapper"),
                    "config",
                    "calibration_kinematics.yaml",
                ]
            ),
            description="Path to kinematics calibration file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS namespace for this robot instance. Useful for multi-robot setups.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="TF prefix for joint names and TF frames. Useful for multi-robot setups.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_moveit_rviz",
            default_value="true",
            description="Launch MoveIt RViz for visualization",
        )
    )

    return LaunchDescription(declared_arguments)
