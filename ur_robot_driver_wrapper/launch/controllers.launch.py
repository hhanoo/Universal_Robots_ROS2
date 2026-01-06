from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    speed_scaling_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "speed_scaling_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    trajectory_spawner = TimerAction(
        period=0.5,  # wait for hardware activation
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "scaled_joint_trajectory_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        joint_state_spawner,
        speed_scaling_spawner,
        trajectory_spawner,
    ])
