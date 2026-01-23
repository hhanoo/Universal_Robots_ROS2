from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    ThisLaunchFileDir,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ===============================
    # 1. Common
    # ===============================
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

    # ===============================
    # 2. UR Driver
    # ===============================
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

    # ===============================
    # 3. UR MoveIt
    # ===============================
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

    # ===============================
    # 4. Motion Action Server
    # ===============================
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ur_description_wrapper"),
                            "urdf",
                            "ur.urdf.xacro",
                        ]
                    ),
                    " robot_ip:=",
                    LaunchConfiguration("robot_ip"),
                    " name:=ur",
                    " ur_type:=",
                    LaunchConfiguration("ur_type"),
                    " use_fake_hardware:=",
                    LaunchConfiguration("use_fake_hardware"),
                    " kinematics_params:=",
                    LaunchConfiguration("kinematics_params"),
                    " tf_prefix:=",
                    LaunchConfiguration("tf_prefix"),
                ]
            ),
            value_type=str,
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ur_moveit_config_wrapper"),
                            "srdf",
                            "ur.srdf.xacro",
                        ]
                    ),
                    " tf_prefix:=",
                    LaunchConfiguration("tf_prefix"),
                ]
            ),
            value_type=str,
        )
    }

    motion_action_server = Node(
        package="ur_motion",
        executable="motion_action_server",
        namespace=LaunchConfiguration("namespace"),
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        parameters=[
            # robot_description,
            # robot_description_semantic,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            common,
            ur_driver,
            ur_moveit,
            motion_action_server,
        ]
    )
