from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # =========================================================
    # Launch arguments (런치 인자 정의)
    # - These values are provided from bringup.launch.py
    #   (bringup.launch.py에서 전달받는 런치 인자들)
    # =========================================================
    robot_ip = LaunchConfiguration("robot_ip")  # Robot or URSIM IP (로봇 또는 URSIM IP)
    ur_type = LaunchConfiguration("ur_type")  # UR robot model type (UR 로봇 모델 타입)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")  # Fake or real HW (가상/실제 하드웨어 선택)
    kinematics_params = LaunchConfiguration("kinematics_params")  # Kinematics calibration (기구학 보정 파일)
    headless_mode = LaunchConfiguration("headless_mode")  # Headless mode flag (헤드리스 모드 플래그)

    # =========================================================
    # UR client library resources
    # - Used only by ur_ros2_control_node
    #   (ur_ros2_control_node 내부에서만 사용됨)
    # =========================================================
    script_filename = PathJoinSubstitution([FindPackageShare("ur_client_library"), "resources", "external_control.urscript"])
    input_recipe_filename = PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"])
    output_recipe_filename = PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"])

    # =========================================================
    # robot_description (URDF generation)
    # - Xacro is expanded at launch-time
    # - Fake/Real hardware, headless mode are resolved here
    #   (런치 시점에 Xacro 실행, fake/real/headless 결정)
    # =========================================================
    robot_description = {
        "robot_description":
            ParameterValue(
                Command([
                    "xacro ",  # Run xacro command (xacro 실행)
                    PathJoinSubstitution([
                        FindPackageShare(
                            "ur_description_wrapper"),  # Find ur_description_wrapper package (ur_description_wrapper 패키지 경로)
                        "urdf",
                        "ur.urdf.xacro"  # UR main xacro file (UR 메인 xacro 파일)
                    ]),
                    " robot_ip:=",
                    robot_ip,  # Pass robot IP to xacro (로봇 IP 전달)
                    " name:=",
                    ur_type,  # Pass robot name/type (로봇 타입 전달)
                    " use_fake_hardware:=",
                    use_fake_hardware,  # Enable fake hardware if true (가상 하드웨어 사용)
                    " kinematics_params:=",
                    kinematics_params,  # Optional kinematics calibration (기구학 보정 파일)
                    " headless_mode:=",
                    headless_mode,  # Headless mode (헤드리스 모드)
                    " script_filename:=",
                    script_filename,
                    " input_recipe_filename:=",
                    input_recipe_filename,
                    " output_recipe_filename:=",
                    output_recipe_filename,
                ]),
                value_type=str)
    }

    # =========================================================
    # UR ros2_control driver node
    # - This node owns controller_manager
    # - robot_state_publisher is implicitly handled by UR driver
    #   (controller_manager 소유 / UR 드라이버 내부에서 TF 처리)
    # =========================================================
    ur_control_node = Node(
        package="ur_robot_driver",  # Package containing the driver executable (드라이버 실행 파일이 있는 패키지)
        executable="ur_ros2_control_node",  # Main UR ros2_control node (UR ros2_control 메인 노드)
        parameters=[  # Parameters passed to the node (노드에 전달할 파라미터)
            robot_description,  # Robot model description (로봇 모델 정보)
            PathJoinSubstitution([FindPackageShare("ur_robot_driver_wrapper"), "config",
                                  "ur_controllers.yaml"]),  # Controller configuration file (컨트롤러 설정 파일)
        ],
        output="screen",  # Print node output to terminal (출력을 터미널로 표시)
    )

    # =========================================================
    # robot_state_publisher node
    # - Publishes TF using robot_description and joint_states
    #   (URDF + joint_states 기반으로 TF 발행)
    # =========================================================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",  # TF publisher package (TF 발행 패키지)
        executable="robot_state_publisher",  # Executable name (실행 파일)
        name="robot_state_publisher",  # Node name (노드 이름)
        parameters=[robot_description],  # Uses same robot_description (동일 URDF 사용)
        output="screen",  # Print logs to screen (로그 출력)
    )

    # =========================================================
    # Launch description return (런치 구성 반환)
    # - Order matters: declare args first, then nodes
    #   (순서 중요: 인자 선언 → 노드 순서)
    # =========================================================
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            description="UR robot or URSIM IP address",  # (UR 로봇 또는 URSIM IP)
        ),
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="UR robot model type",  # (UR 로봇 모델 타입)
            choices=[
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
            ],
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="true = fake hardware, false = real controller",  # (가상/실제 컨트롤러 선택)
        ),
        DeclareLaunchArgument(
            "kinematics_params",
            default_value="",
            description="Kinematics calibration file",  # (기구학 보정 파일)
        ),
        DeclareLaunchArgument(
            "joint_limit_params",
            default_value="",
            description="Joint limits configuration file",  # (조인트 제한 설정 파일)
        ),
        DeclareLaunchArgument(
            "headless_mode",
            default_value="true",
            description="Enable headless mode for robot control",  # (헤드리스 모드 사용 여부)
        ),

        # ---- Nodes to launch (실행할 노드) ----
        ur_control_node,
        robot_state_publisher_node,
    ])
