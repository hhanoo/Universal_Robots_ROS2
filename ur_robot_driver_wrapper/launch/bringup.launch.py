from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction  # Declare args / include sub-launch / opaque function (런치 인자 선언 / 하위 런치 포함 / 불투명 함수)
from launch.conditions import UnlessCondition, IfCondition  # Conditional execution (조건부 실행)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  # Runtime args + safe path join (런치 설정값 + 안전한 경로 결합)
from launch.launch_description_sources import PythonLaunchDescriptionSource  # Source type for .launch.py (파이썬 런치 소스)
from launch_ros.substitutions import FindPackageShare  # Find package share dir (패키지 share 경로 찾기)
from launch_ros.actions import Node  # ROS2 node action (ROS2 노드 실행)
from launch.substitutions import PythonExpression  # Python expression (파이썬 표현식)


def launch_setup(context, *args, **kwargs):
    # =========================================================
    # LaunchConfiguration handles
    # - These are raw user inputs (strings)
    #   (사용자 입력값, 문자열 상태)
    # =========================================================
    robot_ip = LaunchConfiguration("robot_ip")  # Robot/URSIM IP string (로봇/URSIM IP 문자열)
    ur_type = LaunchConfiguration("ur_type")  # UR model type (UR 모델 타입)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")  # Fake HW flag string: "true"/"false" (가상 HW 플래그)
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")  # Kinematics calibration file path (기구학 보정 파일 경로)
    joint_limit_params = LaunchConfiguration("joint_limit_params")  # Joint limit file path (조인트 제한 파일 경로)

    # =========================================================
    # headless_mode policy
    # - fake hardware  -> GUI allowed  -> headless = false
    # - real hardware  -> headless env -> headless = true
    #   (fake=개발, real=현장 기준 정책)
    # =========================================================
    headless_mode = PythonExpression(["'false' if '", use_fake_hardware, "' == 'true' else 'true'"])

    # =========================================================
    # RViz node
    # - Enabled ONLY for fake hardware
    # - Never launched for real robot / URSIM
    # =========================================================
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(use_fake_hardware),  # Run only for fake hardware (가상 HW 일 때만 실행)
        executable="rviz2",  # RViz2 executable (RViz2 실행 파일)
        name="rviz2",  # Node name (노드 이름)
        arguments=[
            "-d",  # Load config option (설정 파일 로드 옵션)
            PathJoinSubstitution([  # Safe join: <share>/rviz/ur_bringup.rviz (안전한 경로 결합)
                FindPackageShare("ur_description_wrapper"),  # /share/ur_description_wrapper (패키지 share 경로)
                "rviz",  # rviz folder (rviz 폴더)
                "view_robot.rviz",  # rviz config (rviz 설정 파일)
            ]),
        ],
    )

    # -----------------------------------------------------
    # Include: Driver launch (드라이버 런치 포함)
    # - Starts ur_ros2_control_node and robot_description generation
    #   (ur_ros2_control_node + robot_description 생성 포함)
    # - Passes critical arguments into driver.launch.py
    #   (driver.launch.py로 핵심 인자 전달)
    # -----------------------------------------------------
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([  # <share>/launch/driver.launch.py (설치 경로 기반 안전 결합)
                FindPackageShare("ur_robot_driver_wrapper"),  # package share (패키지 share)
                "launch",  # launch folder (launch 폴더)
                "driver.launch.py",  # target launch file (대상 런치 파일)
            ])),
        launch_arguments={  # Args passed to included launch (포함 런치에 전달할 인자)
            "robot_ip": robot_ip,  # IP used by UR driver (UR 드라이버가 사용할 IP)
            "ur_type": ur_type,  # Robot model type (로봇 모델 타입)
            "use_fake_hardware": use_fake_hardware,  # Fake hw mode (가상 HW 모드)
            "kinematics_params_file": kinematics_params_file,  # Kinematics file path (기구학 파일 경로)
            "joint_limit_params": joint_limit_params,  # Joint limits file path (조인트 제한 파일 경로)
            "headless_mode": headless_mode,  # Headless mode (헤드리스 모드)
        }.items(),
    )

    # -----------------------------------------------------
    # Include: Controllers launch (컨트롤러 런치 포함)
    # - Spawns joint_state_broadcaster, speed_scaling, trajectory controller
    #   (조인트 상태/스케일링/트래젝토리 컨트롤러 스폰)
    # - Assumes controller_manager is provided by driver (driver에서 controller_manager 제공)
    # -----------------------------------------------------
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([  # <share>/launch/controllers.launch.py (설치 경로 기반)
                FindPackageShare("ur_robot_driver_wrapper"),
                "launch",
                "controllers.launch.py",
            ])),)

    # -----------------------------------------------------
    # Include: Dashboard launch (대시보드 런치 포함)
    # - Runs only when NOT using fake hardware
    #   (fake hardware가 아닐 때만 실행)
    # - Dashboard provides power/brake/play services for real robot/URSIM
    #   (실로봇/URSIM에 대해 power/brake/play 서비스를 제공)
    # -----------------------------------------------------
    dashboard_launch = IncludeLaunchDescription(
        condition=UnlessCondition(use_fake_hardware),  # Skip when fake hw is true (fake hw면 스킵)
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution([  # <share>/launch/dashboard.launch.py (설치 경로 기반)
                FindPackageShare("ur_robot_driver_wrapper"),
                "launch",
                "dashboard.launch.py",
            ])),
        launch_arguments={
            "robot_ip": robot_ip,  # Dashboard needs robot IP (대시보드도 IP 필요)
        }.items(),
    )

    # =========================================================
    # LaunchDescription (런치 구성)
    # - Order matters: declare args first, then include sub-launches
    #   (순서 중요: 인자 선언 → 하위 런치 include)
    # =========================================================
    nodes_to_start = [
        driver_launch,
        controllers_launch,
        dashboard_launch,
        rviz_node,  # RViz node (RViz 노드 추가) - Placed at the end for readability; condition controls execution (가독성을 위해 마지막에 배치; 조건으로 실행 제어)
    ]

    return nodes_to_start


def generate_launch_description():
    # =========================================================
    # Declare launch arguments (런치 인자 선언)
    # - These define CLI interface and defaults
    #   (CLI 인자 인터페이스 및 기본값 정의)
    # =========================================================
    declared_arguments = []

    # -----------------------------------------------------
    # Required arguments (필수 인자)
    # -----------------------------------------------------
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",  # Required: no default here
            description="UR robot or URSIM IP address",  # (UR 로봇 또는 URSIM IP)
        ))

    # -----------------------------------------------------
    # Optional arguments with defaults (기본값이 있는 선택 인자)
    # -----------------------------------------------------
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",  # UR model selection (UR 모델 선택)
            default_value="ur10",  # Default model (기본 모델)
            description="UR robot model type",  # (UR 로봇 모델 타입)
        ))

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",  # Fake HW toggle (가상 하드웨어 토글)
            default_value="false",  # Default: real hardware (기본: 실하드웨어)
            description="true = fake hardware, false = real controller",  # (가상/실제 컨트롤러 선택)
        ))

    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params_file",  # Optional file path (선택 파일 경로)
            default_value="",  # Empty means "not provided" (빈 값이면 미지정)
            description="Kinematics calibration file",  # (기구학 보정 파일)
        ))

    declared_arguments.append(
        DeclareLaunchArgument(
            "joint_limit_params",  # Optional file path (선택 파일 경로)
            default_value="",  # Empty means "not provided" (빈 값이면 미지정)
            description="Joint limits configuration file",  # (조인트 제한 설정 파일)
        ))

    # =========================================================
    # LaunchDescription (런치 구성)
    # - Order matters: declare args first, then include sub-launches
    #   (순서 중요: 인자 선언 → 하위 런치 include)
    # =========================================================
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
