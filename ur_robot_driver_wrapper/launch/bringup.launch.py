from launch import LaunchDescription
from launch.actions import (  # Declare args / include sub-launch / opaque function (런치 인자 선언 / 하위 런치 포함 / 불투명 함수)
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import (
    IfCondition,  # Conditional execution (조건부 실행)
    UnlessCondition,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)  # Source type for .launch.py (파이썬 런치 소스)
from launch.substitutions import PythonExpression  # Python expression (파이썬 표현식)
from launch.substitutions import (  # Runtime args + safe path join (런치 설정값 + 안전한 경로 결합)
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node  # ROS2 node action (ROS2 노드 실행)
from launch_ros.substitutions import (
    FindPackageShare,
)  # Find package share dir (패키지 share 경로 찾기)


def launch_setup(context, *args, **kwargs):
    # =========================================================
    # 1. Initialize Launch Arguments (런치 인자 초기화)
    # =========================================================
    # - LaunchConfiguration wraps user-provided arguments
    #   (LaunchConfiguration은 사용자가 제공한 인자를 래핑)
    # - These are substitution objects that resolve at launch-time
    #   (런치 시점에 해석되는 substitution 객체)
    # - Values come from CLI args or DeclareLaunchArgument defaults
    #   (값은 CLI 인자 또는 DeclareLaunchArgument 기본값에서 옴)
    # =========================================================
    # Common arguments (공통 인자)
    robot_ip = LaunchConfiguration(
        "robot_ip"
    )  # Robot/URSIM IP address (로봇/URSIM IP 주소)
    # Robot-related arguments (로봇 관련 인자)
    ur_type = LaunchConfiguration("ur_type")  # UR robot model type (UR 로봇 모델 타입)
    use_fake_hardware = LaunchConfiguration(
        "use_fake_hardware"
    )  # Fake HW flag: "true"/"false" (가상 HW 플래그)
    kinematics_params = LaunchConfiguration(
        "kinematics_params"
    )  # Kinematics calibration file path (기구학 보정 파일 경로)
    # Controller-related arguments (컨트롤러 관련 인자)
    controller_spawner_timeout = LaunchConfiguration(
        "controller_spawner_timeout"
    )  # Spawner timeout (스폰너 타임아웃)
    initial_joint_controller = LaunchConfiguration(
        "initial_joint_controller"
    )  # Initial joint controller (초기 조인트 컨트롤러)
    activate_joint_controller = LaunchConfiguration(
        "activate_joint_controller"
    )  # Activate joint controller flag (조인트 컨트롤러 활성화 플래그)

    # =========================================================
    # 2. Compute Derived Parameters (파생 파라미터 계산)
    # =========================================================
    # headless_mode policy (헤드리스 모드 정책)
    # - Development (fake hardware): GUI allowed -> headless = false
    #   (개발 환경: 가상 하드웨어 사용 시 GUI 허용 -> headless = false)
    # - Production (real hardware): Headless environment -> headless = true
    #   (운영 환경: 실제 하드웨어 사용 시 헤드리스 환경 -> headless = true)
    # - This ensures proper behavior in different deployment scenarios
    #   (다양한 배포 시나리오에서 적절한 동작 보장)
    # =========================================================
    headless_mode = PythonExpression(
        ["'false' if '", use_fake_hardware, "' == 'true' else 'true'"]
    )

    # =========================================================
    # 3. Construct File Paths (파일 경로 구성)
    # =========================================================
    # - PathJoinSubstitution safely joins path components
    #   (PathJoinSubstitution은 경로 구성 요소를 안전하게 결합)
    # - FindPackageShare locates package share directories
    #   (FindPackageShare는 패키지 share 디렉토리를 찾음)
    # - These paths are resolved at launch-time
    #   (이 경로들은 런치 시점에 해석됨)
    # =========================================================
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(
                "ur_description_wrapper"
            ),  # Package share directory (패키지 share 디렉토리)
            "rviz",  # Subdirectory (하위 디렉토리)
            "view_robot.rviz",  # RViz configuration file (RViz 설정 파일)
        ]
    )

    # =========================================================
    # 4. Create Nodes (노드 생성)
    # =========================================================
    # RViz visualization node (RViz 시각화 노드)
    # - Enabled ONLY for fake hardware (가상 하드웨어일 때만 활성화)
    # - Never launched for real robot / URSIM (실제 로봇/URSIM에서는 실행 안 함)
    # - Provides visual feedback during development/testing
    #   (개발/테스트 중 시각적 피드백 제공)
    # - Condition ensures it only runs when use_fake_hardware="true"
    #   (조건부 실행으로 use_fake_hardware="true"일 때만 실행)
    # =========================================================
    rviz_node = Node(
        package="rviz2",  # RViz2 package (RViz2 패키지)
        executable="rviz2",  # RViz2 executable (RViz2 실행 파일)
        name="rviz2",  # Node name (노드 이름)
        condition=IfCondition(use_fake_hardware),  # Conditional execution (조건부 실행)
        arguments=[
            "-d",  # Load configuration file option (설정 파일 로드 옵션)
            rviz_config_file,  # RViz configuration file path (RViz 설정 파일 경로)
        ],
        output="log",  # Output to log file (로그 파일로 출력)
    )

    # =========================================================
    # 5. Include Sub-Launch Files (하위 런치 파일 포함)
    # =========================================================
    # - IncludeLaunchDescription includes other launch files
    #   (IncludeLaunchDescription은 다른 런치 파일을 포함)
    # - PythonLaunchDescriptionSource specifies .launch.py files
    #   (PythonLaunchDescriptionSource는 .launch.py 파일을 지정)
    # - launch_arguments passes parameters to included launches
    #   (launch_arguments는 포함된 런치에 파라미터를 전달)
    # =========================================================

    # -----------------------------------------------------
    # 5.1 Driver Launch (드라이버 런치)
    # -----------------------------------------------------
    # - Starts ur_ros2_control_node (ur_ros2_control_node 시작)
    # - Generates robot_description via xacro (xacro를 통해 robot_description 생성)
    # - Provides controller_manager (controller_manager 제공)
    # - This is the CORE component that interfaces with hardware
    #   (하드웨어와 인터페이스하는 핵심 컴포넌트)
    # - Must be launched FIRST (가장 먼저 실행되어야 함)
    # - Other components depend on controller_manager
    #   (다른 컴포넌트들이 controller_manager에 의존)
    # -----------------------------------------------------
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare(
                        "ur_robot_driver_wrapper"
                    ),  # Package name (패키지 이름)
                    "launch",  # Launch directory (런치 디렉토리)
                    "driver.launch.py",  # Target launch file (대상 런치 파일)
                ]
            )
        ),
        launch_arguments={  # Arguments passed to driver.launch.py (driver.launch.py에 전달할 인자)
            "robot_ip": robot_ip,  # Robot IP for connection (연결을 위한 로봇 IP)
            "ur_type": ur_type,  # Robot model type (로봇 모델 타입)
            "use_fake_hardware": use_fake_hardware,  # Hardware mode (하드웨어 모드)
            "kinematics_params": kinematics_params,  # Kinematics calibration (기구학 보정)
            "headless_mode": headless_mode,  # Headless mode flag (헤드리스 모드 플래그)
        }.items(),
    )

    # -----------------------------------------------------
    # 5.2 Controllers Launch (컨트롤러 런치)
    # -----------------------------------------------------
    # - Spawns ROS2 controllers (ROS2 컨트롤러 스폰)
    # - Manages controller lifecycle (컨트롤러 생명주기 관리)
    # - Controllers include:
    #   (컨트롤러 포함:)
    #   * Active controllers (always running):
    #     (활성 컨트롤러, 항상 실행 중:)
    #     - joint_state_broadcaster: Publishes joint states (조인트 상태 발행)
    #     - io_and_status_controller: I/O and status handling (I/O 및 상태 처리)
    #     - speed_scaling_state_broadcaster: Speed scaling info (속도 스케일링 정보)
    #     - force_torque_sensor_broadcaster: Force/torque data (힘/토크 데이터)
    #     - tcp_pose_broadcaster: TCP pose (real HW only) (TCP 포즈, 실제 HW만)
    #     - ur_configuration_controller: Robot configuration (로봇 설정)
    #   * Inactive controllers (pre-loaded, can be activated on demand):
    #     (비활성 컨트롤러, 사전 로드, 필요시 활성화 가능:)
    #     - scaled_joint_trajectory_controller: Scaled trajectory control (스케일된 궤적 제어)
    #     - joint_trajectory_controller: Basic trajectory control (기본 궤적 제어)
    #     - forward_velocity_controller: Velocity control (속도 제어)
    #     - forward_position_controller: Position control (위치 제어)
    #     - freedrive_mode_controller: Freedrive mode (자유 구동 모드)
    #     - Other specialized controllers (기타 특수 컨트롤러)
    # - Requires controller_manager from driver_launch
    #   (driver_launch의 controller_manager 필요)
    # - Must be launched AFTER driver_launch
    #   (driver_launch 이후에 실행되어야 함)
    # - Arguments passed:
    #   (전달되는 인자:)
    #   * use_fake_hardware: Adjusts controller list (tcp_pose_broadcaster removal)
    #     (컨트롤러 리스트 조정, tcp_pose_broadcaster 제거)
    #   * controller_spawner_timeout: Timeout for spawn operations
    #     (스폰 작업의 타임아웃)
    #   * initial_joint_controller: Controller to activate on startup
    #     (시작 시 활성화할 컨트롤러)
    #   * activate_joint_controller: Whether to activate initial controller
    #     (초기 컨트롤러 활성화 여부)
    # -----------------------------------------------------
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare(
                        "ur_robot_driver_wrapper"
                    ),  # Package name (패키지 이름)
                    "launch",  # Launch directory (런치 디렉토리)
                    "controllers.launch.py",  # Target launch file (대상 런치 파일)
                ]
            )
        ),
        launch_arguments={  # Arguments passed to controllers.launch.py (controllers.launch.py에 전달할 인자)
            "use_fake_hardware": use_fake_hardware,  # Hardware mode (affects tcp_pose_broadcaster) (하드웨어 모드, tcp_pose_broadcaster에 영향)
            "controller_spawner_timeout": controller_spawner_timeout,  # Spawner timeout (스폰너 타임아웃)
            "initial_joint_controller": initial_joint_controller,  # Initial controller to activate (활성화할 초기 컨트롤러)
            "activate_joint_controller": activate_joint_controller,  # Activate controller flag (컨트롤러 활성화 플래그)
        }.items(),
    )

    # -----------------------------------------------------
    # 5.3 Dashboard Launch (대시보드 런치)
    # -----------------------------------------------------
    # - Provides robot control services (로봇 제어 서비스 제공)
    # - Services include:
    #   (서비스 포함:)
    #   * power_on: Power on the robot (로봇 전원 켜기)
    #   * brake_release: Release brakes (브레이크 해제)
    #   * play: Start robot program (로봇 프로그램 시작)
    # - Only needed for REAL hardware / URSIM
    #   (실제 하드웨어/URSIM에만 필요)
    # - Condition: Skip when use_fake_hardware="true"
    #   (조건: use_fake_hardware="true"일 때 스킵)
    # - Fake hardware doesn't need dashboard services
    #   (가상 하드웨어는 대시보드 서비스가 필요 없음)
    # -----------------------------------------------------
    dashboard_launch = IncludeLaunchDescription(
        condition=UnlessCondition(
            use_fake_hardware
        ),  # Conditional execution (조건부 실행)
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare(
                        "ur_robot_driver_wrapper"
                    ),  # Package name (패키지 이름)
                    "launch",  # Launch directory (런치 디렉토리)
                    "dashboard.launch.py",  # Target launch file (대상 런치 파일)
                ]
            )
        ),
        launch_arguments={  # Arguments passed to dashboard.launch.py (dashboard.launch.py에 전달할 인자)
            "robot_ip": robot_ip,  # Robot IP for dashboard connection (대시보드 연결을 위한 로봇 IP)
        }.items(),
    )

    # =========================================================
    # 6. Assemble Launch Description (런치 구성 조립)
    # =========================================================
    # - Order matters! (순서가 중요!)
    # - Launch sequence:
    #   (런치 순서:)
    #   1. driver_launch: Core driver + controller_manager
    #      (핵심 드라이버 + controller_manager)
    #   2. controllers_launch: Spawn controllers (requires controller_manager)
    #      (컨트롤러 스폰, controller_manager 필요)
    #   3. dashboard_launch: Robot control services (optional, real HW only)
    #      (로봇 제어 서비스, 선택적, 실제 HW만)
    #   4. rviz_node: Visualization (optional, fake HW only)
    #      (시각화, 선택적, 가상 HW만)
    # - Dependencies:
    #   (의존성:)
    #   * controllers_launch depends on driver_launch (controller_manager)
    #     (controllers_launch는 driver_launch에 의존, controller_manager)
    #   * dashboard_launch independent but should start early
    #     (dashboard_launch는 독립적이지만 일찍 시작해야 함)
    #   * rviz_node independent, placed last for readability
    #     (rviz_node는 독립적, 가독성을 위해 마지막에 배치)
    # =========================================================
    nodes_to_start = [
        driver_launch,  # 1. Core driver (must be first) (핵심 드라이버, 첫 번째)
        controllers_launch,  # 2. Controllers (after driver) (컨트롤러, 드라이버 이후)
        dashboard_launch,  # 3. Dashboard (real HW only) (대시보드, 실제 HW만)
        rviz_node,  # 4. Visualization (fake HW only) (시각화, 가상 HW만)
    ]

    return nodes_to_start


def generate_launch_description():
    # =========================================================
    # Declare Launch Arguments (런치 인자 선언)
    # =========================================================
    # - DeclareLaunchArgument defines CLI interface
    #   (DeclareLaunchArgument는 CLI 인터페이스를 정의)
    # - These arguments can be provided via:
    #   (이 인자들은 다음 방법으로 제공 가능:)
    #   * Command line: ros2 launch ... robot_ip:=192.168.1.25
    #   * Launch file: launch_arguments={"robot_ip": "192.168.1.25"}
    #   * Default values: If not provided, defaults are used
    #     (제공되지 않으면 기본값 사용)
    # - Arguments are collected in a list and passed to LaunchDescription
    #   (인자들은 리스트에 수집되어 LaunchDescription에 전달됨)
    # =========================================================
    declared_arguments = []

    # -----------------------------------------------------
    # Required Arguments (필수 인자)
    # -----------------------------------------------------
    # - No default value means argument is required
    #   (기본값이 없으면 인자가 필수)
    # - User must provide this value
    #   (사용자가 이 값을 제공해야 함)
    # -----------------------------------------------------
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",  # Argument name (인자 이름)
            description="UR robot or URSIM IP address. "
            "Required for connecting to robot hardware or URSIM simulator. "
            "Example: 192.168.1.25 or 127.0.0.1 for URSIM",
            # (UR 로봇 또는 URSIM IP 주소. 로봇 하드웨어 또는 URSIM 시뮬레이터 연결에 필요.
            #  예: 192.168.1.25 또는 URSIM의 경우 127.0.0.1)
        )
    )

    # -----------------------------------------------------
    # Optional Arguments with Defaults (기본값이 있는 선택 인자)
    # -----------------------------------------------------
    # - Default values allow arguments to be optional
    #   (기본값이 있으면 인자가 선택적)
    # - User can override defaults via CLI
    #   (사용자가 CLI를 통해 기본값을 덮어쓸 수 있음)
    # -----------------------------------------------------

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",  # Argument name (인자 이름)
            default_value="ur10e",  # Default value (기본값)
            description="UR robot model type. "
            "Determines which robot model configuration to use. "
            "Affects joint limits, kinematics, and physical parameters.",
            # (UR 로봇 모델 타입. 사용할 로봇 모델 설정을 결정.
            #  조인트 제한, 기구학, 물리 파라미터에 영향을 줌)
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",  # Argument name (인자 이름)
            default_value="false",  # Default: real hardware (기본값: 실제 하드웨어)
            description="Hardware mode selection. "
            "true = Use fake hardware (for development/testing without robot). "
            "false = Use real hardware or URSIM (requires robot connection).",
            # (하드웨어 모드 선택.
            #  true = 가상 하드웨어 사용 (로봇 없이 개발/테스트).
            #  false = 실제 하드웨어 또는 URSIM 사용 (로봇 연결 필요))
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params",  # Argument name (인자 이름)
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare(
                        "ur_description_wrapper"
                    ),  # Package share directory (패키지 share 디렉토리)
                    "config",  # Subdirectory (하위 디렉토리)
                    "calibration_kinematics.yaml",  # Kinematics calibration file (기구학 보정 파일)
                ]
            ),  # Empty string means "not provided" (빈 문자열은 "제공되지 않음" 의미)
            description="Path to kinematics calibration file. "
            "Optional. If provided, overrides default kinematics parameters. "
            "Used for robot-specific calibration. "
            "Empty string means use default kinematics from description package.",
            # (기구학 보정 파일 경로. 선택적.
            #  제공되면 기본 기구학 파라미터를 덮어씀.
            #  로봇별 보정에 사용됨.
            #  빈 문자열은 description 패키지의 기본 기구학 사용을 의미)
        )
    )

    # -----------------------------------------------------
    # Controller Configuration Arguments (컨트롤러 설정 인자)
    # -----------------------------------------------------
    # - These arguments control controller spawning and activation
    #   (이 인자들은 컨트롤러 스폰 및 활성화를 제어)
    # - Passed to controllers.launch.py
    #   (controllers.launch.py에 전달됨)
    # -----------------------------------------------------
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",  # Argument name (인자 이름)
            default_value="10",  # Default timeout in seconds (기본 타임아웃, 초 단위)
            description="Timeout for controller spawner operations. "
            "Time to wait for controller_manager to be ready before spawning controllers. "
            "Increase this value if controllers fail to spawn (e.g., slow hardware initialization). "
            "Unit: seconds.",
            # (컨트롤러 스폰너 작업의 타임아웃.
            #  컨트롤러를 스폰하기 전 controller_manager가 준비될 때까지 대기 시간.
            #  컨트롤러 스폰 실패 시 이 값을 증가 (예: 느린 하드웨어 초기화).
            #  단위: 초)
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",  # Argument name (인자 이름)
            default_value="scaled_joint_trajectory_controller",  # Default controller (기본 컨트롤러)
            description="Initial joint controller to activate on startup. "
            "This controller will be moved from inactive to active list and activated immediately. "
            "This is the controller that will be used for robot motion control. "
            "Choices: scaled_joint_trajectory_controller (recommended for MoveIt), "
            "joint_trajectory_controller, forward_velocity_controller, "
            "forward_position_controller, freedrive_mode_controller, "
            "passthrough_trajectory_controller. "
            "Only effective if activate_joint_controller='true'.",
            # (시작 시 활성화할 초기 조인트 컨트롤러.
            #  이 컨트롤러는 비활성 리스트에서 활성 리스트로 이동되어 즉시 활성화됨.
            #  로봇 동작 제어에 사용될 컨트롤러.
            #  선택: scaled_joint_trajectory_controller (MoveIt에 권장),
            #  joint_trajectory_controller, forward_velocity_controller,
            #  forward_position_controller, freedrive_mode_controller,
            #  passthrough_trajectory_controller.
            #  activate_joint_controller='true'일 때만 효과적)
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "freedrive_mode_controller",
                "passthrough_trajectory_controller",
            ],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",  # Argument name (인자 이름)
            default_value="true",  # Default: activate (기본값: 활성화)
            description="Whether to activate the initial joint controller on startup. "
            "true = Activate the controller specified by initial_joint_controller immediately. "
            "This allows immediate robot control after launch. "
            "Recommended for normal operation. "
            "false = Keep all trajectory controllers inactive. "
            "Controllers are loaded but not running. "
            "Use this if you want to manually activate controllers later via "
            "ros2 control switch_controllers command. "
            "Useful for testing or custom controller switching logic.",
            # (시작 시 초기 조인트 컨트롤러를 활성화할지 여부.
            #  true = initial_joint_controller로 지정된 컨트롤러를 즉시 활성화.
            #  런치 후 즉시 로봇 제어 가능.
            #  일반 운영에 권장.
            #  false = 모든 궤적 컨트롤러를 비활성 상태로 유지.
            #  컨트롤러는 로드되지만 실행되지 않음.
            #  ros2 control switch_controllers 명령으로 나중에 수동 활성화하려는 경우 사용.
            #  테스트 또는 커스텀 컨트롤러 전환 로직에 유용)
        )
    )

    # =========================================================
    # LaunchDescription Assembly (런치 구성 조립)
    # =========================================================
    # - LaunchDescription takes a list of actions
    #   (LaunchDescription은 액션 리스트를 받음)
    # - Order: DeclareLaunchArgument first, then OpaqueFunction
    #   (순서: DeclareLaunchArgument 먼저, 그 다음 OpaqueFunction)
    # - OpaqueFunction delays execution until launch-time
    #   (OpaqueFunction은 런치 시점까지 실행을 지연)
    # - This allows runtime argument resolution
    #   (런타임 인자 해석을 허용)
    # - launch_setup function receives context with resolved arguments
    #   (launch_setup 함수는 해석된 인자가 있는 context를 받음)
    # =========================================================
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
