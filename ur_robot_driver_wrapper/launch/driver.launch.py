from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # Declare args / opaque function (런치 인자 선언 / 불투명 함수)
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution  # Command execution / find executable / runtime args / safe path join (명령 실행 / 실행 파일 찾기 / 런치 설정값 / 안전한 경로 결합)
from launch_ros.actions import Node  # ROS2 node action (ROS2 노드 실행)
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue  # Parameter file / value wrapper (파라미터 파일 / 파라미터 값 래퍼)
from launch_ros.substitutions import FindPackageShare  # Find package share dir (패키지 share 경로 찾기)


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
    # - These values are typically provided by bringup.launch.py
    #   (이 값들은 일반적으로 bringup.launch.py에서 제공됨)
    # =========================================================
    robot_ip = LaunchConfiguration("robot_ip")  # Robot/URSIM IP address (로봇/URSIM IP 주소)
    ur_type = LaunchConfiguration("ur_type")  # UR robot model type (UR 로봇 모델 타입)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")  # Fake HW flag: "true"/"false" (가상 HW 플래그)
    kinematics_params = LaunchConfiguration("kinematics_params")  # Kinematics calibration file path (기구학 보정 파일 경로)
    headless_mode = LaunchConfiguration("headless_mode")  # Headless mode flag (헤드리스 모드 플래그)

    # =========================================================
    # 2. Construct File Paths (파일 경로 구성)
    # =========================================================
    # - PathJoinSubstitution safely joins path components
    #   (PathJoinSubstitution은 경로 구성 요소를 안전하게 결합)
    # - FindPackageShare locates package share directories
    #   (FindPackageShare는 패키지 share 디렉토리를 찾음)
    # - These paths are resolved at launch-time
    #   (이 경로들은 런치 시점에 해석됨)
    # =========================================================

    # -----------------------------------------------------
    # 2.1 UR Client Library Resources (UR 클라이언트 라이브러리 리소스)
    # -----------------------------------------------------
    # - External control URScript file (외부 제어 URScript 파일)
    # - RTDE input/output recipe files (RTDE 입출력 레시피 파일)
    # - Used by ur_ros2_control_node for robot communication
    #   (로봇 통신을 위해 ur_ros2_control_node에서 사용)
    # -----------------------------------------------------
    script_filename = PathJoinSubstitution([
        FindPackageShare("ur_client_library"),  # UR client library package (UR 클라이언트 라이브러리 패키지)
        "resources",  # Resources directory (리소스 디렉토리)
        "external_control.urscript",  # External control script (외부 제어 스크립트)
    ])
    input_recipe_filename = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"),  # UR robot driver package (UR 로봇 드라이버 패키지)
        "resources",  # Resources directory (리소스 디렉토리)
        "rtde_input_recipe.txt",  # RTDE input recipe (RTDE 입력 레시피)
    ])
    output_recipe_filename = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"),  # UR robot driver package (UR 로봇 드라이버 패키지)
        "resources",  # Resources directory (리소스 디렉토리)
        "rtde_output_recipe.txt",  # RTDE output recipe (RTDE 출력 레시피)
    ])

    # -----------------------------------------------------
    # 2.2 Controller Configuration File (컨트롤러 설정 파일)
    # -----------------------------------------------------
    # - YAML file containing controller definitions
    #   (컨트롤러 정의를 포함하는 YAML 파일)
    # - Defines available controllers and their parameters
    #   (사용 가능한 컨트롤러와 파라미터 정의)
    # -----------------------------------------------------
    controllers_config_file = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"),  # Official UR driver package (공식 UR 드라이버 패키지)
        "config",  # Config directory (설정 디렉토리)
        "ur_controllers.yaml",  # Controller configuration file (컨트롤러 설정 파일)
    ])

    # -----------------------------------------------------
    # 2.3 Update Rate Configuration File (업데이트 속도 설정 파일)
    # -----------------------------------------------------
    # - YAML file containing controller_manager update rate
    #   (controller_manager 업데이트 속도를 포함하는 YAML 파일)
    # - Defines the control loop frequency
    #   (제어 루프 주파수 정의)
    # - Required for proper controller operation
    #   (적절한 컨트롤러 동작에 필요)
    # - Uses official UR driver's update rate config
    #   (공식 UR 드라이버의 업데이트 속도 설정 사용)
    # -----------------------------------------------------
    update_rate_config_file = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"),  # Official UR driver package (공식 UR 드라이버 패키지)
        "config",  # Config directory (설정 디렉토리)
        ur_type.perform(context) + "_update_rate.yaml",  # Update rate file for robot type (로봇 타입별 업데이트 속도 파일)
    ])

    # =========================================================
    # 3. Generate Robot Description (로봇 설명 생성)
    # =========================================================
    # - robot_description is generated by executing xacro command
    #   (robot_description은 xacro 명령 실행으로 생성됨)
    # - Xacro expands URDF/Xacro files with parameters
    #   (Xacro는 파라미터와 함께 URDF/Xacro 파일을 확장)
    # - Command substitution executes shell command at launch-time
    #   (Command substitution은 런치 시점에 쉘 명령 실행)
    # - Result is a complete URDF XML string
    #   (결과는 완전한 URDF XML 문자열)
    # =========================================================
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),  # Find xacro executable (xacro 실행 파일 찾기)
        " ",  # Space separator (공백 구분자)
        PathJoinSubstitution([
            FindPackageShare("ur_description_wrapper"),  # Description wrapper package (설명 래퍼 패키지)
            "urdf",  # URDF directory (URDF 디렉토리)
            "ur.urdf.xacro",  # Main URDF xacro file (메인 URDF xacro 파일)
        ]),
        " robot_ip:=",
        robot_ip,  # Pass robot IP to xacro (xacro에 로봇 IP 전달)
        " name:=",
        "ur",  # Use "ur" to match official ur_moveit_config (공식 ur_moveit_config와 일치하도록 "ur" 사용)
        " ur_type:=",
        ur_type,  # Pass robot type (로봇 타입 전달)
        " use_fake_hardware:=",
        use_fake_hardware,  # Enable fake hardware if true (가상 하드웨어 사용)
        " kinematics_params:=",
        kinematics_params,  # Optional kinematics calibration (기구학 보정 파일)
        " headless_mode:=",
        headless_mode,  # Headless mode (헤드리스 모드)
        " script_filename:=",
        script_filename,  # External control script (외부 제어 스크립트)
        " input_recipe_filename:=",
        input_recipe_filename,  # RTDE input recipe (RTDE 입력 레시피)
        " output_recipe_filename:=",
        output_recipe_filename,  # RTDE output recipe (RTDE 출력 레시피)
    ])

    # -----------------------------------------------------
    # 3.1 Robot Description Parameter Dictionary (로봇 설명 파라미터 딕셔너리)
    # -----------------------------------------------------
    # - Wraps robot_description_content in ParameterValue
    #   (robot_description_content를 ParameterValue로 래핑)
    # - This dictionary is passed to nodes as parameters
    #   (이 딕셔너리는 노드에 파라미터로 전달됨)
    # - Used by ur_control_node and robot_state_publisher_node
    #   (ur_control_node와 robot_state_publisher_node에서 사용)
    # -----------------------------------------------------
    robot_description = {
        "robot_description":
            ParameterValue(
                value=robot_description_content,  # Xacro command result (Xacro 명령 결과)
                value_type=str,  # Value type: string (값 타입: 문자열)
            )
    }

    # =========================================================
    # 4. Create Nodes (노드 생성)
    # =========================================================

    # -----------------------------------------------------
    # 4.1 UR ros2_control Driver Node (UR ros2_control 드라이버 노드)
    # -----------------------------------------------------
    # - This is the CORE component that interfaces with robot hardware
    #   (로봇 하드웨어와 인터페이스하는 핵심 컴포넌트)
    # - Provides controller_manager for ROS2 control
    #   (ROS2 제어를 위한 controller_manager 제공)
    # - Handles robot communication via RTDE protocol
    #   (RTDE 프로토콜을 통한 로봇 통신 처리)
    # - Manages hardware interface (real or fake)
    #   (하드웨어 인터페이스 관리, 실제 또는 가상)
    # - Parameters:
    #   (파라미터:)
    #   * robot_description: Robot model (URDF) (로봇 모델, URDF)
    #   * controllers_config_file: Controller definitions (컨트롤러 정의)
    # -----------------------------------------------------
    ur_control_node = Node(
        package="ur_robot_driver",  # Package containing the driver executable (드라이버 실행 파일이 있는 패키지)
        executable="ur_ros2_control_node",  # Main UR ros2_control node (UR ros2_control 메인 노드)
        parameters=[  # Parameters passed to the node (노드에 전달할 파라미터)
            robot_description,  # Robot model description (URDF) (로봇 모델 설명, URDF)
            update_rate_config_file,  # Update rate configuration (업데이트 속도 설정)
            ParameterFile(controllers_config_file,
                          allow_substs=True),  # Controller configuration file with substitution support (대체 지원이 있는 컨트롤러 설정 파일)
            # ParameterFile allows variable substitution (e.g., $(var tf_prefix)) in YAML
            # (ParameterFile은 YAML에서 변수 대체를 허용, 예: $(var tf_prefix))
            # allow_substs=True enables substitution of launch variables
            # (allow_substs=True는 런치 변수의 대체를 활성화)
        ],
        output="screen",  # Print node output to terminal (터미널에 노드 출력 표시)
    )

    # -----------------------------------------------------
    # 4.2 Robot State Publisher Node (로봇 상태 발행 노드)
    # -----------------------------------------------------
    # - Publishes TF transforms using robot_description
    #   (robot_description을 사용하여 TF 변환 발행)
    # - Combines URDF static transforms with joint_states
    #   (URDF 정적 변환과 joint_states 결합)
    # - Required for visualization and motion planning
    #   (시각화 및 모션 플래닝에 필요)
    # - Listens to /joint_states topic
    #   (/joint_states 토픽 수신)
    # - Publishes to /tf and /tf_static topics
    #   (/tf 및 /tf_static 토픽 발행)
    # -----------------------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",  # TF publisher package (TF 발행 패키지)
        executable="robot_state_publisher",  # Executable name (실행 파일)
        name="robot_state_publisher",  # Node name (노드 이름)
        parameters=[robot_description],  # Uses same robot_description (동일 robot_description 사용)
        output="screen",  # Print logs to screen (로그를 화면에 출력)
    )

    # =========================================================
    # 5. Assemble Launch Description (런치 구성 조립)
    # =========================================================
    # - Order matters! (순서가 중요!)
    # - Launch sequence:
    #   (런치 순서:)
    #   1. ur_control_node: Core driver + controller_manager
    #      (핵심 드라이버 + controller_manager)
    #   2. robot_state_publisher_node: TF publishing
    #      (TF 발행)
    # - Dependencies:
    #   (의존성:)
    #   * robot_state_publisher_node depends on robot_description
    #     (robot_state_publisher_node는 robot_description에 의존)
    #   * Both nodes can start in parallel (no strict dependency)
    #     (두 노드는 병렬로 시작 가능, 엄격한 의존성 없음)
    #   * controller_manager from ur_control_node is needed by controllers
    #     (ur_control_node의 controller_manager는 컨트롤러에 필요)
    # =========================================================
    nodes_to_start = [
        ur_control_node,  # 1. Core driver (must be first) (핵심 드라이버, 첫 번째)
        robot_state_publisher_node,  # 2. TF publisher (TF 발행)
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
    # - These arguments are typically provided by bringup.launch.py
    #   (이 인자들은 일반적으로 bringup.launch.py에서 제공됨)
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
            # (UR 로봇 또는 URSIM IP 주소.
            #  로봇 하드웨어 또는 URSIM 시뮬레이터 연결에 필요.
            #  예: 192.168.1.25 또는 URSIM의 경우 127.0.0.1)
        ))

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
            "Affects joint limits, kinematics, and physical parameters. "
            "Choices: ur3, ur3e, ur5, ur5e, ur10, ur10e",
            # (UR 로봇 모델 타입.
            #  사용할 로봇 모델 설정을 결정.
            #  조인트 제한, 기구학, 물리 파라미터에 영향을 줌.
            #  선택: ur3, ur3e, ur5, ur5e, ur10, ur10e)
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
            ],
        ))

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
        ))

    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params",  # Argument name (인자 이름)
            default_value="",  # Empty string means "not provided" (빈 문자열은 "제공되지 않음" 의미)
            description="Path to kinematics calibration file. "
            "Optional. If provided, overrides default kinematics parameters. "
            "Used for robot-specific calibration. "
            "Empty string means use default kinematics from description package.",
            # (기구학 보정 파일 경로. 선택적.
            #  제공되면 기본 기구학 파라미터를 덮어씀.
            #  로봇별 보정에 사용됨.
            #  빈 문자열은 description 패키지의 기본 기구학 사용을 의미)
        ))

    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",  # Argument name (인자 이름)
            default_value="true",  # Default: headless mode (기본값: 헤드리스 모드)
            description="Enable headless mode for robot control. "
            "true = No GUI, suitable for production. "
            "false = GUI allowed, suitable for development.",
            # (로봇 제어를 위한 헤드리스 모드 활성화.
            #  true = GUI 없음, 운영 환경에 적합.
            #  false = GUI 허용, 개발 환경에 적합)
        ))

    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",  # Argument name (인자 이름)
            default_value="",  # Default: empty string (no prefix) (기본값: 빈 문자열, 접두사 없음)
            description="tf_prefix for joint names and TF frames. "
            "Useful for multi-robot setups. "
            "If changed, joint names in controllers' configuration must be updated. "
            "Empty string means no prefix (default).",
            # (조인트 이름 및 TF 프레임의 tf_prefix.
            #  다중 로봇 설정에 유용.
            #  변경 시 컨트롤러 설정의 조인트 이름도 업데이트해야 함.
            #  빈 문자열은 접두사 없음 의미, 기본값)
        ))

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
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
