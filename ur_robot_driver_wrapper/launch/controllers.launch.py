from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # Declare args / opaque function (런치 인자 선언 / 불투명 함수)
from launch.substitutions import LaunchConfiguration  # Runtime args (런치 설정값)
from launch_ros.actions import Node  # ROS2 node action (ROS2 노드 실행)


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
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")  # Fake HW flag: "true"/"false" (가상 HW 플래그)
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")  # Spawner timeout (스폰너 타임아웃)
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")  # Initial joint controller (초기 조인트 컨트롤러)
    activate_joint_controller = LaunchConfiguration(
        "activate_joint_controller")  # Activate joint controller flag (조인트 컨트롤러 활성화 플래그)

    # =========================================================
    # 2. Controller Spawner Factory Function (컨트롤러 스폰너 팩토리 함수)
    # =========================================================
    # - This function creates spawner nodes for controllers
    #   (이 함수는 컨트롤러를 위한 스폰너 노드를 생성)
    # - Reusable function to avoid code duplication
    #   (코드 중복을 피하기 위한 재사용 가능한 함수)
    # - Parameters:
    #   (파라미터:)
    #   * controllers: List of controller names to spawn (스폰할 컨트롤러 이름 리스트)
    #   * active: If True, controllers are activated immediately
    #            If False, controllers are spawned but inactive (--inactive flag)
    #     (True면 컨트롤러가 즉시 활성화됨
    #      False면 컨트롤러가 스폰되지만 비활성 상태, --inactive 플래그)
    # - Returns: Node that spawns the specified controllers
    #   (반환: 지정된 컨트롤러를 스폰하는 노드)
    # =========================================================
    def controller_spawner(controllers, active=True):
        # -----------------------------------------------------
        # 2.1 Inactive Flag Handling (비활성 플래그 처리)
        # -----------------------------------------------------
        # - If active=False, add --inactive flag to spawner arguments
        #   (active=False이면 스폰너 인자에 --inactive 플래그 추가)
        # - Inactive controllers are loaded but not running
        #   (비활성 컨트롤러는 로드되지만 실행되지 않음)
        # - Can be activated later via controller_manager service
        #   (나중에 controller_manager 서비스를 통해 활성화 가능)
        # - This allows pre-loading controllers for faster switching
        #   (컨트롤러 전환을 빠르게 하기 위해 사전 로드 허용)
        # -----------------------------------------------------
        inactive_flags = ["--inactive"] if not active else []

        # -----------------------------------------------------
        # 2.2 Spawner Node Creation (스폰너 노드 생성)
        # -----------------------------------------------------
        # - Creates a Node that runs controller_manager spawner
        #   (controller_manager spawner를 실행하는 노드 생성)
        # - Arguments:
        #   (인자:)
        #   * --controller-manager: Namespace of controller_manager
        #     (controller_manager의 네임스페이스)
        #   * --controller-manager-timeout: Timeout for spawn operation
        #     (스폰 작업의 타임아웃)
        #   * --inactive: (optional) Spawn controllers in inactive state
        #     (선택적) 컨트롤러를 비활성 상태로 스폰
        #   * controllers: List of controller names to spawn
        #     (스폰할 컨트롤러 이름 리스트)
        # -----------------------------------------------------
        return Node(
            package="controller_manager",  # Controller manager package (컨트롤러 매니저 패키지)
            executable="spawner",  # Spawner executable (스폰너 실행 파일)
            arguments=[
                "--controller-manager",  # Controller manager namespace option (컨트롤러 매니저 네임스페이스 옵션)
                "/controller_manager",  # Controller manager namespace (컨트롤러 매니저 네임스페이스)
                "--controller-manager-timeout",  # Timeout option (타임아웃 옵션)
                controller_spawner_timeout,  # Timeout value (타임아웃 값)
            ] + inactive_flags  # Add --inactive flag if active=False (active=False이면 --inactive 플래그 추가)
            + controllers,  # Controller names to spawn (스폰할 컨트롤러 이름들)
        )

    # =========================================================
    # 3. Define Controller Lists (컨트롤러 리스트 정의)
    # =========================================================
    # - Controllers are divided into active and inactive lists
    #   (컨트롤러는 활성 및 비활성 리스트로 구분됨)
    # - Active controllers: Always running, essential for operation
    #   (활성 컨트롤러: 항상 실행 중, 동작에 필수)
    # - Inactive controllers: Pre-loaded but not running, can be activated on demand
    #   (비활성 컨트롤러: 사전 로드되지만 실행되지 않음, 필요시 활성화 가능)
    # - This separation allows efficient resource management
    #   (이 구분은 효율적인 리소스 관리 허용)
    # =========================================================

    # -----------------------------------------------------
    # 3.1 Active Controllers (활성 컨트롤러) -> [상태 브로드캐스터 & 시스템 유지]
    # -----------------------------------------------------
    # - These controllers are spawned and activated immediately
    #   (이 컨트롤러들은 스폰되고 즉시 활성화됨)
    # - Essential for basic robot operation
    #   (기본 로봇 동작에 필수)
    # - Controllers:
    #   (컨트롤러:)
    #   * joint_state_broadcaster: Publishes joint states to /joint_states
    #     (/joint_states에 조인트 상태 발행)
    #   * io_and_status_controller: Handles I/O and robot status
    #     (I/O 및 로봇 상태 처리)
    #   * speed_scaling_state_broadcaster: Publishes speed scaling info
    #     (속도 스케일링 정보 발행)
    #   * force_torque_sensor_broadcaster: Publishes force/torque data
    #     (힘/토크 데이터 발행)
    #   * tcp_pose_broadcaster: Publishes TCP pose (real hardware only)
    #     (TCP 포즈 발행, 실제 하드웨어만)
    #   * ur_configuration_controller: Publishes robot configuration
    #     (로봇 설정 발행)
    # -----------------------------------------------------
    controllers_active = [
        "joint_state_broadcaster",  # Joint state publisher (조인트 상태 발행)
        "io_and_status_controller",  # I/O and status handler (I/O 및 상태 처리)
        "speed_scaling_state_broadcaster",  # Speed scaling info (속도 스케일링 정보)
        "force_torque_sensor_broadcaster",  # Force/torque sensor data (힘/토크 센서 데이터)
        "tcp_pose_broadcaster",  # TCP pose (real HW only) (TCP 포즈, 실제 HW만)
        "ur_configuration_controller",  # Robot configuration (로봇 설정)
    ]

    # -----------------------------------------------------
    # 3.2 Inactive Controllers (비활성 컨트롤러) -> [움직임 담당 컨트롤러]
    # -----------------------------------------------------
    # - These controllers are spawned but remain inactive
    #   (이 컨트롤러들은 스폰되지만 비활성 상태로 유지됨)
    # - Can be activated on demand via controller_manager service
    #   (controller_manager 서비스를 통해 필요시 활성화 가능)
    # - Allows multiple controllers to be available without resource conflict
    #   (리소스 충돌 없이 여러 컨트롤러를 사용 가능하게 함)
    # - Controllers:
    #   (컨트롤러:)
    #   * scaled_joint_trajectory_controller: Scaled trajectory control
    #     (스케일된 궤적 제어)
    #   * joint_trajectory_controller: Basic trajectory control
    #     (기본 궤적 제어)
    #   * forward_velocity_controller: Velocity control
    #     (속도 제어)
    #   * forward_position_controller: Position control
    #     (위치 제어)
    #   * forward_effort_controller: Effort/torque control
    #     (힘/토크 제어)
    #   * force_mode_controller: Force control mode
    #     (힘 제어 모드)
    #   * passthrough_trajectory_controller: Passthrough trajectory
    #     (패스스루 궤적)
    #   * freedrive_mode_controller: Freedrive (manual) mode
    #     (자유 구동 모드, 수동)
    #   * tool_contact_controller: Tool contact detection
    #     (툴 접촉 감지)
    # -----------------------------------------------------
    controllers_inactive = [
        "scaled_joint_trajectory_controller",  # Scaled trajectory control (스케일된 궤적 제어)
        "joint_trajectory_controller",  # Basic trajectory control (기본 궤적 제어)
        "forward_velocity_controller",  # Velocity control (속도 제어)
        "forward_position_controller",  # Position control (위치 제어)
        "forward_effort_controller",  # Effort/torque control (힘/토크 제어)
        "force_mode_controller",  # Force control mode (힘 제어 모드)
        "passthrough_trajectory_controller",  # Passthrough trajectory (패스스루 궤적)
        "freedrive_mode_controller",  # Freedrive mode (자유 구동 모드)
        "tool_contact_controller",  # Tool contact detection (툴 접촉 감지)
    ]

    # =========================================================
    # 4. Dynamic Controller Configuration (동적 컨트롤러 설정)
    # =========================================================
    # - Adjust controller lists based on runtime conditions
    #   (런타임 조건에 따라 컨트롤러 리스트 조정)
    # - This allows flexible controller activation
    #   (유연한 컨트롤러 활성화 허용)
    # =========================================================

    # -----------------------------------------------------
    # 4.1 Initial Joint Controller Activation (초기 조인트 컨트롤러 활성화)
    # -----------------------------------------------------
    # - If activate_joint_controller="true", move selected controller to active list
    #   (activate_joint_controller="true"이면 선택된 컨트롤러를 활성 리스트로 이동)
    # - This controller will be activated immediately on startup
    #   (이 컨트롤러는 시작 시 즉시 활성화됨)
    # - Default: scaled_joint_trajectory_controller
    #   (기본값: scaled_joint_trajectory_controller)
    # - Other options: joint_trajectory_controller, forward_velocity_controller, etc.
    #   (다른 옵션: joint_trajectory_controller, forward_velocity_controller 등)
    # -----------------------------------------------------
    if activate_joint_controller.perform(context) == "true":
        # Get the controller name from launch argument
        # (런치 인자에서 컨트롤러 이름 가져오기)
        selected_controller = initial_joint_controller.perform(context)
        # Move from inactive to active list
        # (비활성 리스트에서 활성 리스트로 이동)
        controllers_active.append(selected_controller)
        controllers_inactive.remove(selected_controller)

    # -----------------------------------------------------
    # 4.2 Fake Hardware Adjustment (가상 하드웨어 조정)
    # -----------------------------------------------------
    # - If use_fake_hardware="true", remove tcp_pose_broadcaster from active list
    #   (use_fake_hardware="true"이면 tcp_pose_broadcaster를 활성 리스트에서 제거)
    # - tcp_pose_broadcaster requires real hardware connection
    #   (tcp_pose_broadcaster는 실제 하드웨어 연결 필요)
    # - Fake hardware doesn't provide TCP pose data
    #   (가상 하드웨어는 TCP 포즈 데이터를 제공하지 않음)
    # - This prevents errors when using fake hardware
    #   (가상 하드웨어 사용 시 오류 방지)
    # -----------------------------------------------------
    if use_fake_hardware.perform(context) == "true":
        controllers_active.remove("tcp_pose_broadcaster")

    # =========================================================
    # 5. Create Controller Spawners (컨트롤러 스폰너 생성)
    # =========================================================
    # - Create spawner nodes for active and inactive controllers
    #   (활성 및 비활성 컨트롤러를 위한 스폰너 노드 생성)
    # - Two spawner nodes are created:
    #   (두 개의 스폰너 노드가 생성됨:)
    #   1. Active controllers spawner: Spawns and activates controllers
    #      (활성 컨트롤러 스폰너: 컨트롤러 스폰 및 활성화)
    #   2. Inactive controllers spawner: Spawns controllers in inactive state
    #      (비활성 컨트롤러 스폰너: 비활성 상태로 컨트롤러 스폰)
    # - Both spawners run in parallel
    #   (두 스폰너 모두 병렬 실행)
    # =========================================================
    controller_spawners = [
        controller_spawner(controllers_active),  # Active controllers spawner (활성 컨트롤러 스폰너)
        # controller_spawner(controllers_inactive, active=False),  # Inactive controllers spawner (비활성 컨트롤러 스폰너)
    ]

    # =========================================================
    # 6. Assemble Launch Description (런치 구성 조립)
    # =========================================================
    # - Order matters! (순서가 중요!)
    # - Launch sequence:
    #   (런치 순서:)
    #   1. Active controllers spawner: Spawns essential controllers
    #      (활성 컨트롤러 스폰너: 필수 컨트롤러 스폰)
    #   2. Inactive controllers spawner: Pre-loads optional controllers
    #      (비활성 컨트롤러 스폰너: 선택적 컨트롤러 사전 로드)
    # - Dependencies:
    #   (의존성:)
    #   * Both spawners require controller_manager to be running
    #     (두 스폰너 모두 controller_manager가 실행 중이어야 함)
    #   * controller_manager is provided by driver.launch.py
    #     (controller_manager는 driver.launch.py에서 제공됨)
    #   * This launch file must be included AFTER driver.launch.py
    #     (이 런치 파일은 driver.launch.py 이후에 포함되어야 함)
    # =========================================================
    nodes_to_start = controller_spawners

    return nodes_to_start


def generate_launch_description():
    # =========================================================
    # Declare Launch Arguments (런치 인자 선언)
    # =========================================================
    # - DeclareLaunchArgument defines CLI interface
    #   (DeclareLaunchArgument는 CLI 인터페이스를 정의)
    # - These arguments can be provided via:
    #   (이 인자들은 다음 방법으로 제공 가능:)
    #   * Command line: ros2 launch ... use_fake_hardware:=true
    #   * Launch file: launch_arguments={"use_fake_hardware": "true"}
    #   * Default values: If not provided, defaults are used
    #     (제공되지 않으면 기본값 사용)
    # - Arguments are collected in a list and passed to LaunchDescription
    #   (인자들은 리스트에 수집되어 LaunchDescription에 전달됨)
    # =========================================================
    declared_arguments = []

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
            "use_fake_hardware",  # Argument name (인자 이름)
            default_value="false",  # Default: real hardware (기본값: 실제 하드웨어)
            description="Hardware mode selection. "
            "true = Use fake hardware (removes tcp_pose_broadcaster). "
            "false = Use real hardware (includes tcp_pose_broadcaster).",
            # (하드웨어 모드 선택.
            #  true = 가상 하드웨어 사용 (tcp_pose_broadcaster 제거).
            #  false = 실제 하드웨어 사용 (tcp_pose_broadcaster 포함))
        ))

    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",  # Argument name (인자 이름)
            default_value="10",  # Default timeout in seconds (기본 타임아웃, 초 단위)
            description="Timeout for controller spawner operations. "
            "Time to wait for controller_manager to be ready. "
            "Increase if controllers fail to spawn.",
            # (컨트롤러 스폰너 작업의 타임아웃.
            #  controller_manager가 준비될 때까지 대기 시간.
            #  컨트롤러 스폰 실패 시 증가)
        ))

    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",  # Argument name (인자 이름)
            default_value="scaled_joint_trajectory_controller",  # Default controller (기본 컨트롤러)
            description="Initial joint controller to activate on startup.",
            # (시작 시 활성화할 초기 조인트 컨트롤러. 기본값: scaled_joint_trajectory_controller)
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "freedrive_mode_controller",
                "passthrough_trajectory_controller",
            ],
        ))

    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",  # Argument name (인자 이름)
            default_value="true",  # Default: activate (기본값: 활성화)
            description="Whether to activate the initial joint controller on startup. "
            "true = Activate selected controller immediately. "
            "false = Keep all trajectory controllers inactive.",
            # (시작 시 초기 조인트 컨트롤러를 활성화할지 여부.
            #  true = 선택된 컨트롤러를 즉시 활성화.
            #  false = 모든 궤적 컨트롤러를 비활성 상태로 유지)
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
