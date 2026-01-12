from launch import LaunchDescription
from launch.actions import (  # Declare args / include sub-launch / opaque function (런치 인자 선언 / 하위 런치 포함 / 불투명 함수)
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)  # Source type for .launch.py (파이썬 런치 소스)
from launch.substitutions import (  # Runtime args + safe path join (런치 설정값 + 안전한 경로 결합)
    LaunchConfiguration,
    PathJoinSubstitution,
)
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
    robot_ip = LaunchConfiguration(
        "robot_ip"
    )  # Robot/URSIM IP address (로봇/URSIM IP 주소)

    # =========================================================
    # 2. Include Sub-Launch File (하위 런치 파일 포함)
    # =========================================================
    # - IncludeLaunchDescription includes other launch files
    #   (IncludeLaunchDescription은 다른 런치 파일을 포함)
    # - PythonLaunchDescriptionSource specifies .launch.py files
    #   (PythonLaunchDescriptionSource는 .launch.py 파일을 지정)
    # - launch_arguments passes parameters to included launches
    #   (launch_arguments는 포함된 런치에 파라미터를 전달)
    # =========================================================

    # -----------------------------------------------------
    # Dashboard Client Launch (대시보드 클라이언트 런치)
    # -----------------------------------------------------
    # - Includes official UR robot driver's dashboard client
    #   (공식 UR 로봇 드라이버의 대시보드 클라이언트 포함)
    # - Provides robot control services (로봇 제어 서비스 제공)
    # - Services include:
    #   (서비스 포함:)
    #   * /dashboard_client/power_on: Power on the robot
    #     (로봇 전원 켜기)
    #   * /dashboard_client/brake_release: Release robot brakes
    #     (로봇 브레이크 해제)
    #   * /dashboard_client/play: Start robot program
    #     (로봇 프로그램 시작)
    #   * /dashboard_client/stop: Stop robot program
    #     (로봇 프로그램 중지)
    # - Required for real hardware / URSIM operation
    #   (실제 하드웨어/URSIM 동작에 필요)
    # - Not needed for fake hardware
    #   (가상 하드웨어에는 필요 없음)
    # -----------------------------------------------------
    dashboard_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare(
                        "ur_robot_driver"
                    ),  # Official UR driver package (공식 UR 드라이버 패키지)
                    "launch",  # Launch directory (런치 디렉토리)
                    "ur_dashboard_client.launch.py",  # Target launch file (대상 런치 파일)
                ]
            )
        ),
        launch_arguments={  # Arguments passed to ur_dashboard_client.launch.py (ur_dashboard_client.launch.py에 전달할 인자)
            "robot_ip": robot_ip,  # Robot IP for dashboard connection (대시보드 연결을 위한 로봇 IP)
        }.items(),
    )

    # =========================================================
    # 3. Assemble Launch Description (런치 구성 조립)
    # =========================================================
    # - This launch file is a wrapper that includes the official dashboard client
    #   (이 런치 파일은 공식 대시보드 클라이언트를 포함하는 래퍼)
    # - Provides a consistent interface for dashboard services
    #   (대시보드 서비스를 위한 일관된 인터페이스 제공)
    # - Simplifies integration with bringup.launch.py
    #   (bringup.launch.py와의 통합을 단순화)
    # =========================================================
    nodes_to_start = [
        dashboard_client_launch,  # Dashboard client launch (대시보드 클라이언트 런치)
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
            "Required for connecting to robot dashboard service. "
            "Example: 192.168.1.25 or 127.0.0.1 for URSIM",
            # (UR 로봇 또는 URSIM IP 주소.
            #  로봇 대시보드 서비스 연결에 필요.
            #  예: 192.168.1.25 또는 URSIM의 경우 127.0.0.1)
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
