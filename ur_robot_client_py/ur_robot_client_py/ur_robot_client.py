"""
UR Control Client Library (Python)
===================================
Python client library for controlling UR robots

Features:
- MoveJ/MoveL motion control
- Speed slider control
- Digital I/O control
- Robot state monitoring
- TCP pose tracking via TF
- Program (external control) state monitoring
- Program watchdog: auto-regains control after e-stop/Local mode
"""

import asyncio

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_dashboard_msgs.srv import IsInRemoteControl
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO, SetSpeedSliderFraction

from ur_motion.action import MoveJ, MoveL


class URRobotClient:
    """
    UR Robot Controller (non-Node class).

    This class provides a high-level interface for controlling UR robots.
    All motion commands use async/await for clean sequential programming.

    Example:
        node = Node('my_node')
        robot = URRobotClient(node)

        # Sequential motion control
        # 1. MoveJ to initial position
        success, msg = await robot.move_j([0, -1.57, 1.57, -1.57, -1.57, 0], 0.5)
        await robot.wait(1.0)

        # 2. MoveL to target position
        success, msg = await robot.move_l(tmatrix, 0.3)
        await robot.wait(1.0)
    """

    # ========================================================
    # Initialization
    # ========================================================
    def __init__(self, node: Node):
        """
        Initialize UR Robot Controller.

        Args:
            node: ROS2 node instance for creating subscriptions, clients, etc.
        """
        self.node = node

        # Action clients
        self.movej_client = ActionClient(self.node, MoveJ, "/move_j")
        self.movel_client = ActionClient(self.node, MoveL, "/move_l")
        self.current_movej_goal = None  # current MoveJ goal handle (현재 MoveJ)
        self.current_movel_goal = None  # current MoveL goal handle (현재 MoveL)

        # Service clients
        self.speed_slider_client = self.node.create_client(
            SetSpeedSliderFraction, "/io_and_status_controller/set_speed_slider"
        )
        self.set_io_client = self.node.create_client(
            SetIO, "/io_and_status_controller/set_io"
        )

        # Subscribers
        self.joint_state_sub = self.node.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )
        self.speed_scaling_sub = self.node.create_subscription(
            Float64,
            "/speed_scaling_state_broadcaster/speed_scaling",
            self.speed_scaling_callback,
            10,
        )
        self.io_states_sub = self.node.create_subscription(
            IOStates, "/io_and_status_controller/io_states", self.io_states_callback, 10
        )

        # Program state (external control)
        # Driver publishes latched (transient_local) and only on change,
        # so the subscription QoS must match to receive the last value.
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.program_running_sub = self.node.create_subscription(
            Bool,
            "/io_and_status_controller/robot_program_running",
            self.program_running_callback,
            latched_qos,
        )
        self.robot_mode_sub = self.node.create_subscription(
            RobotMode,
            "/io_and_status_controller/robot_mode",
            self.robot_mode_callback,
            latched_qos,
        )
        self.safety_mode_sub = self.node.create_subscription(
            SafetyMode,
            "/io_and_status_controller/safety_mode",
            self.safety_mode_callback,
            latched_qos,
        )

        # Remote/Local: no topic exists — poll the dashboard service every 5s
        # and cache the result (stays unknown on fake hardware)
        self.remote_control_client = self.node.create_client(
            IsInRemoteControl, "/dashboard_client/is_in_remote_control"
        )
        self.remote_control_timer = self.node.create_timer(
            5.0, self._poll_remote_control
        )

        # Program watchdog (auto-regain control after e-stop / Local mode)
        self.resend_program_client = self.node.create_client(
            Trigger, "/io_and_status_controller/resend_robot_program"
        )
        self.dashboard_connect_client = self.node.create_client(
            Trigger, "/dashboard_client/connect"
        )
        self.watchdog_timer = self.node.create_timer(0.5, self.auto_regain_control)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # State variables
        self.connected = False
        self.latest_joint_state = None
        self.speed_slider = 1.0  # User-set speed slider value
        self.speed_scaling = 1.0  # Actual speed scaling from robot (speed_slider * target_speed_fraction)

        self.digital_in_states = [False] * 18
        self.digital_out_states = [False] * 18

        self.tcp_pose_matrix = np.eye(4)  # 4x4 T-matrix (base -> tool0_controller)
        self.tcp_pose_available = False

        self.program_running = False  # Latest robot_program_running value
        self.program_state_received = False  # Not published on fake hardware
        self.program_maybe_paused = (
            False  # Safety left NORMAL while program was running - PAUSE suspected
        )
        self.resend_in_flight = False  # A resend request is awaiting response
        self._resend_future = None  # In-flight resend future (for timeout pruning)

        self.robot_mode = RobotMode.DISCONNECTED  # ur_dashboard_msgs RobotMode
        self.safety_mode = 0  # ur_dashboard_msgs SafetyMode (0 = not received)
        self.remote_control = -1  # 1=remote, 0=local, -1=unknown
        self.dashboard_needs_reconnect = (
            False  # dashboard_client TCP socket died (Local switch) - reconnecting
        )

        # Steady clock for watchdog throttles (immune to sim-time jumps)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.last_resend_time = Time(
            clock_type=ClockType.STEADY_TIME
        )  # Throttle: one resend per 3s
        self.resend_sent_time = Time(
            clock_type=ClockType.STEADY_TIME
        )  # In-flight timeout tracking
        self.last_dashboard_connect_time = Time(
            clock_type=ClockType.STEADY_TIME
        )  # Throttle: one dashboard connect() attempt per 30s

        # Connection flags
        self.joint_state_ready = False
        self.speed_ready = False
        self.io_ready = False
        self.tcp_ready = False

        self.node.get_logger().info("UR Robot Controller initialized")

    # ========================================================
    # Connection / Ready (Public)
    # ========================================================
    def is_connected(self):
        """
        Connected means: joint_states received at least once (joint_states 1회 이상)

        Returns:
            bool: True if connected
        """
        return self.connected

    def is_robot_ready(self, require_io=False):
        """
        Robot ready means: essential state streams are ready.

        Args:
            require_io (bool): If True, IO states must also be received.

        Returns:
            bool: True if robot is ready
        """
        # Check essential state streams
        base_ready = self.joint_state_ready and self.speed_ready and self.tcp_ready

        # Check optional IO states if required
        if require_io:
            return base_ready and self.io_ready
        return base_ready

    async def wait_robot_ready(self, timeout=5.0, require_io=False):
        """
        Wait until robot is fully ready (상태 수신 완료 대기).

        Conditions:
        - joint_states received
        - speed_scaling received
        - tcp TF available
        - (optional) IO states received

        Args:
            timeout (float): timeout seconds
            require_io (bool): require IO states also

        Returns:
            bool: True if ready
        """
        # Get start time
        start = asyncio.get_event_loop().time()

        # Wait until timeout or robot is ready
        while asyncio.get_event_loop().time() - start < timeout:
            if self.is_robot_ready(require_io=require_io):
                self.node.get_logger().info("🤖 Robot fully ready")
                return True
            await asyncio.sleep(0.05)

        self.node.get_logger().error("❌ Robot not ready (timeout)")
        return False

    # ========================================================
    # State Monitoring (Public Query Methods)
    # ========================================================
    def get_joint_positions(self):
        """
        Get latest joint positions

        Returns:
            list: 6 joint positions in radians (shoulder_pan → wrist_3 order),
                  or None if not available
        """
        if not (self.latest_joint_state and self.connected):
            return None

        # Map by joint name to handle any publish order (e.g. alphabetical),
        # mirroring the C++ client
        expected_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        names = list(self.latest_joint_state.name)
        positions = list(self.latest_joint_state.position)
        if len(names) < 6 or len(positions) < 6:
            return None

        try:
            return [positions[names.index(n)] for n in expected_names]
        except ValueError:
            return None

    def get_tcp_pose(self):
        """
        Get current TCP pose as 4x4 transformation matrix.

        Returns:
            np.ndarray: 4x4 homogeneous transformation matrix (base -> tool0_controller)
        """
        return self.tcp_pose_matrix

    def is_tcp_pose_available(self):
        """
        Check if TCP pose is available from TF.

        Returns:
            bool: True if TCP pose is being tracked via TF
        """
        return self.tcp_pose_available

    def get_speed_slider(self):
        """
        Get current speed slider value (user-set value).

        Returns:
            float: Speed slider fraction [0.01 ~ 1.0]
        """
        return self.speed_slider

    def get_speed_scaling(self):
        """
        Get current speed scaling (actual robot speed).

        Note:
            speed_scaling = speed_slider * target_speed_fraction
            In normal operation (target_speed_fraction=1.0), they are the same.

        Returns:
            float: Actual speed scaling [0.0 ~ 1.0]
        """
        return self.speed_scaling

    def get_digital_in(self, pin):
        """Get digital input pin state"""
        if 0 <= pin < 18:
            return self.digital_in_states[pin]
        return False

    def get_digital_out(self, pin):
        """Get digital output pin state"""
        if 0 <= pin < 18:
            return self.digital_out_states[pin]
        return False

    def is_program_running(self):
        """
        Check if the robot program (external control) is running.

        Note:
            False means the driver lost control (e-stop, Local mode, ...).
            On fake hardware this topic is not published, so the value
            stays False (check program_state_received to distinguish).

        Returns:
            bool: True if the driver reports the program as running
        """
        return self.program_running

    def get_robot_mode(self):
        """
        Get latest robot mode (ur_dashboard_msgs RobotMode constants).

        Returns:
            int: e.g. RUNNING(7), IDLE(5), POWER_OFF(3); DISCONNECTED(0) before first message
        """
        return self.robot_mode

    def get_safety_mode(self):
        """
        Get latest safety mode (ur_dashboard_msgs SafetyMode constants).

        Returns:
            int: e.g. NORMAL(1), PROTECTIVE_STOP(3); 0 before first message
        """
        return self.safety_mode

    def is_remote_control(self):
        """
        Check if the Teach Pendant is in Remote Control mode.

        Note:
            No topic exists for this — the value is cached from a periodic
            (5s) dashboard service poll, so it may lag reality by a few
            seconds. Stays -1 on fake hardware (no dashboard_client).

        Returns:
            int: 1 = remote, 0 = local, -1 = unknown
        """
        return self.remote_control

    # ========================================================
    # Motion Control
    # ========================================================
    async def move_j(self, joints, velocity=0.5, timeout=30.0):
        """
        Execute MoveJ motion

        Args:
            joints (list): 6 joint positions in radians
            velocity (float): Velocity scaling [0.01 ~ 1.0]
            timeout (float): Maximum wait time in seconds

        Returns:
            tuple: (success: bool, message: str)
        """
        # Check joint length
        if len(joints) != 6:
            self.node.get_logger().error(
                f"MoveJ requires 6 joint values, got {len(joints)}"
            )
            return False, "Invalid joint length"

        # Check action server availability
        if not self.movej_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error("MoveJ action server not available")
            return False, "Action server not available"

        # Create goal
        goal = MoveJ.Goal()
        goal.joints = joints
        goal.velocity = velocity

        self.node.get_logger().info(f"Sending MoveJ goal: velocity={velocity:.2f}")

        # Create async future
        loop = asyncio.get_event_loop()
        result_future = loop.create_future()

        # Send goal asynchronously
        send_goal_future = self.movej_client.send_goal_async(goal)

        # ========================== Goal response callback ==========================
        def _goal_response_cb(future):
            try:
                goal_handle = future.result()
            except Exception as e:
                self.node.get_logger().error(f"MoveJ goal response failed: {e}")
                loop.call_soon_threadsafe(
                    result_future.set_result, (False, "Goal response exception")
                )
                return

            if not goal_handle.accepted:
                self.node.get_logger().error("MoveJ goal rejected")
                loop.call_soon_threadsafe(
                    result_future.set_result, (False, "Goal rejected")
                )
                return

            self.node.get_logger().info("MoveJ goal accepted")
            self.current_movej_goal = goal_handle

            # Request result asynchronously
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(_result_cb)

        # ========================== Result callback =============================
        def _result_cb(future):
            try:
                result = future.result().result
            except Exception as e:
                self.node.get_logger().error(f"MoveJ result failed: {e}")
                loop.call_soon_threadsafe(
                    result_future.set_result, (False, "Result exception")
                )
                return

            # Clear current MoveJ goal handle
            self.current_movej_goal = None

            if result.success:
                self.node.get_logger().info(f"✅ MoveJ succeeded: {result.message}")
                loop.call_soon_threadsafe(
                    result_future.set_result, (True, result.message)
                )
            else:
                self.node.get_logger().error(f"❌ MoveJ failed: {result.message}")
                loop.call_soon_threadsafe(
                    result_future.set_result, (False, result.message)
                )

        # Register callback
        send_goal_future.add_done_callback(_goal_response_cb)

        # Wait for completion
        try:
            result = await asyncio.wait_for(result_future, timeout=timeout)
            return result
        except asyncio.TimeoutError:
            return False, f"❌ MoveJ failed: timeout after {timeout}s"

    async def move_l(self, tmatrix, velocity=0.5, timeout=30.0):
        """
        Execute MoveL motion

        Args:
            tmatrix (list): 4x4 transformation matrix (16 elements, row-major)
            velocity (float): Velocity scaling [0.01 ~ 1.0]
            timeout (float): Maximum wait time in seconds

        Returns:
            tuple: (success: bool, message: str)
        """
        # Check matrix length
        if len(tmatrix) != 16:
            self.node.get_logger().error(
                f"MoveL requires 16 elements, got {len(tmatrix)}"
            )
            return False, "Invalid matrix length"

        # Check action server availability
        if not self.movel_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("MoveL action server not available")
            return False, "Action server not available"

        # Create goal
        goal = MoveL.Goal()
        goal.target_tmatrix = tmatrix
        goal.velocity = velocity

        self.node.get_logger().info(f"Sending MoveL goal: velocity={velocity:.2f}")

        # Create async future
        loop = asyncio.get_event_loop()
        result_future = loop.create_future()

        # Send goal asynchronously
        send_goal_future = self.movel_client.send_goal_async(goal)

        # ========================== Goal response callback ==========================
        def _goal_response_cb(future):
            try:
                goal_handle = future.result()
            except Exception as e:
                self.node.get_logger().error(f"MoveL goal response failed: {e}")
                loop.call_soon_threadsafe(
                    result_future.set_result, (False, "Goal response exception")
                )
                return

            if not goal_handle.accepted:
                self.node.get_logger().error("MoveL goal rejected")
                loop.call_soon_threadsafe(
                    result_future.set_result, (False, "Goal rejected")
                )
                return

            self.node.get_logger().info("MoveL goal accepted")

            # Save goal handle
            self.current_movel_goal = goal_handle

            # Request result asynchronously
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(_result_cb)

        # ========================== Result callback =============================
        def _result_cb(future):
            try:
                result = future.result().result
            except Exception as e:
                self.node.get_logger().error(f"MoveL result failed: {e}")
                loop.call_soon_threadsafe(
                    result_future.set_result, (False, "Result exception")
                )
                return

            # Clear current MoveL goal handle
            self.current_movel_goal = None

            if result.success:
                self.node.get_logger().info(f"✅ MoveL succeeded: {result.message}")
                loop.call_soon_threadsafe(
                    result_future.set_result, (True, result.message)
                )
            else:
                self.node.get_logger().error(f"❌ MoveL failed: {result.message}")
                loop.call_soon_threadsafe(
                    result_future.set_result, (False, result.message)
                )

        # Register callback
        send_goal_future.add_done_callback(_goal_response_cb)

        # Wait for completion
        try:
            result = await asyncio.wait_for(result_future, timeout=timeout)
            return result
        except asyncio.TimeoutError:
            return False, f"❌ MoveL failed: timeout after {timeout}s"

    def move_cancel(self):
        """
        Cancel current MoveL / MoveJ motion if exists

        Returns:
            bool: True if cancellation request was sent
        """
        cancelled = False

        # Cancel MoveL if active
        if self.current_movel_goal:
            self.node.get_logger().info("🛑 Cancelling current MoveL goal")
            self.current_movel_goal.cancel_goal_async()
            self.current_movel_goal = None
            cancelled = True

        # Cancel MoveJ if active
        if self.current_movej_goal:
            self.node.get_logger().info("🛑 Cancelling current MoveJ goal")
            self.current_movej_goal.cancel_goal_async()
            self.current_movej_goal = None
            cancelled = True

        # Check if no active motion to cancel
        if not cancelled:
            self.node.get_logger().warn("❌ No active motion to cancel")

        return cancelled

    # ========================================================
    # Utility Functions
    # ========================================================
    async def wait(self, duration_sec):
        """
        Wait for specified duration

        Usage: Used for delays between motions (모션 사이 지연 대기)

        Args:
            duration_sec (float): Wait duration in seconds
        """
        await asyncio.sleep(duration_sec)

    # ========================================================
    # Speed Control
    # ========================================================
    async def set_speed_slider(self, slider_value, timeout=1.0):
        """
        Set speed slider value

        Args:
            slider_value (float): Speed slider value [0.01 ~ 1.0]
            timeout (float): Maximum wait time in seconds

        Returns:
            tuple: (success: bool, message: str)
        """
        # Check speed slider value range
        if not 0.01 <= slider_value <= 1.0:
            self.node.get_logger().error(
                f"Speed slider must be in [0.01, 1.0], got {slider_value}"
            )
            return False, "Invalid speed slider value"

        # Check service availability
        if not self.speed_slider_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("Speed slider service not available")
            return False, "Speed slider service not available"

        # Create request
        request = SetSpeedSliderFraction.Request()
        request.speed_slider_fraction = slider_value

        # Create async future
        loop = asyncio.get_event_loop()
        result_future = loop.create_future()

        # Send request asynchronously
        send_request_future = self.speed_slider_client.call_async(request)

        # ========================== Response callback =============================
        def _response_cb(future):
            try:
                response = future.result()
            except Exception as e:
                self.node.get_logger().error(f"Speed slider service failed: {e}")
                loop.call_soon_threadsafe(
                    result_future.set_result, (False, "Service exception")
                )
                return

            if response.success:
                self.speed_slider = slider_value
                self.node.get_logger().info(
                    f"✅ Speed slider set to {slider_value * 100:.1f}%"
                )
                loop.call_soon_threadsafe(result_future.set_result, (True, "Success"))
            else:
                self.node.get_logger().error("❌ Failed to set speed slider")
                loop.call_soon_threadsafe(result_future.set_result, (False, "Failed"))

        # Register callback
        send_request_future.add_done_callback(_response_cb)

        try:
            result = await asyncio.wait_for(result_future, timeout=timeout)
            return result
        except asyncio.TimeoutError:
            return False, f"❌ Speed slider failed: timeout after {timeout}s"

    # ========================================================
    # I/O Control
    # ========================================================
    async def set_digital_out(self, pin, value, timeout=1.0):
        """
        Set digital output pin

        Args:
            pin (int): Pin number [0-17]
            value (bool): Output value (True=HIGH, False=LOW)
            timeout (float): Maximum wait time in seconds

        Returns:
            tuple: (success: bool, message: str)
        """
        if not 0 <= pin <= 17:
            self.node.get_logger().error(f"Invalid pin number: {pin} (must be 0-17)")
            return False, "Invalid pin number"

        if not self.set_io_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("I/O service not available")
            return False, "I/O service not available"

        # Create request
        request = SetIO.Request()
        request.fun = 1
        request.pin = pin
        request.state = 1.0 if value else 0.0

        # Create async future
        loop = asyncio.get_event_loop()
        result_future = loop.create_future()

        # Send request asynchronously
        send_request_future = self.set_io_client.call_async(request)

        # ========================== Response callback =============================
        def _response_cb(future):
            try:
                response = future.result()
            except Exception as e:
                self.node.get_logger().error(f"I/O service failed: {e}")
                loop.call_soon_threadsafe(
                    result_future.set_result, (False, "Service exception")
                )
                return

            if response.success:
                self.node.get_logger().info(
                    f'✅ Digital output pin {pin} set to {"HIGH" if value else "LOW"}'
                )
                loop.call_soon_threadsafe(result_future.set_result, (True, "Success"))
            else:
                self.node.get_logger().error(
                    f"❌ Failed to set digital output pin {pin}"
                )
                loop.call_soon_threadsafe(result_future.set_result, (False, "Failed"))

        # Register callback
        send_request_future.add_done_callback(_response_cb)

        try:
            result = await asyncio.wait_for(result_future, timeout=timeout)
            return result
        except asyncio.TimeoutError:
            return False, f"❌ Digital output failed: timeout after {timeout}s"

    # ========================================================
    # Internal Callbacks
    # ========================================================
    def joint_state_callback(self, msg):
        """Callback for joint state updates"""
        self.latest_joint_state = msg

        # Log first time only
        if not self.joint_state_ready:
            self.node.get_logger().info(
                f"✅ Robot connected! Received joint states (joint count: {len(msg.name)})"
            )

        # Set joint state ready to True and connected to True
        self.joint_state_ready = True
        self.connected = True

        # Update TCP pose from TF
        self._update_tcp_pose_from_tf()

    def speed_scaling_callback(self, msg):
        """Callback for speed scaling updates"""
        self.speed_scaling = msg.data / 100.0

        # Log first time only
        if not self.speed_ready:
            self.node.get_logger().info(
                f"⚡ Speed scaling updates received: {msg.data:.1f}%"
            )

        # Set speed ready to True
        self.speed_ready = True

    def io_states_callback(self, msg):
        """Callback for I/O states updates"""
        # Update digital inputs
        for i in range(min(len(msg.digital_in_states), 18)):
            self.digital_in_states[i] = msg.digital_in_states[i].state > 0.5

        # Update digital outputs
        for i in range(min(len(msg.digital_out_states), 18)):
            self.digital_out_states[i] = msg.digital_out_states[i].state > 0.5

        # Log first time only
        if not self.io_ready:
            self.node.get_logger().info(
                f"🔌 I/O states received (DI: {len(msg.digital_in_states)}, DO: {len(msg.digital_out_states)})"
            )

        # Set io ready to True
        self.io_ready = True

    def program_running_callback(self, msg):
        """Callback for robot program (external control) state updates"""
        prev = self.program_running
        self.program_running = msg.data

        # Log first reception, then only transitions
        if not self.program_state_received:
            self.program_state_received = True
            self.node.get_logger().info(
                f'🤖 Program state received: {"running" if msg.data else "stopped"}'
            )
        elif msg.data and not prev:
            self.node.get_logger().info("✅ Robot program running - control regained")
        elif not msg.data and prev:
            self.node.get_logger().warn("⚠️ Robot program stopped - control lost")

    def robot_mode_callback(self, msg):
        """Callback for robot mode updates (log only on change)"""
        prev = self.robot_mode
        self.robot_mode = msg.mode
        if prev != msg.mode:
            if msg.mode == RobotMode.RUNNING:
                self.node.get_logger().info(
                    f"🤖 Robot mode: RUNNING ({prev} -> {msg.mode})"
                )
            else:
                self.node.get_logger().warn(
                    f"🤖 Robot mode changed: {prev} -> {msg.mode}"
                )

    def safety_mode_callback(self, msg):
        """Callback for safety mode updates (log only on change)"""
        prev = self.safety_mode
        self.safety_mode = msg.mode
        if prev != msg.mode:
            if msg.mode == SafetyMode.NORMAL:
                self.node.get_logger().info(
                    f"🛡️ Safety mode: NORMAL ({prev} -> {msg.mode})"
                )
            else:
                self.node.get_logger().warn(
                    f"🛡️ Safety mode changed: {prev} -> {msg.mode}"
                )
                if (
                    self.program_state_received
                    and self.program_running
                    and not self.program_maybe_paused
                ):
                    self.program_maybe_paused = True
                    self.node.get_logger().warn(
                        "⏸️ Safety left NORMAL while program was running - PAUSE "
                        f"suspected, recovery armed (safety_mode: {prev} -> {msg.mode})"
                    )

    def auto_regain_control(self):
        """Watchdog tick (500ms): resend the program once the robot recovers
        (RUNNING + NORMAL) from e-stop/Local mode. Never automates physical
        recovery (e-stop release, power/brake, Remote switch)."""
        # program_maybe_paused lets this proceed even though e-stop leaves
        # program_running True (PAUSE, not stop)
        if not self.program_state_received or (
            self.program_running and not self.program_maybe_paused
        ):
            return

        # Wait until the operator finished physical recovery
        if (
            self.robot_mode != RobotMode.RUNNING
            or self.safety_mode != SafetyMode.NORMAL
        ):
            return

        now = self.steady_clock.now()

        # Drop a stuck in-flight request (e.g. driver restarted mid-call)
        if self.resend_in_flight:
            if (now - self.resend_sent_time).nanoseconds / 1e9 > 10.0:
                self.node.get_logger().warn(
                    "⚠️ resend_robot_program response timed out - resetting"
                )
                if self._resend_future is not None:
                    self.resend_program_client.remove_pending_request(
                        self._resend_future
                    )
                    self._resend_future = None
                self.resend_in_flight = False
            else:
                return

        # Throttle: one resend attempt per 3 seconds
        if (now - self.last_resend_time).nanoseconds / 1e9 < 3.0:
            return

        if not self.resend_program_client.service_is_ready():
            self.node.get_logger().warn(
                "⚠️ resend_robot_program service not available",
                throttle_duration_sec=10.0,
            )
            return

        self.last_resend_time = now
        self.resend_sent_time = now
        self.resend_in_flight = True

        self.node.get_logger().info(
            "🔄 Attempting to regain robot control (resend_robot_program)..."
        )

        self._resend_future = self.resend_program_client.call_async(Trigger.Request())
        self._resend_future.add_done_callback(self._resend_response)

    def _resend_response(self, future):
        """Handle resend_robot_program response (success ≠ program actually resumed)"""
        self.resend_in_flight = False
        self._resend_future = None
        try:
            response = future.result()
            if not response.success:
                self.node.get_logger().warn(
                    "❌ resend_robot_program failed - will retry"
                )
            elif self.remote_control == 1:
                self.program_maybe_paused = False
            # else: Local "success" is a false-positive - stay armed, let throttle retry
        except Exception as e:
            self.node.get_logger().warn(f"❌ resend_robot_program exception: {e}")

    def _poll_remote_control(self):
        """Poll is_in_remote_control and cache the result; if dashboard_client's
        socket died (Local switch), retry /dashboard_client/connect instead (30s throttle)
        """
        if self.dashboard_needs_reconnect:
            now = self.steady_clock.now()
            if (now - self.last_dashboard_connect_time).nanoseconds / 1e9 < 30.0:
                return
            if not self.dashboard_connect_client.service_is_ready():
                return

            self.last_dashboard_connect_time = now
            future = self.dashboard_connect_client.call_async(Trigger.Request())
            future.add_done_callback(self._dashboard_connect_response)
            return

        if not self.remote_control_client.service_is_ready():
            return
        future = self.remote_control_client.call_async(IsInRemoteControl.Request())
        future.add_done_callback(self._remote_control_response)

    def _remote_control_response(self, future):
        """Cache Remote/Local mode; log only on transition"""
        try:
            response = future.result()
            if not response.success:
                if not self.dashboard_needs_reconnect:
                    self.dashboard_needs_reconnect = True
                    self.node.get_logger().warn(
                        "⚠️ dashboard_client is_in_remote_control failed - "
                        "will retry connect (30s throttle)"
                    )
                return
        except Exception:
            if not self.dashboard_needs_reconnect:
                self.dashboard_needs_reconnect = True
                self.node.get_logger().warn(
                    "⚠️ dashboard_client is_in_remote_control exception - "
                    "will retry connect (30s throttle)"
                )
            return

        prev = self.remote_control
        self.remote_control = 1 if response.remote_control else 0
        if self.remote_control == 0 and prev != 0:
            self.node.get_logger().warn(
                "⚠️ Pendant is in LOCAL mode - switch to Remote to regain control"
            )
        elif self.remote_control == 1 and prev == 0:
            self.node.get_logger().info("✅ Pendant switched to Remote control")

    def _dashboard_connect_response(self, future):
        """Handle /dashboard_client/connect response; resumes normal polling on success"""
        try:
            response = future.result()
            if response.success and self.dashboard_needs_reconnect:
                self.dashboard_needs_reconnect = False
                self.node.get_logger().info("✅ dashboard_client reconnected")
        except Exception:
            pass  # Best-effort only - next 30s throttle window retries

    # ========================================================
    # Internal Helper Methods
    # ========================================================
    def _update_tcp_pose_from_tf(self):
        """Update TCP pose from TF transform"""
        try:
            transform = self.tf_buffer.lookup_transform(
                "base",  # Target frame (robot base)
                "tool0_controller",  # Source frame (actual TCP from UR driver)
                rclpy.time.Time(),  # Latest available
                rclpy.duration.Duration(seconds=0.1),  # 100ms timeout
            )

            # Convert to 4x4 matrix
            self.tcp_pose_matrix = self._transform_to_matrix(transform.transform)

            # Mark as available (log only on first success)
            if not self.tcp_pose_available:
                self.node.get_logger().info(
                    "📍 TCP pose tracking active (TF synchronized)"
                )

            # Set tcp pose available to True and tcp ready to True
            self.tcp_pose_available = True
            self.tcp_ready = True

        except Exception as e:
            if self.tcp_pose_available:
                self.node.get_logger().warn(
                    f"Lost TF transform (base -> tool0_controller): {e}"
                )
            # Set tcp pose available to False and tcp ready to False
            self.tcp_pose_available = False
            self.tcp_ready = False

    def _transform_to_matrix(self, transform):
        """
        Convert ROS Transform to 4x4 homogeneous transformation matrix.

        Args:
            transform: geometry_msgs.msg.Transform

        Returns:
            np.ndarray: 4x4 transformation matrix
        """
        T = np.eye(4)

        # Translation
        T[0, 3] = transform.translation.x
        T[1, 3] = transform.translation.y
        T[2, 3] = transform.translation.z

        # Rotation (quaternion to matrix)
        quat = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
        rot_matrix = R.from_quat(quat).as_matrix()
        T[:3, :3] = rot_matrix

        return T
