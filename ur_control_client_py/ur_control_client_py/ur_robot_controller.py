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
"""

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener
from ur_motion.action import MoveJ, MoveL
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO, SetSpeedSliderFraction


class URRobotController:
    """
    UR Robot Controller (non-Node class).

    This class provides a high-level interface for controlling UR robots.
    It requires a ROS2 node instance to be passed in the constructor.

    Example:
        node = Node('my_node')
        robot = URRobotController(node)
        robot.move_j([0, -1.57, 1.57, -1.57, -1.57, 0], 0.5)
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

        # State variables
        self.latest_joint_state = None
        self.connected = False
        self.speed_slider = 1.0  # User-set speed slider value
        self.speed_scaling = 1.0  # Actual speed scaling from robot (speed_slider * target_speed_fraction)
        self.digital_in_states = [False] * 18
        self.digital_out_states = [False] * 18

        # TF setup for TCP pose tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.tcp_pose_matrix = np.eye(4)  # 4x4 T-matrix (base -> tool0_controller)
        self.tcp_pose_available = False

        self.node.get_logger().info("UR Robot Controller initialized")

    # ========================================================
    # State Monitoring (Public Query Methods)
    # ========================================================
    def is_connected(self):
        """Check if robot is connected"""
        return self.connected

    def get_joint_positions(self):
        """
        Get latest joint positions

        Returns:
            list: 6 joint positions in radians, or None if not available
        """
        if self.latest_joint_state and self.connected:
            if len(self.latest_joint_state.position) >= 6:
                return list(self.latest_joint_state.position[:6])
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

    # ========================================================
    # Motion Control
    # ========================================================
    def move_j(self, joints, velocity=0.5, wait=True):
        """
        Execute MoveJ motion

        Args:
            joints (list): 6 joint positions in radians
            velocity (float): Velocity scaling [0.01 ~ 1.0]
            wait (bool): If True, blocks until complete; if False, returns immediately

        Returns:
            bool: True if command sent successfully (wait=False) or completed successfully (wait=True)
        """
        if len(joints) != 6:
            self.node.get_logger().error(
                f"MoveJ requires 6 joint values, got {len(joints)}"
            )
            return False

        if not self.movej_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("MoveJ action server not available")
            return False

        goal = MoveJ.Goal()
        goal.joints = joints
        goal.velocity = velocity

        self.node.get_logger().info(f"Sending MoveJ goal: velocity={velocity:.2f}")

        send_goal_future = self.movej_client.send_goal_async(goal)

        if not wait:
            return True

        # Blocking mode - wait for completion
        # Note: Caller should execute this in a separate thread if needed
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=10.0)

        if not send_goal_future.done():
            self.node.get_logger().error("MoveJ goal response timeout")
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("MoveJ goal rejected")
            return False

        self.node.get_logger().info("MoveJ goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=120.0)

        if not result_future.done():
            self.node.get_logger().error("MoveJ result timeout")
            return False

        result = result_future.result().result
        if result.success:
            self.node.get_logger().info(f"✅ MoveJ succeeded: {result.message}")
            return True
        else:
            self.node.get_logger().error(f"❌ MoveJ failed: {result.message}")
            return False

    def move_l(self, tmatrix, velocity=0.5, wait=True):
        """
        Execute MoveL motion

        Args:
            tmatrix (list): 4x4 transformation matrix (16 elements, row-major)
            velocity (float): Velocity scaling [0.01 ~ 1.0]
            wait (bool): If True, blocks until complete; if False, returns immediately

        Returns:
            bool: True if command sent successfully (wait=False) or completed successfully (wait=True)
        """
        if len(tmatrix) != 16:
            self.node.get_logger().error(
                f"MoveL requires 16 elements, got {len(tmatrix)}"
            )
            return False

        if not self.movel_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("MoveL action server not available")
            return False

        goal = MoveL.Goal()
        goal.target_tmatrix = tmatrix
        goal.velocity = velocity

        self.node.get_logger().info(f"Sending MoveL goal: velocity={velocity:.2f}")

        send_goal_future = self.movel_client.send_goal_async(goal)

        if not wait:
            return True

        # Blocking mode - wait for completion
        # Note: Caller should execute this in a separate thread if needed
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=10.0)

        if not send_goal_future.done():
            self.node.get_logger().error("MoveL goal response timeout")
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("MoveL goal rejected")
            return False

        self.node.get_logger().info("MoveL goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=120.0)

        if not result_future.done():
            self.node.get_logger().error("MoveL result timeout")
            return False

        result = result_future.result().result
        if result.success:
            self.node.get_logger().info(f"✅ MoveL succeeded: {result.message}")
            return True
        else:
            self.node.get_logger().error(f"❌ MoveL failed: {result.message}")
            return False

    # ========================================================
    # Speed Control
    # ========================================================
    def set_speed_slider(self, slider_value, wait=True):
        """
        Set speed slider value

        Args:
            slider_value (float): Speed slider value [0.01 ~ 1.0]
            wait (bool): Wait for response (default: True)
                        Set to False to call asynchronously during motion

        Returns:
            bool: True if succeeded (when wait=True)
            Future: Service call future (when wait=False)
        """
        if not 0.01 <= slider_value <= 1.0:
            self.node.get_logger().warn(
                f"Speed slider must be in [0.01, 1.0], got {slider_value}"
            )
            return False

        if not self.speed_slider_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("Speed slider service not available")
            return False

        request = SetSpeedSliderFraction.Request()
        request.speed_slider_fraction = slider_value

        future = self.speed_slider_client.call_async(request)

        if not wait:
            # Async mode: return future without waiting
            # Update internal state immediately (optimistic update)
            self.speed_slider = slider_value
            self.node.get_logger().info(
                f"🔄 Speed slider change requested: {slider_value * 100:.1f}% (async)"
            )
            return future

        # Sync mode: wait for response
        rclpy.spin_until_future_complete(self.node, future)

        response = future.result()
        if response.success:
            self.speed_slider = slider_value  # Update internal state
            self.node.get_logger().info(
                f"✅ Speed slider set to {slider_value * 100:.1f}%"
            )
        else:
            self.node.get_logger().warn("❌ Failed to set speed slider")

        return response.success

    # ========================================================
    # I/O Control
    # ========================================================
    def set_digital_out(self, pin, value):
        """
        Set digital output pin

        Args:
            pin (int): Pin number [0-17]
            value (bool): Output value (True=HIGH, False=LOW)

        Returns:
            bool: True if succeeded
        """
        if not 0 <= pin <= 17:
            self.node.get_logger().warn(f"Invalid pin number: {pin} (must be 0-17)")
            return False

        if not self.set_io_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("I/O service not available")
            return False

        request = SetIO.Request()
        request.fun = 1  # Set digital output
        request.pin = pin
        request.state = 1.0 if value else 0.0

        future = self.set_io_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        response = future.result()
        if response.success:
            self.node.get_logger().info(
                f'✅ Digital output pin {pin} set to {"HIGH" if value else "LOW"}'
            )
        else:
            self.node.get_logger().warn(f"❌ Failed to set digital output pin {pin}")

        return response.success

    # ========================================================
    # Internal Callbacks
    # ========================================================
    def joint_state_callback(self, msg):
        """Callback for joint state updates"""
        self.latest_joint_state = msg

        was_connected = self.connected
        self.connected = True

        if not was_connected:
            self.node.get_logger().info(
                f"✅ Robot connected! Received joint states (joint count: {len(msg.name)})"
            )

        # Update TCP pose from TF
        self._update_tcp_pose_from_tf()

    def speed_scaling_callback(self, msg):
        """Callback for speed scaling updates"""
        self.speed_scaling = msg.data / 100.0

        # Log first time only
        if not hasattr(self, "_speed_logged"):
            self._speed_logged = True
            self.node.get_logger().info(
                f"⚡ Speed scaling updates received: {msg.data:.1f}%"
            )

    def io_states_callback(self, msg):
        """Callback for I/O states updates"""
        # Update digital inputs
        for i in range(min(len(msg.digital_in_states), 18)):
            self.digital_in_states[i] = msg.digital_in_states[i].state > 0.5

        # Update digital outputs
        for i in range(min(len(msg.digital_out_states), 18)):
            self.digital_out_states[i] = msg.digital_out_states[i].state > 0.5

        # Log first time only
        if not hasattr(self, "_io_logged"):
            self._io_logged = True
            self.node.get_logger().info(
                f"🔌 I/O states received (DI: {len(msg.digital_in_states)}, DO: {len(msg.digital_out_states)})"
            )

    # ========================================================
    # Internal Helper Methods
    # ========================================================
    def _update_tcp_pose_from_tf(self):
        """Update TCP pose from TF transform (internal method called by joint_state_callback)"""
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
                self.tcp_pose_available = True

        except Exception as e:
            if self.tcp_pose_available:
                self.node.get_logger().warn(
                    f"Lost TF transform (base -> tool0_controller): {e}"
                )
                self.tcp_pose_available = False

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
