"""
UR Control Client Library (Python)
===================================
Python client library for controlling UR robots

Features:
- MoveJ/MoveL motion control
- Speed slider control
- Digital I/O control
- Robot state monitoring
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetSpeedSliderFraction, SetIO
from ur_motion.action import MoveJ, MoveL


class URControlClient(Node):
    """UR Robot Control Client"""

    def __init__(self):
        super().__init__('ur_control_client_py')

        # Action clients
        self.movej_client = ActionClient(self, MoveJ, '/move_j')
        self.movel_client = ActionClient(self, MoveL, '/move_l')

        # Service clients
        self.speed_slider_client = self.create_client(SetSpeedSliderFraction, '/io_and_status_controller/set_speed_slider')
        self.set_io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')

        # Subscribers
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.speed_scaling_sub = self.create_subscription(Float64, '/speed_scaling_state_broadcaster/speed_scaling',
                                                          self.speed_scaling_callback, 10)
        self.io_states_sub = self.create_subscription(IOStates, '/io_and_status_controller/io_states', self.io_states_callback,
                                                      10)

        # State variables
        self.latest_joint_state = None
        self.connected = False
        self.speed_scaling = 1.0
        self.digital_in_states = [False] * 18
        self.digital_out_states = [False] * 18

        self.get_logger().info('UR Control Client (Python) initialized')

    # ========== Motion Control ==========

    def move_j(self, joints, velocity=0.5, wait=True):
        """
        Execute MoveJ motion
        
        Args:
            joints (list): 6 joint positions in radians
            velocity (float): Velocity scaling [0.01 ~ 1.0]
            wait (bool): Wait for motion to complete
        
        Returns:
            bool: True if succeeded
        """
        if len(joints) != 6:
            self.get_logger().error(f'MoveJ requires 6 joint values, got {len(joints)}')
            return False

        if not self.movej_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveJ action server not available')
            return False

        goal = MoveJ.Goal()
        goal.joints = joints
        goal.velocity = velocity

        self.get_logger().info(f'Sending MoveJ goal: velocity={velocity:.2f}')

        send_goal_future = self.movej_client.send_goal_async(goal)

        if not wait:
            return True

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('MoveJ goal rejected')
            return False

        self.get_logger().info('MoveJ goal accepted, waiting for result...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.success:
            self.get_logger().info(f'✅ MoveJ succeeded: {result.message}')
            return True
        else:
            self.get_logger().error(f'❌ MoveJ failed: {result.message}')
            return False

    def move_l(self, tmatrix, velocity=0.5, wait=True):
        """
        Execute MoveL motion
        
        Args:
            tmatrix (list): 4x4 transformation matrix (16 elements, row-major)
            velocity (float): Velocity scaling [0.01 ~ 1.0]
            wait (bool): Wait for motion to complete
        
        Returns:
            bool: True if succeeded
        """
        if len(tmatrix) != 16:
            self.get_logger().error(f'MoveL requires 16 elements, got {len(tmatrix)}')
            return False

        if not self.movel_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveL action server not available')
            return False

        goal = MoveL.Goal()
        goal.target_tmatrix = tmatrix
        goal.velocity = velocity

        self.get_logger().info(f'Sending MoveL goal: velocity={velocity:.2f}')

        send_goal_future = self.movel_client.send_goal_async(goal)

        if not wait:
            return True

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('MoveL goal rejected')
            return False

        self.get_logger().info('MoveL goal accepted, waiting for result...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.success:
            self.get_logger().info(f'✅ MoveL succeeded: {result.message}')
            return True
        else:
            self.get_logger().error(f'❌ MoveL failed: {result.message}')
            return False

    # ========== Speed Control ==========

    def set_speed_slider(self, fraction):
        """
        Set speed slider fraction
        
        Args:
            fraction (float): Speed slider [0.01 ~ 1.0]
        
        Returns:
            bool: True if succeeded
        """
        if not 0.01 <= fraction <= 1.0:
            self.get_logger().warn(f'Speed slider must be in [0.01, 1.0], got {fraction}')
            return False

        if not self.speed_slider_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Speed slider service not available')
            return False

        request = SetSpeedSliderFraction.Request()
        request.speed_slider_fraction = fraction

        future = self.speed_slider_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.success:
            self.get_logger().info(f'✅ Speed slider set to {fraction * 100:.1f}%')
        else:
            self.get_logger().warn('❌ Failed to set speed slider')

        return response.success

    def get_speed_scaling(self):
        """Get current speed scaling"""
        return self.speed_scaling

    # ========== I/O Control ==========

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
            self.get_logger().warn(f'Invalid pin number: {pin} (must be 0-17)')
            return False

        if not self.set_io_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('I/O service not available')
            return False

        request = SetIO.Request()
        request.fun = 1  # Set digital output
        request.pin = pin
        request.state = 1.0 if value else 0.0

        future = self.set_io_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.success:
            self.get_logger().info(f'✅ Digital output pin {pin} set to {"HIGH" if value else "LOW"}')
        else:
            self.get_logger().warn(f'❌ Failed to set digital output pin {pin}')

        return response.success

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

    # ========== State Monitoring ==========

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

    # ========== Callbacks ==========

    def joint_state_callback(self, msg):
        """Callback for joint state updates"""
        self.latest_joint_state = msg

        was_connected = self.connected
        self.connected = True

        if not was_connected:
            self.get_logger().info(f'✅ Robot connected! Received joint states (joint count: {len(msg.name)})')

    def speed_scaling_callback(self, msg):
        """Callback for speed scaling updates"""
        self.speed_scaling = msg.data / 100.0

        # Log first time only
        if not hasattr(self, '_speed_logged'):
            self._speed_logged = True
            self.get_logger().info(f'⚡ Speed scaling updates received: {msg.data:.1f}%')

    def io_states_callback(self, msg):
        """Callback for I/O states updates"""
        # Update digital inputs
        for i in range(min(len(msg.digital_in_states), 18)):
            self.digital_in_states[i] = msg.digital_in_states[i].state > 0.5

        # Update digital outputs
        for i in range(min(len(msg.digital_out_states), 18)):
            self.digital_out_states[i] = msg.digital_out_states[i].state > 0.5

        # Log first time only
        if not hasattr(self, '_io_logged'):
            self._io_logged = True
            self.get_logger().info(f'🔌 I/O states received (DI: {len(msg.digital_in_states)}, DO: {len(msg.digital_out_states)})')
