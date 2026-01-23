#ifndef UR_ROBOT_CLIENT_HPP
#define UR_ROBOT_CLIENT_HPP

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

// TF2 for TCP pose tracking
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// UR-specific messages and services
#include "ur_msgs/msg/io_states.hpp"
#include "ur_msgs/srv/set_io.hpp"
#include "ur_msgs/srv/set_speed_slider_fraction.hpp"

// Action definitions
#include <array>
#include <chrono>
#include <functional>
#include <future>  // 추가: async 패턴 지원
#include <memory>
#include <mutex>  // 추가: goal handle 보호
#include <string>
#include <vector>

#include "ur_motion/action/move_j.hpp"
#include "ur_motion/action/move_l.hpp"

/**
 * @brief UR Robot Control Client
 *
 * Provides high-level interface for controlling UR robots:
 *   - MoveJ/MoveL motion control
 *   - Speed slider control
 *   - Digital I/O control
 *   - Robot state monitoring
 *   - TCP pose tracking via TF
 */
class URRobotClient : public rclcpp::Node {
   public:
    struct MotionResult {
        bool        success;
        std::string message;
    };

    URRobotClient();
    ~URRobotClient();

    // ========================================================
    // Connection & Robot Ready
    // ========================================================
    /**
     * @brief Check if robot is connected
     * @return true if connected
     *
     * Note: Connected means joint_states received at least once
     */
    bool isConnected() const;

    /**
     * @brief Check if robot is fully ready
     * @param require_io If true, IO states must also be received
     * @return true if robot is ready
     *
     *
     * Ready conditions:
     * - joint_states received
     * - speed_scaling received
     * - TCP TF available
     * - (optional) IO states received
     */
    bool isRobotReady(bool require_io = false) const;

    /**
     * @brief Wait until robot is fully ready
     * @param timeout_sec Maximum wait time in seconds (default: 5.0)
     * @param require_io If true, IO states must also be received
     * @return true if ready within timeout
     *
     */
    bool waitRobotReady(double timeout_sec = 5.0, bool require_io = false);

    // ========================================================
    // State Monitoring
    // ========================================================
    /**
     * @brief Get latest joint positions
     * @param joints Output vector for 6 joint positions
     * @return true if valid data available
     */
    bool getJointPositions(std::vector<double>& joints) const;

    /**
     * @brief Get current TCP pose as 4x4 transformation matrix
     * @return 4x4 homogeneous transformation matrix (base -> tool0_controller)
     *
     */
    std::array<double, 16> getTcpPose() const;

    /**
     * @brief Check if TCP pose is available from TF
     * @return true if TCP pose is being tracked via TF
     *
     */
    bool isTcpPoseAvailable() const;

    /**
     * @brief Get current speed slider value (user-set)
     * @return Speed slider fraction [0.01 ~ 1.0]
     *
     */
    double getSpeedSlider() const;

    /**
     * @brief Get current speed scaling (actual robot speed)
     * @return Current speed scaling [0.0 ~ 1.0]
     *
     * Note: speed_scaling = speed_slider * target_speed_fraction
     *       In normal operation (target_speed_fraction=1.0), they are the same.
     */
    double getSpeedScaling() const;

    /**
     * @brief Get digital input pin state
     * @param pin Pin number [0-17]
     * @return Pin state (true=HIGH, false=LOW)
     */
    bool getDigitalIn(int pin) const;

    /**
     * @brief Get digital output pin state
     * @param pin Pin number [0-17]
     * @return Pin state (true=HIGH, false=LOW)
     */
    bool getDigitalOut(int pin) const;

    // ========================================================
    // Motion Control
    // ========================================================
    /**
     * @brief Execute MoveJ (joint space motion)
     * @param joints 6 joint positions in radians
     * @param velocity Velocity scaling [0.01 ~ 1.0]
     * @param timeout Maximum wait time in seconds (default: 30.0)
     * @return Future with MotionResult (success, message)
     *
     */
    std::future<MotionResult> moveJ(
        const std::vector<double>& joints,
        double                     velocity = 0.5,
        double                     timeout  = 30.0);

    /**
     * @brief Execute MoveL (Cartesian linear motion)
     * @param tmatrix 4x4 transformation matrix (row-major, 16 elements)
     * @param velocity Velocity scaling [0.01 ~ 1.0]
     * @param timeout Maximum wait time in seconds (default: 30.0)
     * @return Future with MotionResult (success, message)
     *
     */
    std::future<MotionResult> moveL(
        const std::array<double, 16>& tmatrix,
        double                        velocity = 0.5,
        double                        timeout  = 30.0);

    /**
     * @brief Cancel current MoveJ/MoveL motion
     * @return true if cancellation request was sent
     *
     */
    bool moveCancel();

    // ========================================================
    // Speed Control
    // ========================================================
    /**
     * @brief Set speed slider fraction
     * @param fraction Speed slider [0.01 ~ 1.0]
     * @param timeout Maximum wait time in seconds (default: 1.0)
     * @return Future with MotionResult (success, message)
     *
     */
    std::future<MotionResult> setSpeedSlider(
        double fraction,
        double timeout = 1.0);

    // ========================================================
    // I/O Control
    // ========================================================
    /**
     * @brief Set digital output pin
     * @param pin Pin number [0-17]
     *            0-7: Standard digital outputs
     *            8-15: Configurable digital outputs
     *            16-17: Tool digital outputs
     * @param value Output value (true=HIGH, false=LOW)
     * @param timeout Maximum wait time in seconds (default: 1.0)
     * @return Future with MotionResult (success, message)
     *
     */
    std::future<MotionResult> setDigitalOut(
        int    pin,
        bool   value,
        double timeout = 1.0);

   private:
    // ========================================================
    // Initialization
    // ========================================================
    // * Action Clients
    rclcpp_action::Client<ur_motion::action::MoveJ>::SharedPtr movej_client_;
    rclcpp_action::Client<ur_motion::action::MoveL>::SharedPtr movel_client_;

    using GoalHandleMoveJ = rclcpp_action::ClientGoalHandle<ur_motion::action::MoveJ>;
    using GoalHandleMoveL = rclcpp_action::ClientGoalHandle<ur_motion::action::MoveL>;

    std::shared_ptr<GoalHandleMoveJ> current_movej_goal_;
    std::shared_ptr<GoalHandleMoveL> current_movel_goal_;
    std::mutex                       goal_mutex_;

    // * Service Clients
    rclcpp::Client<ur_msgs::srv::SetSpeedSliderFraction>::SharedPtr speed_slider_client_;
    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr                  set_io_client_;

    // * Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr       speed_scaling_sub_;
    rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr       io_states_sub_;

    // * TF
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // * State variables
    bool connected_;

    sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
    rclcpp::Time                            last_joint_state_time_;

    double speed_slider_;   // User-set speed slider [0.01 ~ 1.0]
    double speed_scaling_;  // Current speed scaling [0.0 ~ 1.0]

    std::array<bool, 18> digital_in_states_;   // Digital input states
    std::array<bool, 18> digital_out_states_;  // Digital output states

    std::array<double, 16> tcp_pose_matrix_;  // 4x4 T-matrix (base -> tool0_controller)
    bool                   tcp_pose_available_;

    // * Connection flags
    bool joint_state_ready_;
    bool speed_ready_;
    bool io_ready_;
    bool tcp_ready_;

    // ========================================================
    // Callbacks
    // ========================================================
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void speedScalingCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void ioStatesCallback(const ur_msgs::msg::IOStates::SharedPtr msg);

    // ========================================================
    // Helper Functions
    // ========================================================
    /**
     * @brief Update TCP pose from TF transform
     */
    void updateTcpPoseFromTf();

    /**
     * @brief Convert ROS Transform to 4x4 transformation matrix
     * @param transform ROS Transform message
     * @return 4x4 transformation matrix (row-major)
     */
    std::array<double, 16> transformToMatrix(const geometry_msgs::msg::Transform& transform);
};

#endif  // UR_ROBOT_CLIENT_HPP