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
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <rclcpp/executors.hpp>
#include <string>
#include <thread>
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
    bool isConnected() const;
    bool isRobotReady(bool require_io = false) const;
    bool waitRobotReady(double timeout_sec = 5.0, bool require_io = false);

    // ========================================================
    // State Monitoring
    // ========================================================
    bool isTcpPoseAvailable() const;

    std::array<double, 6>  getJointPositions() const;
    std::array<double, 16> getTcpPose() const;

    double getSpeedSlider() const;
    double getSpeedScaling() const;

    bool getDigitalIn(int pin) const;
    bool getDigitalOut(int pin) const;

    // ========================================================
    // Motion Control
    // ========================================================
    std::future<MotionResult> moveJ(
        const std::vector<double>& joints,
        double                     velocity = 0.5,
        double                     timeout  = 30.0);
    std::future<MotionResult> moveL(
        const std::array<double, 16>& tmatrix,
        double                        velocity = 0.5,
        double                        timeout  = 30.0);
    bool moveCancel();

    // ========================================================
    // Speed Control
    // ========================================================
    std::future<MotionResult> setSpeedSlider(
        double fraction,
        double timeout = 1.0);

    // ========================================================
    // I/O Control
    // ========================================================
    std::future<MotionResult> setDigitalOut(
        int    pin,
        bool   value,
        double timeout = 1.0);

   private:
    // ========================================================
    // Initialization
    // ========================================================
    // * Executor
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
    std::thread                                         executor_thread_;
    std::atomic<bool>                                   executor_running_;

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
    // Executor Thread
    // ========================================================
    void startExecutorThread();
    void stopExecutorThread();

    // ========================================================
    // Callbacks
    // ========================================================
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void speedScalingCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void ioStatesCallback(const ur_msgs::msg::IOStates::SharedPtr msg);

    // ========================================================
    // Helper Functions
    // ========================================================
    void                   updateTcpPoseFromTf();
    std::array<double, 16> transformToMatrix(const geometry_msgs::msg::Transform& transform);
};

#endif  // UR_ROBOT_CLIENT_HPP