#ifndef UR_ROBOT_CLIENT_HPP
#define UR_ROBOT_CLIENT_HPP

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

// TF2 for TCP pose tracking
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// UR-specific messages and services
#include "ur_dashboard_msgs/msg/robot_mode.hpp"
#include "ur_dashboard_msgs/msg/safety_mode.hpp"
#include "ur_dashboard_msgs/srv/is_in_remote_control.hpp"
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
    bool isProgramRunning() const;
    int  isRemoteControl() const;  // 1=remote, 0=local, -1=unknown

    int8_t  getRobotMode() const;   // ur_dashboard_msgs::msg::RobotMode 상수
    uint8_t getSafetyMode() const;  // ur_dashboard_msgs::msg::SafetyMode 상수

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
        double                     timeout  = 60.0);
    std::future<MotionResult> moveL(
        const std::array<double, 16>& tmatrix,
        double                        velocity = 0.5,
        double                        timeout  = 60.0);
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

    // * Program watchdog (auto-regain control after e-stop / Local mode)
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                 program_running_sub_;
    rclcpp::Subscription<ur_dashboard_msgs::msg::RobotMode>::SharedPtr   robot_mode_sub_;
    rclcpp::Subscription<ur_dashboard_msgs::msg::SafetyMode>::SharedPtr  safety_mode_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr                    resend_program_client_;
    rclcpp::Client<ur_dashboard_msgs::srv::IsInRemoteControl>::SharedPtr remote_control_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr                    dashboard_connect_client_;
    rclcpp::TimerBase::SharedPtr                                         watchdog_timer_;

    std::atomic<bool>    program_running_;                   // Latest robot_program_running value
    std::atomic<bool>    program_state_received_;            // Watchdog armed only after first message
    std::atomic<bool>    control_lost_logged_;               // Pairs "control lost" / "control regained" logs
    std::atomic<bool>    resend_in_flight_;                  // A resend request is awaiting response
    std::atomic<bool>    program_maybe_paused_{false};       // Safety left NORMAL while program was running - PAUSE suspected
    std::atomic<int8_t>  robot_mode_;                        // ur_dashboard_msgs::msg::RobotMode
    std::atomic<uint8_t> safety_mode_;                       // ur_dashboard_msgs::msg::SafetyMode
    std::atomic<int8_t>  remote_control_{-1};                // 1=remote, 0=local, -1=unknown
    std::atomic<bool>    dashboard_needs_reconnect_{false};  // dashboard_client TCP socket died (Local switch) - reconnecting
    unsigned             watchdog_tick_{0};                  // Remote 폴링 주기 계산용 (5s)

    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    rclcpp::Time  last_resend_time_;             // Throttle: one resend per 3 seconds
    rclcpp::Time  resend_sent_time_;             // In-flight timeout tracking
    rclcpp::Time  last_dashboard_connect_time_;  // Throttle: one dashboard connect() attempt per 30 seconds

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
    void programRunningCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void robotModeCallback(const ur_dashboard_msgs::msg::RobotMode::SharedPtr msg);
    void safetyModeCallback(const ur_dashboard_msgs::msg::SafetyMode::SharedPtr msg);
    void autoRegainControl();
    void checkRemoteControl();

    // ========================================================
    // Helper Functions
    // ========================================================
    void                   updateTcpPoseFromTf();
    std::array<double, 16> transformToMatrix(const geometry_msgs::msg::Transform& transform);
};

#endif  // UR_ROBOT_CLIENT_HPP