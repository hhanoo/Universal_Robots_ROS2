#include "ur_robot_client/ur_robot_client.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <thread>

using namespace std::chrono_literals;

URRobotClient::URRobotClient()
    : Node("ur_robot_client"),
      executor_running_(false),
      program_running_(false),
      program_state_received_(false),
      control_lost_logged_(false),
      resend_in_flight_(false),
      robot_mode_(ur_dashboard_msgs::msg::RobotMode::DISCONNECTED),
      safety_mode_(0),
      last_resend_time_(0, 0, RCL_STEADY_TIME),
      resend_sent_time_(0, 0, RCL_STEADY_TIME),
      connected_(false),
      last_joint_state_time_(rclcpp::Clock().now()),
      speed_slider_(1.0),
      speed_scaling_(1.0),
      tcp_pose_available_(false),
      joint_state_ready_(false),
      speed_ready_(false),
      io_ready_(false),
      tcp_ready_(false) {
    RCLCPP_INFO(this->get_logger(), "Initializing UR Robot Client...");

    // Initialize Action Clients
    movej_client_ = rclcpp_action::create_client<ur_motion::action::MoveJ>(
        this, "/move_j");
    movel_client_ = rclcpp_action::create_client<ur_motion::action::MoveL>(
        this, "/move_l");

    // Initialize Service Clients
    speed_slider_client_ = this->create_client<ur_msgs::srv::SetSpeedSliderFraction>(
        "/io_and_status_controller/set_speed_slider");
    set_io_client_ = this->create_client<ur_msgs::srv::SetIO>(
        "/io_and_status_controller/set_io");

    // Initialize Subscribers
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        10,
        std::bind(&URRobotClient::jointStateCallback, this, std::placeholders::_1));

    speed_scaling_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/speed_scaling_state_broadcaster/speed_scaling",
        10,
        std::bind(&URRobotClient::speedScalingCallback, this, std::placeholders::_1));

    io_states_sub_ = this->create_subscription<ur_msgs::msg::IOStates>(
        "/io_and_status_controller/io_states",
        10,
        std::bind(&URRobotClient::ioStatesCallback, this, std::placeholders::_1));

    // Initialize Program Watchdog
    // Driver publishes these as latched (transient_local) and only on change,
    // so the subscription QoS must match to receive the last value.
    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    program_running_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/io_and_status_controller/robot_program_running",
        latched_qos,
        std::bind(&URRobotClient::programRunningCallback, this, std::placeholders::_1));

    robot_mode_sub_ = this->create_subscription<ur_dashboard_msgs::msg::RobotMode>(
        "/io_and_status_controller/robot_mode",
        latched_qos,
        std::bind(&URRobotClient::robotModeCallback, this, std::placeholders::_1));

    safety_mode_sub_ = this->create_subscription<ur_dashboard_msgs::msg::SafetyMode>(
        "/io_and_status_controller/safety_mode",
        latched_qos,
        std::bind(&URRobotClient::safetyModeCallback, this, std::placeholders::_1));

    resend_program_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/io_and_status_controller/resend_robot_program");
    remote_control_client_ = this->create_client<ur_dashboard_msgs::srv::IsInRemoteControl>(
        "/dashboard_client/is_in_remote_control");

    watchdog_timer_ = this->create_wall_timer(
        500ms,
        std::bind(&URRobotClient::autoRegainControl, this));

    // Initialize TF
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize TCP pose matrix
    tcp_pose_matrix_.fill(0.0);
    tcp_pose_matrix_[0] = tcp_pose_matrix_[5] = tcp_pose_matrix_[10] = tcp_pose_matrix_[15] = 1.0;

    // Initialize I/O states
    digital_in_states_.fill(false);
    digital_out_states_.fill(false);

    // Start executor thread
    startExecutorThread();

    RCLCPP_INFO(this->get_logger(), "UR Robot Controller initialized");
}

URRobotClient::~URRobotClient() {
    RCLCPP_INFO(this->get_logger(), "UR Robot Client shutting down");
    stopExecutorThread();
}

// ========================================================================================
// Connection & Robot Ready
// ========================================================================================
/**
 * @brief Check if robot is connected
 * @return true if connected
 *
 * Note: Connected means joint_states received at least once
 */
bool URRobotClient::isConnected() const {
    return connected_;
}

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
bool URRobotClient::isRobotReady(bool require_io) const {
    bool base_ready = joint_state_ready_ && speed_ready_ && tcp_ready_;

    if (require_io) {
        return base_ready && io_ready_;
    }
    return base_ready;
}

/**
 * @brief Wait until robot is fully ready
 * @param timeout_sec Maximum wait time in seconds (default: 5.0)
 * @param require_io If true, IO states must also be received
 * @return true if ready within timeout
 *
 */
bool URRobotClient::waitRobotReady(double timeout_sec, bool require_io) {
    auto start_time = this->now();
    auto timeout    = rclcpp::Duration::from_seconds(timeout_sec);

    while (rclcpp::ok() && (this->now() - start_time) < timeout) {
        if (isRobotReady(require_io)) {
            RCLCPP_INFO(this->get_logger(), "🤖 Robot fully ready");
            return true;
        }
        std::this_thread::sleep_for(50ms);
    }

    RCLCPP_ERROR(this->get_logger(), "❌ Robot not ready (timeout)");
    return false;
}

// ========================================================================================
// State Monitoring
// ========================================================================================
/**
 * @brief Check if TCP pose is available from TF
 * @return true if TCP pose is being tracked via TF
 *
 */
bool URRobotClient::isTcpPoseAvailable() const {
    return tcp_pose_available_;
}

/**
 * @brief Check if the robot program (external control) is running
 * @return true if the driver reports the program as running
 *
 * Note: false means the driver lost control (e-stop, Local mode, ...)
 *       and the watchdog is trying to regain it automatically.
 */
bool URRobotClient::isProgramRunning() const {
    return program_running_;
}

/**
 * @brief Check if the Teach Pendant is in Remote Control mode
 * @return 1 = remote, 0 = local, -1 = unknown (dashboard not available yet)
 *
 * Note: There is no topic for this — the value is cached from a periodic
 *       (5s) dashboard service poll, so it may lag reality by a few seconds.
 *       Stays -1 on fake hardware (no dashboard_client).
 */
int URRobotClient::isRemoteControl() const {
    return remote_control_;
}

/**
 * @brief Get latest robot mode (ur_dashboard_msgs::msg::RobotMode constants)
 * @return e.g. RUNNING(7), IDLE(5), POWER_OFF(3); DISCONNECTED(0) before first message
 */
int8_t URRobotClient::getRobotMode() const {
    return robot_mode_;
}

/**
 * @brief Get latest safety mode (ur_dashboard_msgs::msg::SafetyMode constants)
 * @return e.g. NORMAL(1), PROTECTIVE_STOP(3); 0 before first message
 */
uint8_t URRobotClient::getSafetyMode() const {
    return safety_mode_;
}

/**
 * @brief Get latest joint positions
 * @return 6 joint positions in radians
 */
std::array<double, 6> URRobotClient::getJointPositions() const {
    if (!latest_joint_state_ || !connected_) {
        return std::array<double, 6>();
    }

    // Check if data is recent (within 1 second)
    rclcpp::Duration time_since_update = this->now() - last_joint_state_time_;
    if (time_since_update.seconds() > 1.0) {
        return std::array<double, 6>();
    }

    // Map by joint name to handle any publish order (e.g. alphabetical)
    static const std::array<std::string, 6> expected_names = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    const auto& names     = latest_joint_state_->name;
    const auto& positions = latest_joint_state_->position;

    if (names.size() < 6 || positions.size() < 6) {
        return std::array<double, 6>();
    }

    std::array<double, 6> joints{};
    for (size_t i = 0; i < 6; ++i) {
        auto it = std::find(names.begin(), names.end(), expected_names[i]);
        if (it == names.end()) {
            return std::array<double, 6>();
        }
        joints[i] = positions[std::distance(names.begin(), it)];
    }
    return joints;
}

/**
 * @brief Get current TCP pose as 4x4 transformation matrix
 * @return 4x4 homogeneous transformation matrix (base -> tool0_controller)
 *
 */
std::array<double, 16> URRobotClient::getTcpPose() const {
    return tcp_pose_matrix_;
}

/**
 * @brief Get current speed slider value (user-set)
 * @return Speed slider fraction [0.01 ~ 1.0]
 *
 */
double URRobotClient::getSpeedSlider() const {
    return speed_slider_;
}

/**
 * @brief Get current speed scaling (actual robot speed)
 * @return Current speed scaling [0.0 ~ 1.0]
 *
 * Note: speed_scaling = speed_slider * target_speed_fraction
 *       In normal operation (target_speed_fraction=1.0), they are the same.
 */
double URRobotClient::getSpeedScaling() const {
    return speed_scaling_;
}

/**
 * @brief Get digital input pin state
 * @param pin Pin number [0-17]
 * @return Pin state (true=HIGH, false=LOW)
 */
bool URRobotClient::getDigitalIn(int pin) const {
    if (pin < 0 || pin >= 18) {
        return false;
    }
    return digital_in_states_[pin];
}

/**
 * @brief Get digital output pin state
 * @param pin Pin number [0-17]
 * @return Pin state (true=HIGH, false=LOW)
 */
bool URRobotClient::getDigitalOut(int pin) const {
    if (pin < 0 || pin >= 18) {
        return false;
    }
    return digital_out_states_[pin];
}

// ========================================================================================
// Motion Control
// ========================================================================================
/**
 * @brief Execute MoveJ (joint space motion)
 * @param joints 6 joint positions in radians
 * @param velocity Velocity scaling [0.01 ~ 1.0]
 * @param timeout Maximum wait time in seconds (default: 30.0)
 * @return Future with MotionResult (success, message)
 *
 */
std::future<URRobotClient::MotionResult> URRobotClient::moveJ(
    // Create promise and future
    const std::vector<double>& joints, double velocity, double timeout) {
    auto promise = std::make_shared<std::promise<MotionResult>>();
    auto future  = promise->get_future();

    // Create timeout timer and goal handle pointer
    auto timeout_timer   = std::make_shared<rclcpp::TimerBase::SharedPtr>();
    auto goal_handle_ptr = std::make_shared<std::shared_ptr<GoalHandleMoveJ>>();

    // Check joint length
    if (joints.size() != 6) {
        RCLCPP_ERROR(this->get_logger(), "MoveJ requires 6 joint values, got %zu", joints.size());
        promise->set_value({false, "Invalid joint length"});
        return future;
    }

    // Check action server availability
    RCLCPP_INFO(this->get_logger(), "Waiting for MoveJ action server...");
    if (!movej_client_->wait_for_action_server(5s)) {
        RCLCPP_ERROR(this->get_logger(), "MoveJ action server not available");
        promise->set_value({false, "Action server not available"});
        return future;
    }
    RCLCPP_INFO(this->get_logger(), "MoveJ action server connected");

    // Create goal
    auto goal     = ur_motion::action::MoveJ::Goal();
    goal.joints   = joints;
    goal.velocity = velocity;

    RCLCPP_INFO(this->get_logger(), "Sending MoveJ goal: velocity=%.2f", velocity);

    // Create timeout timer
    *timeout_timer = this->create_wall_timer(
        std::chrono::duration<double>(timeout),
        [this, promise, goal_handle_ptr, timeout_timer]() {
            if (*goal_handle_ptr) {
                RCLCPP_INFO(this->get_logger(), "Cancelling goal due to timeout");
                movej_client_->async_cancel_goal(*goal_handle_ptr);
            }

            try {
                promise->set_value({false, "Timeout"});
            } catch (const std::future_error&) {
                // Result already set
            }

            (*timeout_timer)->cancel();
        });

    // Send goal options
    auto send_goal_options = rclcpp_action::Client<ur_motion::action::MoveJ>::SendGoalOptions();

    // Goal response callback
    send_goal_options.goal_response_callback =
        [this, promise, goal_handle_ptr, timeout_timer](std::shared_ptr<GoalHandleMoveJ> goal_handle) {
            // Goal rejected
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "MoveJ goal rejected");

                (*timeout_timer)->cancel();

                try {
                    promise->set_value({false, "Goal rejected"});
                } catch (const std::future_error&) {
                    // Result already set
                }
                return;
            }

            // Goal accepted
            RCLCPP_INFO(this->get_logger(), "MoveJ goal accepted");
            *goal_handle_ptr = goal_handle;

            // Save goal handle to mutex
            {
                std::lock_guard<std::mutex> lock(goal_mutex_);
                current_movej_goal_ = goal_handle;
            }
        };

    // Result callback
    send_goal_options.result_callback =
        [this, promise, timeout_timer](const GoalHandleMoveJ::WrappedResult& result) {
            // Cancel timeout timer
            (*timeout_timer)->cancel();

            // Clear current MoveJ goal handle
            {
                std::lock_guard<std::mutex> lock(goal_mutex_);
                current_movej_goal_.reset();
            }

            // Create motion result
            MotionResult motion_result;

            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "✅ MoveJ succeeded: %s", result.result->message.c_str());
                    motion_result = {result.result->success, result.result->message};
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "🛑 MoveJ canceled");
                    motion_result = {false, "Canceled"};
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "❌ MoveJ aborted");
                    motion_result = {false, "Aborted"};
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "❌ MoveJ failed");
                    motion_result = {false, "Failed"};
                    break;
            }

            // Set result to promise
            try {
                promise->set_value(motion_result);
            } catch (const std::future_error&) {
                RCLCPP_WARN(this->get_logger(), "Promise already set (result callback)");
            }
        };

    // Send goal
    movej_client_->async_send_goal(goal, send_goal_options);

    return future;
}

/**
 * @brief Execute MoveL (Cartesian linear motion)
 * @param tmatrix 4x4 transformation matrix (row-major, 16 elements)
 * @param velocity Velocity scaling [0.01 ~ 1.0]
 * @param timeout Maximum wait time in seconds (default: 30.0)
 * @return Future with MotionResult (success, message)
 *
 */
std::future<URRobotClient::MotionResult> URRobotClient::moveL(
    const std::array<double, 16>& tmatrix, double velocity, double timeout) {
    // Create promise and future
    auto promise = std::make_shared<std::promise<MotionResult>>();
    auto future  = promise->get_future();

    // Create timeout timer and goal handle pointer
    auto timeout_timer   = std::make_shared<rclcpp::TimerBase::SharedPtr>();
    auto goal_handle_ptr = std::make_shared<std::shared_ptr<GoalHandleMoveL>>();

    // Check action server availability
    if (!movel_client_->wait_for_action_server(5s)) {
        RCLCPP_ERROR(this->get_logger(), "MoveL action server not available");
        promise->set_value({false, "Action server not available"});
        return future;
    }

    // Create goal
    auto goal           = ur_motion::action::MoveL::Goal();
    goal.target_tmatrix = tmatrix;
    goal.velocity       = velocity;

    RCLCPP_INFO(this->get_logger(), "Sending MoveL goal: velocity=%.2f", velocity);

    // Create timeout timer
    *timeout_timer = this->create_wall_timer(
        std::chrono::duration<double>(timeout),
        [this, promise, goal_handle_ptr, timeout_timer]() {
            if (*goal_handle_ptr) {
                RCLCPP_ERROR(this->get_logger(), "❌ MoveL timeout - cancelling goal");
                movel_client_->async_cancel_goal(*goal_handle_ptr);
            }

            try {
                promise->set_value({false, "Timeout"});
            } catch (const std::future_error&) {
                // Result already set
            }

            (*timeout_timer)->cancel();
        });

    // Send goal options
    auto send_goal_options = rclcpp_action::Client<ur_motion::action::MoveL>::SendGoalOptions();

    // Goal response callback
    send_goal_options.goal_response_callback =
        [this, promise, goal_handle_ptr, timeout_timer](std::shared_ptr<GoalHandleMoveL> goal_handle) {
            // Goal rejected
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "MoveL goal rejected");

                (*timeout_timer)->cancel();

                try {
                    promise->set_value({false, "Goal rejected"});
                } catch (const std::future_error&) {
                    // Result already set
                }
                return;
            }

            // Goal accepted
            RCLCPP_INFO(this->get_logger(), "MoveL goal accepted");

            // Save goal handle to pointer
            *goal_handle_ptr = goal_handle;

            // Save goal handle to mutex
            {
                std::lock_guard<std::mutex> lock(goal_mutex_);
                current_movel_goal_ = goal_handle;
            }
        };

    // Result callback
    send_goal_options.result_callback =
        [this, promise, timeout_timer](const GoalHandleMoveL::WrappedResult& result) {
            // Cancel timeout timer
            (*timeout_timer)->cancel();

            // Clear current MoveL goal handle
            {
                std::lock_guard<std::mutex> lock(goal_mutex_);
                current_movel_goal_.reset();
            }

            // Create motion result
            MotionResult motion_result;

            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "✅ MoveL succeeded: %s",
                                result.result->message.c_str());
                    motion_result = {result.result->success, result.result->message};
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "🛑 MoveL canceled");
                    motion_result = {false, "Canceled"};
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "❌ MoveL aborted");
                    motion_result = {false, "Aborted"};
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "❌ MoveL failed");
                    motion_result = {false, "Failed"};
                    break;
            }

            // Set result to promise
            try {
                promise->set_value(motion_result);
            } catch (const std::future_error&) {
                // Result already set
            }
        };

    // Send goal
    movel_client_->async_send_goal(goal, send_goal_options);

    return future;
}

/**
 * @brief Cancel current MoveJ/MoveL motion
 * @return true if cancellation request was sent
 *
 */
bool URRobotClient::moveCancel() {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    bool                        cancelled = false;

    if (current_movel_goal_) {
        RCLCPP_INFO(this->get_logger(), "🛑 Cancelling current MoveL goal");
        movel_client_->async_cancel_goal(current_movel_goal_);
        current_movel_goal_.reset();
        cancelled = true;
    }

    if (current_movej_goal_) {
        RCLCPP_INFO(this->get_logger(), "🛑 Cancelling current MoveJ goal");
        movej_client_->async_cancel_goal(current_movej_goal_);
        current_movej_goal_.reset();
        cancelled = true;
    }

    if (!cancelled) {
        RCLCPP_WARN(this->get_logger(), "❌ No active motion to cancel");
    }

    return cancelled;
}

// ========================================================================================
// Speed Control
// ========================================================================================
/**
 * @brief Set speed slider fraction
 * @param fraction Speed slider [0.01 ~ 1.0]
 * @param timeout Maximum wait time in seconds (default: 1.0)
 * @return Future with MotionResult (success, message)
 *
 */
std::future<URRobotClient::MotionResult> URRobotClient::setSpeedSlider(
    double fraction, double timeout) {
    // Create promise and future
    auto promise = std::make_shared<std::promise<MotionResult>>();
    auto future  = promise->get_future();

    // Create timeout timer
    auto timeout_timer = std::make_shared<rclcpp::TimerBase::SharedPtr>();

    // Check speed slider value range
    if (fraction < 0.01 || fraction > 1.0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Speed slider must be in [0.01, 1.0], got %.2f", fraction);
        promise->set_value({false, "Invalid speed slider value"});
        return future;
    }

    // Check service availability
    if (!speed_slider_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(this->get_logger(), "Speed slider service not available");
        promise->set_value({false, "Service not available"});
        return future;
    }

    // Create request
    auto request                   = std::make_shared<ur_msgs::srv::SetSpeedSliderFraction::Request>();
    request->speed_slider_fraction = fraction;

    // Create timeout timer
    *timeout_timer = this->create_wall_timer(
        std::chrono::duration<double>(timeout),
        [promise, timeout_timer]() {
            try {
                promise->set_value({false, "Timeout"});
            } catch (const std::future_error&) {
                // Result already set
            }
            (*timeout_timer)->cancel();
        });

    // Response callback
    auto response_callback = [this, promise, fraction, timeout_timer](
                                 rclcpp::Client<ur_msgs::srv::SetSpeedSliderFraction>::SharedFuture future) {
        // Cancel timeout timer
        (*timeout_timer)->cancel();

        try {
            // Get response
            auto response = future.get();

            // Set speed slider
            if (response->success) {
                this->speed_slider_ = fraction;
                RCLCPP_INFO(this->get_logger(),
                            "✅ Speed slider set to %.1f%%", fraction * 100.0);

                try {
                    promise->set_value({true, "Success"});
                } catch (const std::future_error&) {
                    // Result already set
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ Failed to set speed slider");
                try {
                    promise->set_value({false, "Failed"});
                } catch (const std::future_error&) {
                    // Result already set
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Speed slider service failed: %s", e.what());
            try {
                promise->set_value({false, "Service exception"});
            } catch (const std::future_error&) {
                // Result already set
            }
        }
    };

    // Send request
    speed_slider_client_->async_send_request(request, response_callback);

    return future;
}

// ========================================================================================
// I/O Control
// ========================================================================================
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
std::future<URRobotClient::MotionResult> URRobotClient::setDigitalOut(
    int pin, bool value, double timeout) {
    // Create promise and future
    auto promise = std::make_shared<std::promise<MotionResult>>();
    auto future  = promise->get_future();

    // Create timeout timer
    auto timeout_timer = std::make_shared<rclcpp::TimerBase::SharedPtr>();

    // Check pin number range
    if (pin < 0 || pin > 17) {
        RCLCPP_ERROR(this->get_logger(), "Invalid pin number: %d (must be 0-17)", pin);
        promise->set_value({false, "Invalid pin number"});
        return future;
    }

    // Check service availability
    if (!set_io_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(this->get_logger(), "I/O service not available");
        promise->set_value({false, "Service not available"});
        return future;
    }

    // Create request
    auto request   = std::make_shared<ur_msgs::srv::SetIO::Request>();
    request->fun   = 1;
    request->pin   = pin;
    request->state = value ? 1.0 : 0.0;

    // Create timeout timer
    *timeout_timer = this->create_wall_timer(
        std::chrono::duration<double>(timeout),
        [promise, timeout_timer]() {
            try {
                promise->set_value({false, "Timeout"});
            } catch (const std::future_error&) {
                // Result already set
            }
            (*timeout_timer)->cancel();
        });

    // Response callback
    auto response_callback = [this, promise, pin, value, timeout_timer](
                                 rclcpp::Client<ur_msgs::srv::SetIO>::SharedFuture future) {
        // Cancel timeout timer
        (*timeout_timer)->cancel();

        try {
            // Get response
            auto response = future.get();

            // Set digital output
            if (response->success) {
                RCLCPP_INFO(this->get_logger(),
                            "✅ Digital output pin %d set to %s",
                            pin, value ? "HIGH" : "LOW");
                try {
                    promise->set_value({true, "Success"});
                } catch (const std::future_error&) {
                    // Result already set
                }
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "❌ Failed to set digital output pin %d", pin);
                try {
                    promise->set_value({false, "Failed"});
                } catch (const std::future_error&) {
                    // Result already set
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "I/O service failed: %s", e.what());
            try {
                promise->set_value({false, "Service exception"});
            } catch (const std::future_error&) {
                // Result already set
            }
        }
    };

    // Send request
    set_io_client_->async_send_request(request, response_callback);

    return future;
}

// ========================================================
// Executor Thread
// ========================================================
/**
 * @brief Start executor thread
 */
void URRobotClient::startExecutorThread() {
    executor_running_ = true;

    // Create MultiThreadedExecutor
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(this->get_node_base_interface());

    // Start executor in separate thread
    executor_thread_ = std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "🔄 Executor thread started");
        executor_->spin();
        RCLCPP_INFO(this->get_logger(), "🛑 Executor thread stopped");
    });

    RCLCPP_INFO(this->get_logger(), "✅ Background executor started");
}

/**
 * @brief Stop executor thread
 */
void URRobotClient::stopExecutorThread() {
    if (!executor_running_) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Stopping executor thread...");
    executor_running_ = false;

    // Cancel executor
    if (executor_) {
        executor_->cancel();
    }

    // Join executor thread
    if (executor_thread_.joinable()) {
        executor_thread_.join();
    }

    // Remove node from executor
    if (executor_) {
        executor_->remove_node(this->get_node_base_interface());
        executor_.reset();
    }

    RCLCPP_INFO(this->get_logger(), "✅ Executor thread stopped");
}

// ========================================================================================
// Callbacks
// ========================================================================================
void URRobotClient::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!msg) {
        return;
    }

    latest_joint_state_    = msg;
    last_joint_state_time_ = this->now();

    if (!joint_state_ready_) {
        RCLCPP_INFO(this->get_logger(),
                    "✅ Robot connected! Received joint states (joint count: %zu)",
                    msg->name.size());
        joint_state_ready_ = true;
    }

    connected_ = true;

    updateTcpPoseFromTf();
}

void URRobotClient::speedScalingCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    if (!msg) {
        return;
    }

    // Topic publishes percent (0~100); store as fraction [0.0 ~ 1.0]
    speed_scaling_ = msg->data / 100.0;

    if (!speed_ready_) {
        RCLCPP_INFO(this->get_logger(), "⚡ Speed scaling updates received: %.1f%%", msg->data);
        speed_ready_ = true;
    }
}

void URRobotClient::ioStatesCallback(const ur_msgs::msg::IOStates::SharedPtr msg) {
    if (!msg) {
        return;
    }

    // Update digital inputs
    for (size_t i = 0; i < msg->digital_in_states.size() && i < 18; ++i) {
        digital_in_states_[i] = msg->digital_in_states[i].state > 0.5;
    }

    // Update digital outputs
    for (size_t i = 0; i < msg->digital_out_states.size() && i < 18; ++i) {
        digital_out_states_[i] = msg->digital_out_states[i].state > 0.5;
    }

    if (!io_ready_) {
        RCLCPP_INFO(this->get_logger(), "🔌 I/O states received (DI: %zu, DO: %zu)",
                    msg->digital_in_states.size(), msg->digital_out_states.size());
        io_ready_ = true;
    }
}

void URRobotClient::programRunningCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg) {
        return;
    }

    program_running_        = msg->data;
    program_state_received_ = true;

    if (msg->data) {
        if (control_lost_logged_.exchange(false)) {
            RCLCPP_INFO(this->get_logger(), "✅ Robot program running - control regained");
        }
    } else {
        if (!control_lost_logged_.exchange(true)) {
            RCLCPP_WARN(this->get_logger(),
                        "⚠️ Robot program stopped - control lost, auto-regain armed "
                        "(robot_mode: %d, safety_mode: %u)",
                        static_cast<int>(robot_mode_.load()),
                        static_cast<unsigned>(safety_mode_.load()));
        }
    }
}

void URRobotClient::robotModeCallback(const ur_dashboard_msgs::msg::RobotMode::SharedPtr msg) {
    if (!msg) {
        return;
    }

    int8_t prev = robot_mode_.exchange(msg->mode);
    if (prev == msg->mode) {
        return;
    }

    if (msg->mode == ur_dashboard_msgs::msg::RobotMode::RUNNING) {
        RCLCPP_INFO(this->get_logger(), "🤖 Robot mode: RUNNING (%d -> %d)",
                    static_cast<int>(prev), static_cast<int>(msg->mode));
    } else {
        RCLCPP_WARN(this->get_logger(), "🤖 Robot mode changed: %d -> %d",
                    static_cast<int>(prev), static_cast<int>(msg->mode));
    }
}

void URRobotClient::safetyModeCallback(const ur_dashboard_msgs::msg::SafetyMode::SharedPtr msg) {
    if (!msg) {
        return;
    }

    uint8_t prev = safety_mode_.exchange(msg->mode);
    if (prev == msg->mode) {
        return;
    }

    if (msg->mode == ur_dashboard_msgs::msg::SafetyMode::NORMAL) {
        RCLCPP_INFO(this->get_logger(), "🛡️ Safety mode: NORMAL (%u -> %u)",
                    static_cast<unsigned>(prev), static_cast<unsigned>(msg->mode));
    } else {
        RCLCPP_WARN(this->get_logger(), "🛡️ Safety mode changed: %u -> %u",
                    static_cast<unsigned>(prev), static_cast<unsigned>(msg->mode));
    }
}

/**
 * @brief Watchdog tick (500ms): regain control after the operator recovers the robot
 *
 * Resends the external control program when it stopped (e-stop, Local mode, ...)
 * and the robot is physically ready again (RUNNING + NORMAL). Physical recovery
 * (releasing e-stop, power/brake, switching to Remote) is the operator's job -
 * this never calls unlock_protective_stop/restart_safety/power_on/brake_release.
 */
void URRobotClient::autoRegainControl() {
    // Periodic Remote/Local poll for state queries (every 10 ticks = 5s).
    // Runs regardless of program state so isRemoteControl() stays fresh.
    if (watchdog_tick_++ % 10 == 0) {
        checkRemoteControl();
    }

    // Armed only after the first program state message (stays dormant on fake HW)
    if (!program_state_received_ || program_running_) {
        return;
    }

    // Wait until the operator finished physical recovery
    if (robot_mode_ != ur_dashboard_msgs::msg::RobotMode::RUNNING ||
        safety_mode_ != ur_dashboard_msgs::msg::SafetyMode::NORMAL) {
        return;
    }

    auto now = steady_clock_.now();

    // Drop a stuck in-flight request (e.g. driver restarted mid-call)
    if (resend_in_flight_) {
        if ((now - resend_sent_time_).seconds() > 10.0) {
            RCLCPP_WARN(this->get_logger(),
                        "⚠️ resend_robot_program response timed out - resetting");
            resend_program_client_->prune_pending_requests();
            resend_in_flight_ = false;
        } else {
            return;
        }
    }

    // Throttle: one resend attempt per 3 seconds
    if ((now - last_resend_time_).seconds() < 3.0) {
        return;
    }

    if (!resend_program_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), steady_clock_, 10000,
                             "⚠️ resend_robot_program service not available");
        return;
    }

    last_resend_time_ = now;
    resend_sent_time_ = now;
    resend_in_flight_ = true;

    RCLCPP_INFO(this->get_logger(),
                "🔄 Attempting to regain robot control (resend_robot_program)...");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    resend_program_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            resend_in_flight_ = false;
            try {
                auto response = future.get();
                // success only means the program was written to the socket;
                // real confirmation is robot_program_running becoming true
                if (!response->success) {
                    RCLCPP_WARN(this->get_logger(),
                                "❌ resend_robot_program failed - will retry");
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(),
                            "❌ resend_robot_program exception: %s", e.what());
            }
        });

    checkRemoteControl();
}

/**
 * @brief Poll the pendant Remote/Local mode and cache it (isRemoteControl())
 *
 * Local mode makes resend_robot_program "succeed" without the program actually
 * running, so transitions are logged. The dashboard service only exists on
 * real hardware — on fake HW this silently skips and the cache stays unknown.
 */
void URRobotClient::checkRemoteControl() {
    if (!remote_control_client_->service_is_ready()) {
        return;
    }

    auto request = std::make_shared<ur_dashboard_msgs::srv::IsInRemoteControl::Request>();
    remote_control_client_->async_send_request(
        request,
        [this](rclcpp::Client<ur_dashboard_msgs::srv::IsInRemoteControl>::SharedFuture future) {
            try {
                auto response = future.get();
                if (!response->success) {
                    return;
                }

                // Cache + log only on transition
                int8_t prev = remote_control_.exchange(response->remote_control ? 1 : 0);
                if (!response->remote_control && prev != 0) {
                    RCLCPP_WARN(this->get_logger(),
                                "⚠️ Pendant is in LOCAL mode - switch to Remote to regain control");
                } else if (response->remote_control && prev == 0) {
                    RCLCPP_INFO(this->get_logger(), "✅ Pendant switched to Remote control");
                }
            } catch (const std::exception&) {
                // Best-effort only
            }
        });
}

// ========================================================================================
// Helper Functions
// ========================================================================================
/**
 * @brief Update TCP pose from TF transform
 */
void URRobotClient::updateTcpPoseFromTf() {
    try {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_->lookupTransform(
            "base",                    // Target frame (robot base)
            "tool0_controller",        // Source frame (actual TCP from UR driver)
            tf2::TimePointZero,        // Latest available
            tf2::durationFromSec(0.1)  // 100ms timeout
        );

        // Convert to 4x4 matrix
        tcp_pose_matrix_ = transformToMatrix(transform_stamped.transform);

        // Mark as available (log only on first success)
        if (!tcp_pose_available_) {
            RCLCPP_INFO(this->get_logger(), "📍 TCP pose tracking active (TF synchronized)");
        }

        tcp_pose_available_ = true;
        tcp_ready_          = true;

    } catch (const tf2::TransformException& e) {
        if (tcp_pose_available_) {
            RCLCPP_WARN(this->get_logger(),
                        "Lost TF transform (base -> tool0_controller): %s", e.what());
        }
        tcp_pose_available_ = false;
        tcp_ready_          = false;
    }
}

/**
 * @brief Convert ROS Transform to 4x4 transformation matrix
 * @param transform ROS Transform message
 * @return 4x4 transformation matrix (row-major)
 */
std::array<double, 16> URRobotClient::transformToMatrix(
    const geometry_msgs::msg::Transform& transform) {
    std::array<double, 16> matrix;
    matrix.fill(0.0);

    // Translation
    matrix[3]  = transform.translation.x;  // T[0, 3]
    matrix[7]  = transform.translation.y;  // T[1, 3]
    matrix[11] = transform.translation.z;  // T[2, 3]

    // Rotation (quaternion to matrix)
    double qx = transform.rotation.x;
    double qy = transform.rotation.y;
    double qz = transform.rotation.z;
    double qw = transform.rotation.w;

    // Rotation matrix calculation (quaternion -> rotation matrix)
    matrix[0] = 1 - 2 * (qy * qy + qz * qz);  // R[0,0]
    matrix[1] = 2 * (qx * qy - qz * qw);      // R[0,1]
    matrix[2] = 2 * (qx * qz + qy * qw);      // R[0,2]

    matrix[4] = 2 * (qx * qy + qz * qw);      // R[1,0]
    matrix[5] = 1 - 2 * (qx * qx + qz * qz);  // R[1,1]
    matrix[6] = 2 * (qy * qz - qx * qw);      // R[1,2]

    matrix[8]  = 2 * (qx * qz - qy * qw);      // R[2,0]
    matrix[9]  = 2 * (qy * qz + qx * qw);      // R[2,1]
    matrix[10] = 1 - 2 * (qx * qx + qy * qy);  // R[2,2]

    matrix[15] = 1.0;  // T[3,3]

    return matrix;
}