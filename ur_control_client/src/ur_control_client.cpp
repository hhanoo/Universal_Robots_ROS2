#include "ur_control_client/ur_control_client.hpp"

#include <chrono>
#include <future>

using namespace std::chrono_literals;

URControlClient::URControlClient()
    : Node("ur_control_client"),
      last_joint_state_time_(this->now()),
      connected_(false),
      speed_scaling_(1.0) {
    RCLCPP_INFO(this->get_logger(), "Initializing UR Control Client...");

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
        std::bind(&URControlClient::jointStateCallback, this, std::placeholders::_1));

    speed_scaling_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/speed_scaling_state_broadcaster/speed_scaling",
        10,
        std::bind(&URControlClient::speedScalingCallback, this, std::placeholders::_1));

    io_states_sub_ = this->create_subscription<ur_msgs::msg::IOStates>(
        "/io_and_status_controller/io_states",
        10,
        std::bind(&URControlClient::ioStatesCallback, this, std::placeholders::_1));

    // Initialize I/O states
    digital_in_states_.fill(false);
    digital_out_states_.fill(false);

    RCLCPP_INFO(this->get_logger(), "UR Control Client initialized");
}

URControlClient::~URControlClient() {
    RCLCPP_INFO(this->get_logger(), "UR Control Client shutting down");
}

// ============================================================
// Motion Control
// ============================================================

bool URControlClient::moveJ(const std::vector<double>& joints, double velocity, bool wait) {
    if (joints.size() != 6) {
        RCLCPP_ERROR(this->get_logger(), "MoveJ requires 6 joint values, got %zu", joints.size());
        return false;
    }

    if (!movej_client_->wait_for_action_server(5s)) {
        RCLCPP_ERROR(this->get_logger(), "MoveJ action server not available");
        return false;
    }

    // Create goal
    auto goal     = ur_motion::action::MoveJ::Goal();
    goal.joints   = joints;
    goal.velocity = velocity;

    RCLCPP_INFO(this->get_logger(), "Sending MoveJ goal: velocity=%.2f", velocity);

    // Send goal
    auto send_goal_options = rclcpp_action::Client<ur_motion::action::MoveJ>::SendGoalOptions();

    // Feedback callback
    send_goal_options.feedback_callback =
        [this](auto, const std::shared_ptr<const ur_motion::action::MoveJ::Feedback> feedback) {
            RCLCPP_INFO(this->get_logger(), "MoveJ feedback: progress=%.2f", feedback->progress);
        };

    auto goal_handle_future = movej_client_->async_send_goal(goal, send_goal_options);

    if (!wait) {
        return true;  // Return immediately without waiting
    }

    // Wait for goal to be accepted
    if (goal_handle_future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for MoveJ goal acceptance");
        return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "MoveJ goal rejected");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "MoveJ goal accepted, waiting for result...");

    // Wait for result
    auto result_future = movej_client_->async_get_result(goal_handle);
    if (result_future.wait_for(60s) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for MoveJ result");
        return false;
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "✅ MoveJ succeeded: %s", result.result->message.c_str());
        return result.result->success;
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ MoveJ failed with code: %d", static_cast<int>(result.code));
        return false;
    }
}

bool URControlClient::moveL(const std::array<double, 16>& tmatrix, double velocity, bool wait) {
    if (!movel_client_->wait_for_action_server(5s)) {
        RCLCPP_ERROR(this->get_logger(), "MoveL action server not available");
        return false;
    }

    // Create goal
    auto goal           = ur_motion::action::MoveL::Goal();
    goal.target_tmatrix = tmatrix;
    goal.velocity       = velocity;

    RCLCPP_INFO(this->get_logger(), "Sending MoveL goal: velocity=%.2f", velocity);

    // Send goal
    auto send_goal_options = rclcpp_action::Client<ur_motion::action::MoveL>::SendGoalOptions();

    // Feedback callback
    send_goal_options.feedback_callback =
        [this](auto, const std::shared_ptr<const ur_motion::action::MoveL::Feedback> feedback) {
            RCLCPP_INFO(this->get_logger(), "MoveL feedback: progress=%.2f", feedback->progress);
        };

    auto goal_handle_future = movel_client_->async_send_goal(goal, send_goal_options);

    if (!wait) {
        return true;  // Return immediately without waiting
    }

    // Wait for goal to be accepted
    if (goal_handle_future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for MoveL goal acceptance");
        return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "MoveL goal rejected");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "MoveL goal accepted, waiting for result...");

    // Wait for result
    auto result_future = movel_client_->async_get_result(goal_handle);
    if (result_future.wait_for(60s) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for MoveL result");
        return false;
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "✅ MoveL succeeded: %s", result.result->message.c_str());
        return result.result->success;
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ MoveL failed with code: %d", static_cast<int>(result.code));
        return false;
    }
}

// ============================================================
// Speed Control
// ============================================================

bool URControlClient::setSpeedSlider(double fraction) {
    if (!speed_slider_client_) {
        RCLCPP_ERROR(this->get_logger(), "Speed slider service client not initialized");
        return false;
    }

    if (fraction < 0.01 || fraction > 1.0) {
        RCLCPP_WARN(this->get_logger(), "Speed slider fraction must be in range [0.01, 1.0], got %.2f", fraction);
        return false;
    }

    if (!speed_slider_client_->wait_for_service(1s)) {
        RCLCPP_WARN(this->get_logger(), "Speed slider service not available");
        return false;
    }

    auto request                   = std::make_shared<ur_msgs::srv::SetSpeedSliderFraction::Request>();
    request->speed_slider_fraction = fraction;

    auto future = speed_slider_client_->async_send_request(request);

    if (future.wait_for(2s) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for speed slider service response");
        return false;
    }

    auto response = future.get();
    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "✅ Speed slider set to %.1f%%", fraction * 100.0);
    } else {
        RCLCPP_WARN(this->get_logger(), "❌ Failed to set speed slider");
    }

    return response->success;
}

double URControlClient::getSpeedScaling() const {
    return speed_scaling_;
}

// ============================================================
// I/O Control
// ============================================================

bool URControlClient::setDigitalOut(int pin, bool value) {
    if (!set_io_client_) {
        RCLCPP_ERROR(this->get_logger(), "I/O service client not initialized");
        return false;
    }

    if (pin < 0 || pin > 17) {
        RCLCPP_WARN(this->get_logger(), "Invalid pin number: %d (must be 0-17)", pin);
        return false;
    }

    if (!set_io_client_->wait_for_service(1s)) {
        RCLCPP_WARN(this->get_logger(), "I/O service not available");
        return false;
    }

    auto request   = std::make_shared<ur_msgs::srv::SetIO::Request>();
    request->fun   = 1;  // Set digital output
    request->pin   = pin;
    request->state = value ? 1.0 : 0.0;

    auto future = set_io_client_->async_send_request(request);

    if (future.wait_for(2s) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for I/O service response");
        return false;
    }

    auto response = future.get();
    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "✅ Digital output pin %d set to %s", pin, value ? "HIGH" : "LOW");
    } else {
        RCLCPP_WARN(this->get_logger(), "❌ Failed to set digital output pin %d", pin);
    }

    return response->success;
}

bool URControlClient::getDigitalIn(int pin) const {
    if (pin < 0 || pin >= 18) {
        return false;
    }
    return digital_in_states_[pin];
}

bool URControlClient::getDigitalOut(int pin) const {
    if (pin < 0 || pin >= 18) {
        return false;
    }
    return digital_out_states_[pin];
}

// ============================================================
// State Monitoring
// ============================================================

bool URControlClient::isConnected() const {
    return connected_;
}

bool URControlClient::getJointPositions(std::vector<double>& joints) const {
    if (!latest_joint_state_ || !connected_) {
        return false;
    }

    // Check if data is recent (within 1 second)
    rclcpp::Duration time_since_update = this->now() - last_joint_state_time_;
    if (time_since_update.seconds() > 1.0) {
        return false;
    }

    if (latest_joint_state_->position.size() >= 6) {
        joints.clear();
        joints.reserve(6);
        for (size_t i = 0; i < 6; ++i) {
            joints.push_back(latest_joint_state_->position[i]);
        }
        return true;
    }

    return false;
}

// ============================================================
// Callbacks
// ============================================================

void URControlClient::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!msg) {
        return;
    }

    latest_joint_state_    = msg;
    last_joint_state_time_ = this->now();

    bool was_connected = connected_;
    connected_         = true;

    if (!was_connected) {
        RCLCPP_INFO(this->get_logger(), "✅ Robot connected! Received joint states (joint count: %zu)", msg->name.size());
    }
}

void URControlClient::speedScalingCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    if (!msg) {
        return;
    }

    speed_scaling_ = msg->data / 100.0;  // Convert from percentage to fraction

    static bool first_log = true;
    if (first_log) {
        first_log = false;
        RCLCPP_INFO(this->get_logger(), "⚡ Speed scaling updates received: %.1f%%", msg->data);
    }
}

void URControlClient::ioStatesCallback(const ur_msgs::msg::IOStates::SharedPtr msg) {
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

    static bool first_log = true;
    if (first_log) {
        first_log = false;
        RCLCPP_INFO(this->get_logger(), "🔌 I/O states received (DI: %zu, DO: %zu)",
                    msg->digital_in_states.size(), msg->digital_out_states.size());
    }
}
