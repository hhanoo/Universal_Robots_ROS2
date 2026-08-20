#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>

#include "ur_motion/action/move_j.hpp"
#include "ur_motion/action/move_l.hpp"
#include "ur_motion/motion_backend.hpp"
#include "ur_motion/moveit_backend.hpp"
#include "ur_motion/transform_utils.hpp"

using namespace std::chrono_literals;

namespace ur_motion {

class MotionActionServer : public rclcpp::Node {
   public:
    // Motion wait result enum
    enum class MotionWaitResult {
        COMPLETED,  // Motion completed successfully
        CANCELED,   // Motion canceled by user
        ABORTED     // Motion aborted due to error
    };

    // Motion settling guard — MoveIt execute() already blocks until the controller reports done
    static constexpr double                    VELOCITY_THRESHOLD = 0.01;  // rad/s - threshold for considering robot stopped
    static constexpr double                    STABLE_DURATION    = 0.05;  // seconds - duration robot must be stable
    static constexpr std::chrono::milliseconds POLL_INTERVAL{5};           // polling interval for motion check

    // Action type aliases
    using MoveJ           = ur_motion::action::MoveJ;
    using MoveL           = ur_motion::action::MoveL;
    using GoalHandleMoveJ = rclcpp_action::ServerGoalHandle<MoveJ>;
    using GoalHandleMoveL = rclcpp_action::ServerGoalHandle<MoveL>;

    MotionActionServer()
        : Node("motion_action_server") {
        // Subscribe to joint states for motion completion detection
        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(&MotionActionServer::jointStateCallback, this, std::placeholders::_1));

        // Create MoveJ action server
        movej_server_ = rclcpp_action::create_server<MoveJ>(
            this,
            "move_j",
            std::bind(&MotionActionServer::handleMoveJGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionActionServer::handleMoveJCancel, this, std::placeholders::_1),
            std::bind(&MotionActionServer::handleMoveJAccept, this, std::placeholders::_1));

        // Create MoveL action server
        movel_server_ = rclcpp_action::create_server<MoveL>(
            this,
            "move_l",
            std::bind(&MotionActionServer::handleMoveLGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionActionServer::handleMoveLCancel, this, std::placeholders::_1),
            std::bind(&MotionActionServer::handleMoveLAccept, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "UR Action Server started");
    }

    // Initialize backends (must be called after object is managed by shared_ptr)
    void initBackends() {
        // Setup MoveIt parameters before initializing MoveItBackend
        setupMoveItParameters();

        // Initialize MoveItBackend for both MoveJ and MoveL
        try {
            motion_backend_ = std::make_shared<MoveItBackend>(shared_from_this());
            RCLCPP_INFO(get_logger(), "MoveItBackend initialized for both MoveJ and MoveL");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize MoveItBackend: %s", e.what());
            RCLCPP_WARN(get_logger(), "MoveJ and MoveL will not be available. Please ensure robot_description and robot_description_semantic are set.");
            motion_backend_ = nullptr;
        }
    }

    // Setup MoveIt required parameters
    void setupMoveItParameters() {
        // Declared empty when the launch file did not set them
        if (!has_parameter("robot_description")) {
            declare_parameter<std::string>("robot_description", "");
        }
        if (!has_parameter("robot_description_semantic")) {
            declare_parameter<std::string>("robot_description_semantic", "");
        }
    }

   private:
    // MoveIt backend for both MoveJ and MoveL
    std::shared_ptr<MoveItBackend> motion_backend_;

    // Flag to request cancel motion
    std::atomic_bool cancel_requested_{false};

    // Action servers
    rclcpp_action::Server<MoveJ>::SharedPtr movej_server_;
    rclcpp_action::Server<MoveL>::SharedPtr movel_server_;

    // Joint state subscription and storage
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    sensor_msgs::msg::JointState                                  last_joint_state_;
    std::mutex                                                    joint_state_mutex_;

    // Store latest joint state thread-safely
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        last_joint_state_ = *msg;
    }

    // Check if all joint velocities are below threshold
    bool isRobotStopped() {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);

        if (last_joint_state_.velocity.empty())
            return false;

        for (double v : last_joint_state_.velocity) {
            if (std::abs(v) > VELOCITY_THRESHOLD) {
                return false;
            }
        }
        return true;
    }

    // Wait until robot actually stops
    MotionWaitResult waitUntilMotionDone() {
        auto stable_start = now();

        while (rclcpp::ok()) {
            // 1. cancel requested check (highest priority)
            if (cancel_requested_.load()) {
                RCLCPP_WARN(get_logger(), "Motion canceled while waiting");
                return MotionWaitResult::CANCELED;
            }
            // 2. normal completion check
            if (isRobotStopped()) {
                if ((now() - stable_start).seconds() > STABLE_DURATION) {
                    return MotionWaitResult::COMPLETED;
                }
            } else {
                stable_start = now();
            }
            rclcpp::sleep_for(POLL_INTERVAL);
        }

        // 3. node shutdown or system error
        RCLCPP_ERROR(get_logger(), "Motion aborted (rclcpp shutdown)");
        return MotionWaitResult::ABORTED;
    }

    // ================= MoveJ =================
    // Handle MoveJ goal (Accept only 6-DOF joint targets)
    rclcpp_action::GoalResponse handleMoveJGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveJ::Goal> goal) {
        return goal->joints.size() == 6
                   ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                   : rclcpp_action::GoalResponse::REJECT;
    }

    // Handle MoveJ cancel (Cancel is accepted but stop command is not yet sent)
    rclcpp_action::CancelResponse handleMoveJCancel(const std::shared_ptr<GoalHandleMoveJ>) {
        RCLCPP_INFO(get_logger(), "MoveJ cancel requested");

        // Set cancel requested flag
        cancel_requested_.store(true);

        // Cancel motion if backend is initialized
        if (motion_backend_) {
            motion_backend_->moveCancel();
        }

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Handle MoveJ accept (Run execution in detached thread)
    void handleMoveJAccept(const std::shared_ptr<GoalHandleMoveJ> goal_handle) {
        std::thread(&MotionActionServer::executeMoveJ, this, goal_handle).detach();
    }

    // Execute MoveJ command
    void executeMoveJ(const std::shared_ptr<GoalHandleMoveJ> goal_handle) {
        // Reset cancel requested flag
        cancel_requested_.store(false);

        // Check if backend is initialized
        if (!motion_backend_) {
            abortGoal(goal_handle, "MoveItBackend not initialized");
            return;
        }

        // Get goal
        const auto goal = goal_handle->get_goal();

        // Send MoveJ command using MoveItBackend
        auto motion_result = motion_backend_->moveJ(goal->joints, goal->velocity);

        // Handle motion result
        if (!motion_result.success) {
            abortGoal(goal_handle, "MoveJ failed: " + motion_result.message);
            return;
        }

        // Wait for motion to complete or be canceled or aborted
        switch (waitUntilMotionDone()) {
            case MotionWaitResult::COMPLETED:
                succeedGoal(goal_handle, "MoveJ done");
                break;

            case MotionWaitResult::CANCELED: {
                auto result     = std::make_shared<MoveJ::Result>();
                result->success = false;
                result->message = "MoveJ canceled";
                goal_handle->canceled(result);
                break;
            }

            case MotionWaitResult::ABORTED:
                abortGoal(goal_handle, "MoveJ aborted");
                break;
        }
    }

    // ================= MoveL =================
    // Handle MoveL goal (Always accept MoveL goals)
    rclcpp_action::GoalResponse handleMoveLGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveL::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle MoveL cancel (Cancel is accepted but stop command is not yet sent)
    rclcpp_action::CancelResponse handleMoveLCancel(const std::shared_ptr<GoalHandleMoveL>) {
        RCLCPP_INFO(get_logger(), "MoveL cancel requested");

        // Set cancel requested flag
        cancel_requested_.store(true);

        // Cancel motion if backend is initialized
        if (motion_backend_) {
            motion_backend_->moveCancel();
        }

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Handle MoveL accept (Run execution in detached thread)
    void handleMoveLAccept(const std::shared_ptr<GoalHandleMoveL> goal_handle) {
        std::thread(&MotionActionServer::executeMoveL, this, goal_handle).detach();
    }

    // Execute MoveL command
    void executeMoveL(const std::shared_ptr<GoalHandleMoveL> goal_handle) {
        // Reset cancel requested flag
        cancel_requested_.store(false);

        // Check if backend is initialized
        if (!motion_backend_) {
            abortGoal(goal_handle, "MoveItBackend not initialized");
            return;
        }

        // Get goal
        const auto goal = goal_handle->get_goal();

        // Send MoveL command using MoveItBackend (blended run when vias are present)
        MotionResult motion_result;
        if (goal->via_tmatrix.empty()) {
            motion_result = motion_backend_->moveL(goal->target_tmatrix, goal->velocity);
        } else if (goal->via_tmatrix.size() % 16 != 0 ||
                   goal->via_r.size() != goal->via_tmatrix.size() / 16 ||
                   goal->via_velocity.size() != goal->via_r.size()) {
            abortGoal(goal_handle, "MoveL via arrays malformed (N x 16 / N / N expected)");
            return;
        } else {
            const size_t                        n = goal->via_tmatrix.size() / 16;
            std::vector<std::array<double, 16>> vias(n);
            for (size_t i = 0; i < n; ++i)
                std::copy_n(goal->via_tmatrix.begin() + i * 16, 16, vias[i].begin());
            std::vector<double> via_r(goal->via_r.begin(), goal->via_r.end());
            std::vector<double> via_vel(goal->via_velocity.begin(), goal->via_velocity.end());
            motion_result = motion_backend_->moveL(vias, via_r, via_vel,
                                                   goal->target_tmatrix, goal->velocity);
        }

        // Handle motion result
        if (!motion_result.success) {
            abortGoal(goal_handle, "MoveL failed: " + motion_result.message);
            return;
        }

        // Wait for motion to complete or be canceled or aborted
        switch (waitUntilMotionDone()) {
            case MotionWaitResult::COMPLETED:
                succeedGoal(goal_handle, "MoveL done");
                break;

            case MotionWaitResult::CANCELED: {
                auto result     = std::make_shared<MoveL::Result>();
                result->success = false;
                result->message = "MoveL canceled";
                goal_handle->canceled(result);
                break;
            }

            case MotionWaitResult::ABORTED:
                abortGoal(goal_handle, "MoveL aborted");
                break;
        }
    }

    // Helper: Abort MoveJ goal with error message
    void abortGoal(const std::shared_ptr<GoalHandleMoveJ> goal_handle, const std::string& message) {
        auto result     = std::make_shared<MoveJ::Result>();
        result->success = false;
        result->message = message;
        RCLCPP_ERROR(get_logger(), "%s", message.c_str());
        goal_handle->abort(result);
    }

    // Helper: Abort MoveL goal with error message
    void abortGoal(const std::shared_ptr<GoalHandleMoveL> goal_handle, const std::string& message) {
        auto result     = std::make_shared<MoveL::Result>();
        result->success = false;
        result->message = message;
        RCLCPP_ERROR(get_logger(), "%s", message.c_str());
        goal_handle->abort(result);
    }

    // Helper: Succeed MoveJ goal with success message
    void succeedGoal(const std::shared_ptr<GoalHandleMoveJ> goal_handle, const std::string& message) {
        auto result     = std::make_shared<MoveJ::Result>();
        result->success = true;
        result->message = message;
        goal_handle->succeed(result);
    }

    // Helper: Succeed MoveL goal with success message
    void succeedGoal(const std::shared_ptr<GoalHandleMoveL> goal_handle, const std::string& message) {
        auto result     = std::make_shared<MoveL::Result>();
        result->success = true;
        result->message = message;
        goal_handle->succeed(result);
    }
};

}  // namespace ur_motion

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create and spin action server node
    auto node = std::make_shared<ur_motion::MotionActionServer>();

    // Initialize backends (must be called after shared_ptr is created)
    node->initBackends();

    // Spin node
    rclcpp::spin(node);

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
