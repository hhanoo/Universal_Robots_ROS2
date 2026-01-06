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
#include "ur_motion/trajectory_backend.hpp"
#include "ur_motion/transform_utils.hpp"

using namespace std::chrono_literals;

namespace ur_motion {

class URActionServer : public rclcpp::Node {
   public:
    // Action type aliases
    using MoveJ           = ur_motion::action::MoveJ;
    using MoveL           = ur_motion::action::MoveL;
    using GoalHandleMoveJ = rclcpp_action::ServerGoalHandle<MoveJ>;
    using GoalHandleMoveL = rclcpp_action::ServerGoalHandle<MoveL>;

    URActionServer()
        : Node("ur_action_server") {
        // Subscribe to joint states for motion completion detection
        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(&URActionServer::jointStateCallback, this, std::placeholders::_1));

        // Create MoveJ action server
        movej_server_ = rclcpp_action::create_server<MoveJ>(
            this,
            "move_j",
            std::bind(&URActionServer::handleMoveJGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&URActionServer::handleMoveJCancel, this, std::placeholders::_1),
            std::bind(&URActionServer::handleMoveJAccept, this, std::placeholders::_1));

        // Create MoveL action server
        movel_server_ = rclcpp_action::create_server<MoveL>(
            this,
            "move_l",
            std::bind(&URActionServer::handleMoveLGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&URActionServer::handleMoveLCancel, this, std::placeholders::_1),
            std::bind(&URActionServer::handleMoveLAccept, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "UR Action Server started");
    }

    // Initialize backends (must be called after object is managed by shared_ptr)
    void initBackends() {
        // MoveJ uses TrajectoryBackend
        movej_backend_ = std::make_shared<TrajectoryBackend>(shared_from_this());
        RCLCPP_INFO(get_logger(), "TrajectoryBackend initialized for MoveJ");

        // Setup MoveIt parameters before initializing MoveItBackend
        setupMoveItParameters();

        // MoveL uses MoveItBackend
        try {
            movel_backend_ = std::make_shared<MoveItBackend>(shared_from_this());
            RCLCPP_INFO(get_logger(), "MoveItBackend initialized for MoveL");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize MoveItBackend: %s", e.what());
            RCLCPP_WARN(get_logger(), "MoveL will not be available. Please ensure robot_description and robot_description_semantic are set.");
            movel_backend_ = nullptr;  // Set to nullptr to indicate initialization failed
        }
    }

    // Setup MoveIt required parameters
    void setupMoveItParameters() {
        // Declare MoveIt parameters if not already declared
        // These parameters should be set by launch file or other nodes ()
        if (!has_parameter("robot_description")) {
            declare_parameter<std::string>("robot_description", "");
        }
        if (!has_parameter("robot_description_semantic")) {
            declare_parameter<std::string>("robot_description_semantic", "");
        }
    }

   private:
    // Separate backends for MoveJ and MoveL (using interface types)
    std::shared_ptr<MoveJBackend> movej_backend_;
    std::shared_ptr<MoveLBackend> movel_backend_;

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
    bool isRobotStopped(double vel_threshold = 0.01) {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);

        if (last_joint_state_.velocity.empty())
            return false;

        for (double v : last_joint_state_.velocity) {
            if (std::abs(v) > vel_threshold) {
                return false;
            }
        }
        return true;
    }

    // Wait until robot actually stops
    void waitUntilMotionDone() {
        auto stable_start = now();

        while (rclcpp::ok()) {
            if (isRobotStopped()) {
                if ((now() - stable_start).seconds() > 0.2) {
                    return;
                }
            } else {
                stable_start = now();
            }
            rclcpp::sleep_for(10ms);
        }
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
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Handle MoveJ accept (Run execution in detached thread)
    void handleMoveJAccept(const std::shared_ptr<GoalHandleMoveJ> goal_handle) {
        std::thread(&URActionServer::executeMoveJ, this, goal_handle).detach();
    }

    // Execute MoveJ command
    void executeMoveJ(const std::shared_ptr<GoalHandleMoveJ> goal_handle) {
        const auto goal = goal_handle->get_goal();

        // Check if backend is initialized
        if (!movej_backend_) {
            auto result     = std::make_shared<MoveJ::Result>();
            result->success = false;
            result->message = "MoveJBackend not initialized";
            RCLCPP_ERROR(get_logger(), "MoveJBackend not initialized");
            goal_handle->abort(result);
            return;
        }

        // Send MoveJ command using MoveJBackend
        auto motion_result = movej_backend_->moveJ(goal->joints, goal->velocity);

        // Check if motion command failed
        if (!motion_result.success) {
            auto result     = std::make_shared<MoveJ::Result>();
            result->success = false;
            result->message = motion_result.message;
            RCLCPP_ERROR(get_logger(), "MoveJ failed: %s", motion_result.message.c_str());
            goal_handle->abort(result);
            return;
        }

        // Wait until robot actually stops
        waitUntilMotionDone();

        auto result     = std::make_shared<MoveJ::Result>();
        result->success = true;
        result->message = "MoveJ done";

        goal_handle->succeed(result);
    }

    // ================= MoveL =================
    // Handle MoveL goal (Always accept MoveL goals)
    rclcpp_action::GoalResponse handleMoveLGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveL::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle MoveL cancel (Cancel is accepted but stop command is not yet sent)
    rclcpp_action::CancelResponse handleMoveLCancel(const std::shared_ptr<GoalHandleMoveL>) {
        RCLCPP_INFO(get_logger(), "MoveL cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Handle MoveL accept (Run execution in detached thread)
    void handleMoveLAccept(const std::shared_ptr<GoalHandleMoveL> goal_handle) {
        std::thread(&URActionServer::executeMoveL, this, goal_handle).detach();
    }

    // Execute MoveL command
    void executeMoveL(const std::shared_ptr<GoalHandleMoveL> goal_handle) {
        const auto goal = goal_handle->get_goal();

        // Check if backend is initialized
        if (!movel_backend_) {
            auto result     = std::make_shared<MoveL::Result>();
            result->success = false;
            result->message = "MoveLBackend not initialized";
            RCLCPP_ERROR(get_logger(), "MoveLBackend not initialized");
            goal_handle->abort(result);
            return;
        }

        // Send MoveL command using MoveLBackend
        auto motion_result = movel_backend_->moveL(goal->target_tmatrix, goal->velocity);

        // Check if motion command failed
        if (!motion_result.success) {
            auto result     = std::make_shared<MoveL::Result>();
            result->success = false;
            result->message = motion_result.message;
            RCLCPP_ERROR(get_logger(), "MoveL failed: %s", motion_result.message.c_str());
            goal_handle->abort(result);
            return;
        }

        // Wait until robot actually stops
        waitUntilMotionDone();

        auto result     = std::make_shared<MoveL::Result>();
        result->success = true;
        result->message = "MoveL done";

        goal_handle->succeed(result);
    }
};

}  // namespace ur_motion

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create and spin action server node
    auto node = std::make_shared<ur_motion::URActionServer>();

    // Initialize backends (must be called after shared_ptr is created)
    node->initBackends();

    // Spin node
    rclcpp::spin(node);

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
