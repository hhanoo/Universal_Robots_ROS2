#include "ur_motion/moveit_backend.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <algorithm>

#include "ur_motion/transform_utils.hpp"

namespace ur_motion {

MoveItBackend::MoveItBackend(rclcpp::Node::SharedPtr node)
    : node_(node) {
    // Get planning group name from parameter or try to find it automatically
    planning_group_name_ = node_->declare_parameter<std::string>("planning_group", "");

    if (planning_group_name_.empty()) {
        // Try to find planning group automatically
        if (!initializeMoveGroup()) {
            throw std::runtime_error("Failed to initialize MoveIt: Could not find planning group");
        }
    } else {
        // Use provided planning group name
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_name_);
        move_group_->setPlanningTime(5.0);
        move_group_->setNumPlanningAttempts(5);
    }
}

bool MoveItBackend::initializeMoveGroup() {
    try {
        // 1) Load robot model to get available planning groups --------------------
        robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");
        auto                                 robot_model = robot_model_loader.getModel();

        if (!robot_model) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load robot model");
            return false;
        }

        // 2) Get available planning groups --------------------
        const auto& group_names = robot_model->getJointModelGroupNames();

        if (group_names.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "No planning groups found in robot model");
            return false;
        }

        // 3) Try common group names first --------------------
        std::vector<std::string> preferred_names = {"ur10e_manipulator", "ur10_manipulator", "ur5e_manipulator",
                                                    "ur5_manipulator", "ur3e_manipulator", "ur3_manipulator",
                                                    "manipulator"};

        for (const auto& preferred : preferred_names) {
            if (std::find(group_names.begin(), group_names.end(), preferred) != group_names.end()) {
                planning_group_name_ = preferred;
                RCLCPP_INFO(node_->get_logger(), "Using planning group: %s", planning_group_name_.c_str());
                break;
            }
        }

        // 4) If no preferred name found, use the first available group --------------------
        if (planning_group_name_.empty()) {
            planning_group_name_ = group_names[0];
            RCLCPP_WARN(node_->get_logger(), "No preferred planning group found. Using first available: %s",
                        planning_group_name_.c_str());
        }

        // 5) Initialize MoveGroupInterface --------------------
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_name_);
        move_group_->setPlanningTime(5.0);       // planning max time
        move_group_->setNumPlanningAttempts(5);  // planning max attempts

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize MoveGroup: %s", e.what());
        return false;
    }
}

MotionResult MoveItBackend::moveL(const std::array<double, 16>& T, double vel) {
    if (!move_group_) {
        return {false, "MoveGroupInterface not initialized"};
    }

    // 1) Velocity scaling (속도 퍼센트 → MoveIt 스케일)
    // - velocity scaling is directly set from user input (vel) -> 속도 스케일링
    // - acceleration scaling is coupled to velocity to improve responsiveness -> 가속도 스케일링
    double v   = std::clamp(vel, 0.01, 1.0);  // clamp velocity (속도 제한)
    double acc = std::clamp(vel, 0.01, 1.0);  // clamp acceleration (가속도 제한)
    move_group_->setMaxVelocityScalingFactor(v);
    move_group_->setMaxAccelerationScalingFactor(acc);

    // 2) T-matrix → Pose (변환)
    geometry_msgs::msg::Pose target_pose = tmatrixToPose(T);

    // 3) Cartesian waypoints (직선 경로 포인트)
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    // 4) Compute Cartesian path (MoveIt Cartesian Path)
    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction = move_group_->computeCartesianPath(
        waypoints,
        0.005,  // eef_step = 5mm (직선 분해 간격)
        0.0,    // jump_threshold (관절 점프 제한)
        trajectory);

    if (fraction < 0.999) {  // planning success rate
        return {false, "MoveIt Cartesian path planning failed"};
    }

    // 5) Recompute trajectory timing using iterative time parameterization
    // computeCartesianPath는 기본 시간을 사용하므로, 속도/가속도 제한을 고려하여 시간을 재계산
    // iterative_time_parameterization은 각 waypoint 간의 속도와 가속도를 최적화하여 부드러운 움직임 생성
    try {
        // Convert trajectory message to MoveIt RobotTrajectory
        robot_trajectory::RobotTrajectory robot_trajectory(
            move_group_->getRobotModel(), planning_group_name_);
        robot_trajectory.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);

        // Create time parameterization with velocity/acceleration limits
        trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;

        // Apply velocity and acceleration scaling factors
        // computeTimeStamps will optimize the trajectory timing based on joint limits and scaling factors
        bool success = time_parameterization.computeTimeStamps(robot_trajectory, v, acc);

        if (!success) {
            RCLCPP_WARN(node_->get_logger(), "Time parameterization failed, using original trajectory");
        } else {
            // Convert back to trajectory message
            robot_trajectory.getRobotTrajectoryMsg(trajectory);
            RCLCPP_DEBUG(node_->get_logger(), "Trajectory time recomputed with velocity: %.2f, acceleration: %.2f", v, acc);
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Failed to recompute trajectory timing: %s", e.what());
        // Continue with original trajectory if time parameterization fails
    }

    // 6) Execute trajectory (FollowJointTrajectory로 자동 전달)
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    auto result = move_group_->execute(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        return {false, "MoveIt execution failed"};
    }

    return {true, "MoveL done (MoveIt Cartesian)"};
}

}  // namespace ur_motion
