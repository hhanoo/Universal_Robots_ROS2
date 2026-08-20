#include "ur_motion/moveit_backend.hpp"

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <algorithm>
#include <chrono>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ur_motion/transform_utils.hpp"

namespace ur_motion {

MoveItBackend::MoveItBackend(rclcpp::Node::SharedPtr node)
    : node_(node) {
    // Initialize TF2 buffer and listener for frame transformations
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Planning group from parameter or auto-detection
    planning_group_name_ = node_->declare_parameter<std::string>("planning_group", "");

    if (planning_group_name_.empty()) {
        // Try to find planning group automatically
        if (!initializeMoveGroup()) {
            throw std::runtime_error("Failed to initialize MoveIt: Could not find planning group");
        }
    } else {
        // Use provided planning group name
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_name_);
        applyPlannerSettings();
    }

    // Pre-warms the state monitor for the first motion command
    move_group_->startStateMonitor();
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
        std::vector<std::string> preferred_names = {"ur_manipulator", "ur10e_manipulator", "ur10_manipulator",
                                                    "ur5e_manipulator", "ur5_manipulator", "ur3e_manipulator",
                                                    "ur3_manipulator", "manipulator"};

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
        applyPlannerSettings();

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize MoveGroup: %s", e.what());
        return false;
    }
}

void MoveItBackend::applyPlannerSettings() {
    move_group_->setPlanningTime(PLANNING_TIME);
    move_group_->setNumPlanningAttempts(PLANNING_ATTEMPTS);
}

MotionResult MoveItBackend::moveJ(const std::vector<double>& joints, double vel) {
    if (!move_group_) {
        return {false, "MoveGroupInterface not initialized"};
    }

    // 0) Ensure the state monitor is running (started in the constructor, returns at once)
    try {
        move_group_->startStateMonitor();
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Failed to start state monitor: %s", e.what());
    }

    // 1) Velocity scaling (acceleration follows velocity)
    double v   = std::clamp(vel, 0.01, 1.0);  // clamp velocity (속도 제한)
    double acc = std::clamp(vel, 0.01, 1.0);  // clamp acceleration (가속도 제한)
    move_group_->setMaxVelocityScalingFactor(v);
    move_group_->setMaxAccelerationScalingFactor(acc);

    // 2) Set target joint values (rejected when out of joint limits)
    if (!move_group_->setJointValueTarget(joints)) {
        return {false, "Target joint values are out of bounds"};
    }

    // 3) Plan in joint space (OMPL)
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const                                           ok = static_cast<bool>(move_group_->plan(plan));

    if (!ok) {
        RCLCPP_ERROR(node_->get_logger(), "MoveJ planning failed");
        return {false, "MoveJ planning failed"};
    }

    RCLCPP_INFO(node_->get_logger(), "MoveJ planning succeeded. Trajectory has %zu points.",
                plan.trajectory_.joint_trajectory.points.size());

    // 4) Execute (MoveIt forwards it to FollowJointTrajectory)
    auto result = move_group_->execute(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "MoveJ execution failed");
        return {false, "MoveJ execution failed"};
    }

    return {true, "MoveJ done (MoveIt Joint Space)"};
}

MotionResult MoveItBackend::moveL(const std::array<double, 16>& T, double vel) {
    if (!move_group_) {
        return {false, "MoveGroupInterface not initialized"};
    }

    // 0) Ensure the state monitor is running (started in the constructor, returns at once)
    try {
        move_group_->startStateMonitor();
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Failed to start state monitor: %s", e.what());
    }

    // 1) Velocity scaling (acceleration follows velocity)
    double v   = std::clamp(vel, 0.01, 1.0);  // clamp velocity (속도 제한)
    double acc = std::clamp(vel, 0.01, 1.0);  // clamp acceleration (가속도 제한)
    move_group_->setMaxVelocityScalingFactor(v);
    move_group_->setMaxAccelerationScalingFactor(acc);

    // 2) T-matrix -> Pose in the MoveIt planning frame
    geometry_msgs::msg::Pose target_pose_raw = tmatrixToPose(T);
    geometry_msgs::msg::Pose target_pose     = transformPoseToMoveItFrame(target_pose_raw);

    // 3) Compute a straight-line Cartesian path
    move_group_->setStartStateToCurrentState();

    moveit_msgs::msg::RobotTrajectory trajectory;

    const double jump_threshold = 5.0;

    double fraction = 0.0;
    fraction        = move_group_->computeCartesianPath(
        {target_pose},       // waypoints (target pose only)
        CARTESIAN_EEF_STEP,  // straight-line resolution
        jump_threshold,      // joint jump limit between waypoints
        trajectory);

    RCLCPP_INFO(node_->get_logger(), "Cartesian path planning fraction: %.3f (required: >= 0.999)", fraction);

    // 4) Check if Cartesian path planning succeeded
    if (fraction < 0.999) {
        RCLCPP_ERROR(node_->get_logger(), "Cartesian path planning failed (fraction: %.3f). Required: >= 0.999. Aborting moveL.", fraction);
        return {false,
                "Cartesian path planning failed: Could not generate a straight-line path to target pose. "
                "The target may be unreachable via a straight line, in collision, or exceed joint limits."};
    }

    // 5) Recompute timing (computeCartesianPath ignores the scaling factors)
    try {
        // Step 5-1: Convert trajectory message to MoveIt RobotTrajectory object
        robot_trajectory::RobotTrajectory robot_trajectory(move_group_->getRobotModel(), planning_group_name_);
        robot_trajectory.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);

        // Step 5-2: Create time parameterization with velocity/acceleration limits
        trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;

        // Step 5-3: Apply velocity and acceleration scaling factors
        bool success = time_parameterization.computeTimeStamps(robot_trajectory, v, acc);

        if (!success) {
            RCLCPP_WARN(node_->get_logger(), "Time parameterization failed, using original trajectory");
        } else {
            robot_trajectory.getRobotTrajectoryMsg(trajectory);
            RCLCPP_INFO(node_->get_logger(), "Trajectory time recomputed with velocity: %.2f, acceleration: %.2f", v, acc);
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Failed to recompute trajectory timing: %s", e.what());
        // Fall through with the original timing
    }

    // 6) Execute (MoveIt forwards it to FollowJointTrajectory)
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    auto result = move_group_->execute(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        return {false, "MoveIt execution failed"};
    }

    return {true, "MoveL done (MoveIt Cartesian)"};
}

MotionResult MoveItBackend::moveL(const std::vector<std::array<double, 16>>& via_T,
                                  const std::vector<double>&                 via_r,
                                  const std::vector<double>&                 via_vel,
                                  const std::array<double, 16>&              target_T,
                                  double                                     target_vel) {
    if (!move_group_) {
        return {false, "MoveGroupInterface not initialized"};
    }
    if (via_T.empty()) {
        return moveL(target_T, target_vel);
    }
    if (via_r.size() != via_T.size() || via_vel.size() != via_T.size()) {
        return {false, "via_r/via_vel size mismatch with via_T"};
    }

    // 1) Lazy-create the Pilz sequence action client (absent unless the pipeline is loaded)
    if (!seq_client_) {
        seq_client_ = rclcpp_action::create_client<MoveGroupSequence>(node_, "sequence_move_group");
    }
    if (!seq_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_WARN(node_->get_logger(), "sequence_move_group unavailable - falling back to point-by-point MoveL");
        return moveLPointByPoint(via_T, via_vel, target_T, target_vel);
    }

    // 2) One LIN item per via; the target closes the run
    MoveGroupSequence::Goal goal;
    for (size_t i = 0; i < via_T.size(); ++i) {
        if (via_r[i] <= 0.0) {
            return {false, "via blend radius must be > 0"};
        }
        goal.request.items.push_back(makeLinItem(via_T[i], via_vel[i], via_r[i]));
    }
    goal.request.items.push_back(makeLinItem(target_T, target_vel, 0.0));
    goal.planning_options.plan_only = false;

    RCLCPP_INFO(node_->get_logger(), "Blended MoveL: %zu segments (Pilz LIN sequence)",
                goal.request.items.size());

    // 3) The result only arrives after execution finishes
    auto goal_future = seq_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        return {false, "sequence goal was not accepted in time"};
    }
    auto handle = goal_future.get();
    if (!handle) {
        return {false, "sequence goal rejected"};
    }
    seq_goal_handle_ = handle;

    auto result_future = seq_client_->async_get_result(handle);
    // 120s budget per segment, like a single MoveL
    const auto result_timeout = std::chrono::seconds(120 * static_cast<long>(goal.request.items.size()));
    if (result_future.wait_for(result_timeout) != std::future_status::ready) {
        seq_client_->async_cancel_goal(handle);
        seq_goal_handle_.reset();
        return {false, "sequence execution timed out"};
    }
    seq_goal_handle_.reset();

    auto wrapped = result_future.get();
    const int ec = wrapped.result ? wrapped.result->response.error_code.val : 0;
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
        ec == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        return {true, "MoveL done (Pilz blended sequence)"};
    }

    // Planning-stage failures abort before any motion - safe to degrade to the old behavior
    const bool planning_failed = (ec == moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED ||
                                  ec == moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN ||
                                  ec == moveit_msgs::msg::MoveItErrorCodes::FAILURE);
    if (wrapped.code != rclcpp_action::ResultCode::CANCELED && planning_failed) {
        RCLCPP_WARN(node_->get_logger(),
                    "Pilz sequence planning failed (code %d) - falling back to point-by-point MoveL", ec);
        return moveLPointByPoint(via_T, via_vel, target_T, target_vel);
    }
    return {false, "Pilz sequence failed (MoveItErrorCode " + std::to_string(ec) + ")"};
}

MotionResult MoveItBackend::moveLPointByPoint(const std::vector<std::array<double, 16>>& via_T,
                                              const std::vector<double>&                 via_vel,
                                              const std::array<double, 16>&              target_T,
                                              double                                     target_vel) {
    for (size_t i = 0; i < via_T.size(); ++i) {
        MotionResult r = moveL(via_T[i], via_vel[i]);
        if (!r.success) {
            return r;
        }
    }
    return moveL(target_T, target_vel);
}

moveit_msgs::msg::MotionSequenceItem MoveItBackend::makeLinItem(
    const std::array<double, 16>& T, double vel, double radius_m) {
    // T is base -> tool0_controller (same as single-target moveL)
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = move_group_->getPlanningFrame();
    ps.pose            = transformPoseToMoveItFrame(tmatrixToPose(T));

    moveit_msgs::msg::MotionSequenceItem item;
    item.blend_radius = radius_m;

    auto& req                           = item.req;
    req.group_name                      = planning_group_name_;
    req.pipeline_id                     = "pilz_industrial_motion_planner";
    req.planner_id                      = "LIN";
    req.max_velocity_scaling_factor     = std::clamp(vel, 0.01, 1.0);
    req.max_acceleration_scaling_factor = std::clamp(vel, 0.01, 1.0);
    req.allowed_planning_time           = 5.0;
    req.goal_constraints.push_back(
        kinematic_constraints::constructGoalConstraints(move_group_->getEndEffectorLink(), ps));
    return item;
}

geometry_msgs::msg::Pose MoveItBackend::transformPoseToMoveItFrame(const geometry_msgs::msg::Pose& pose) {
    // Input "base -> tool0_controller" equals "base_link_inertia -> tool0"
    if (!move_group_) {
        return pose;
    }

    std::string planning_frame = move_group_->getPlanningFrame();

    try {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "base_link_inertia";  // T-matrix is in base_link_inertia frame
        pose_stamped.header.stamp    = node_->now();
        pose_stamped.pose            = pose;

        geometry_msgs::msg::PoseStamped transformed_pose;
        tf_buffer_->transform(pose_stamped, transformed_pose, planning_frame, tf2::Duration(std::chrono::seconds(1)));

        RCLCPP_DEBUG(node_->get_logger(), "Transformed pose from base_link_inertia to %s", planning_frame.c_str());
        return transformed_pose.pose;

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform from base_link_inertia to %s: %s. Using original pose.", planning_frame.c_str(), ex.what());
        return pose;
    }
}

void MoveItBackend::moveCancel() {
    if (!move_group_) {
        RCLCPP_WARN(node_->get_logger(), "MoveItBackend::moveCancel() called but move_group_ is null");
        return;
    }

    RCLCPP_WARN(node_->get_logger(), "MoveItBackend::moveCancel() - stopping MoveGroup execution");

    // Blended runs execute under the sequence action
    if (seq_goal_handle_ && seq_client_) {
        seq_client_->async_cancel_goal(seq_goal_handle_);
    }
    move_group_->stop();
}

}  // namespace ur_motion
