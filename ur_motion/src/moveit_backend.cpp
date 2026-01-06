#include "ur_motion/moveit_backend.hpp"

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
    // (프레임 변환을 위한 TF2 버퍼 및 리스너 초기화)
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

    // 0) Ensure current state is updated (현재 상태 업데이트 확인)
    // MoveIt needs to know the current robot state for planning
    // (MoveIt은 경로 계획을 위해 현재 로봇 상태를 알아야 함)
    try {
        // Start state monitor to receive joint states (joint_states를 받기 위해 상태 모니터 시작)
        move_group_->startStateMonitor();
        // Give it a moment to receive joint states (joint_states를 받을 시간 제공)
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Failed to start state monitor: %s", e.what());
    }

    // 1) Velocity scaling (속도 퍼센트 → MoveIt 스케일)
    // - velocity scaling is directly set from user input (vel) -> 속도 스케일링
    // - acceleration scaling is coupled to velocity to improve responsiveness -> 가속도 스케일링
    double v   = std::clamp(vel, 0.01, 1.0);  // clamp velocity (속도 제한)
    double acc = std::clamp(vel, 0.01, 1.0);  // clamp acceleration (가속도 제한)
    move_group_->setMaxVelocityScalingFactor(v);
    move_group_->setMaxAccelerationScalingFactor(acc);

    // 2) T-matrix → Pose (변환)
    // T-matrix is "base -> tool0_controller" which equals "base_link_inertia -> tool0"
    // (T-matrix는 "base -> tool0_controller"이며, 이는 "base_link_inertia -> tool0"와 같음)
    // Step 2-1: Extract position and orientation from T-matrix (4x4 transformation matrix)
    // (T-matrix에서 위치와 회전 추출)
    geometry_msgs::msg::Pose target_pose_raw = tmatrixToPose(T);
    // → target_pose_raw: pose in base_link_inertia frame (tool0_controller link)

    // Step 2-2: Transform pose from base_link_inertia frame to MoveIt planning frame (world)
    // (base_link_inertia 프레임에서 MoveIt planning frame (world)로 pose 변환)
    // This ensures coordinate frame consistency with MoveIt's planning frame
    // (MoveIt의 planning frame과 좌표계 일관성 보장)
    geometry_msgs::msg::Pose target_pose = transformPoseToMoveItFrame(target_pose_raw);
    // → target_pose: pose in world frame (tool0 link)

    // 3) Try Cartesian path planning first (직선 경로 계획 시도)
    // If it fails, fall back to regular planning (실패 시 일반 경로 계획으로 대체)
    // Cartesian path planning: generates a straight-line path in Cartesian space
    // (Cartesian path planning: Cartesian 공간에서 직선 경로 생성)
    // - Creates waypoints along a straight line from current to target position
    // - Computes inverse kinematics (IK) for each waypoint
    // - Returns fraction (0.0-1.0) indicating how much of the path was successfully planned
    // (현재 위치에서 목표 위치까지 직선상의 waypoint 생성, 각 waypoint에서 IK 계산, 성공률 반환)
    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction = move_group_->computeCartesianPath(
        {target_pose},  // waypoints (목표 위치만 포함)
        0.001,          // eef_step = 1mm (직선 분해 간격: waypoint 간 최대 거리)
        0.0,            // jump_threshold = 0 (관절 점프 제한: 0 = 제한 없음)
        trajectory);

    RCLCPP_INFO(node_->get_logger(), "Cartesian path planning fraction: %.3f (required: >= 0.999)", fraction);

    // 4) Check if Cartesian path planning succeeded
    // (직선 경로 계획 성공 여부 확인)
    // If Cartesian path fails, return error immediately without trying regular planning
    // (직선 경로가 실패하면 regular planning을 시도하지 않고 즉시 오류 반환)
    // This ensures only straight-line (Cartesian) movements are executed
    // (직선 (Cartesian) 이동만 실행되도록 보장)
    if (fraction < 0.999) {
        RCLCPP_ERROR(node_->get_logger(), "Cartesian path planning failed (fraction: %.3f). Required: >= 0.999. Aborting moveL.", fraction);
        return {false,
                "Cartesian path planning failed: Could not generate a straight-line path to target pose. "
                "The target may be unreachable via a straight line, in collision, or exceed joint limits."};
    }

    // 5) Recompute trajectory timing using iterative time parameterization
    // computeCartesianPath는 기본 시간을 사용하므로, 속도/가속도 제한을 고려하여 시간을 재계산
    // iterative_time_parameterization은 각 waypoint 간의 속도와 가속도를 최적화하여 부드러운 움직임 생성
    // This step ensures the trajectory respects velocity/acceleration limits and creates smooth motion
    // (이 단계는 trajectory가 속도/가속도 제한을 준수하고 부드러운 움직임을 생성하도록 보장)
    try {
        // Step 5-1: Convert trajectory message to MoveIt RobotTrajectory object
        // (trajectory message를 MoveIt RobotTrajectory 객체로 변환)
        robot_trajectory::RobotTrajectory robot_trajectory(
            move_group_->getRobotModel(), planning_group_name_);
        robot_trajectory.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);

        // Step 5-2: Create time parameterization with velocity/acceleration limits
        // (속도/가속도 제한을 고려한 시간 재계산)
        trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;

        // Step 5-3: Apply velocity and acceleration scaling factors
        // computeTimeStamps will optimize the trajectory timing based on joint limits and scaling factors
        // (관절 제한과 스케일링 팩터를 기반으로 trajectory 시간 최적화)
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
    // MoveIt automatically sends the trajectory to the controller via FollowJointTrajectory action
    // (MoveIt은 자동으로 trajectory를 FollowJointTrajectory action을 통해 controller에 전송)
    // The trajectory is sent to /scaled_joint_trajectory_controller/follow_joint_trajectory
    // (trajectory는 /scaled_joint_trajectory_controller/follow_joint_trajectory로 전송됨)
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    auto result = move_group_->execute(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        return {false, "MoveIt execution failed"};
    }

    return {true, "MoveL done (MoveIt Cartesian)"};
}

geometry_msgs::msg::Pose MoveItBackend::transformPoseToMoveItFrame(const geometry_msgs::msg::Pose& pose) {
    // T-matrix is "base -> tool0_controller" which equals "base_link_inertia -> tool0"
    // MoveIt needs pose in planning frame (e.g., "world") for "tool0" link
    // Transform from "base_link_inertia" frame to planning frame
    // (T-matrix는 "base -> tool0_controller"이며, 이는 "base_link_inertia -> tool0"와 같음)
    // (MoveIt은 planning frame (예: "world")에서 "tool0" 링크의 pose 필요)
    // ("base_link_inertia" 프레임에서 planning frame으로 변환)

    if (!move_group_) {
        return pose;
    }

    std::string planning_frame = move_group_->getPlanningFrame();

    try {
        // Transform from base_link_inertia frame to planning frame
        // (base_link_inertia 프레임에서 planning frame으로 변환)
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "base_link_inertia";  // T-matrix is in base_link_inertia frame
        pose_stamped.header.stamp    = node_->now();
        pose_stamped.pose            = pose;

        geometry_msgs::msg::PoseStamped transformed_pose;
        tf_buffer_->transform(pose_stamped, transformed_pose, planning_frame, tf2::Duration(std::chrono::seconds(1)));

        RCLCPP_DEBUG(node_->get_logger(), "Transformed pose from base_link_inertia to %s", planning_frame.c_str());
        return transformed_pose.pose;

    } catch (const tf2::TransformException& ex) {
        // If transform fails, use original pose
        // (변환이 실패하면 원본 pose 사용)
        RCLCPP_WARN(node_->get_logger(), "Could not transform from base_link_inertia to %s: %s. Using original pose.", planning_frame.c_str(), ex.what());
        return pose;
    }
}

}  // namespace ur_motion
