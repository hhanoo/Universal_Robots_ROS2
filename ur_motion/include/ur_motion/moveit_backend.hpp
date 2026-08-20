#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <memory>
#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "ur_motion/motion_backend.hpp"

namespace ur_motion {

class MoveItBackend : public MoveJBackend, public MoveLBackend {
   public:
    explicit MoveItBackend(rclcpp::Node::SharedPtr node);

    // Joint space move using MoveIt (MoveIt 기반 관절 공간 이동)
    MotionResult moveJ(const std::vector<double>& joints, double vel) override;

    // Cartesian linear move using MoveIt (MoveIt 기반 직선 이동)
    MotionResult moveL(const std::array<double, 16>& T, double vel) override;

    // Blended Cartesian run using the Pilz sequence action (Pilz 시퀀스 기반 블렌드 직선 이동)
    MotionResult moveL(
        const std::vector<std::array<double, 16>>& via_T,
        const std::vector<double>&                 via_r,
        const std::vector<double>&                 via_vel,
        const std::array<double, 16>& target_T, double target_vel) override;

    // Cancel motion
    void moveCancel() override;

   private:
    // Planner tuning (latency between consecutive commands vs. path quality)
    static constexpr int    PLANNING_ATTEMPTS  = 3;      // OMPL waits for this many solutions, then hybridizes them
    static constexpr double PLANNING_TIME      = 2.0;    // planner time budget [s]
    static constexpr double CARTESIAN_EEF_STEP = 0.005;  // moveL waypoint spacing [m]

    rclcpp::Node::SharedPtr node_;

    // Created only after the planning group name is known
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    // Planning group name
    std::string planning_group_name_;

    // TF2 buffer for frame transformations (프레임 변환을 위한 TF2 버퍼)
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Auto-detect a planning group and create the interface
    bool initializeMoveGroup();

    // Planner tuning shared by both creation paths
    void applyPlannerSettings();

    // Transform pose from tool0_controller frame to tool0 frame
    geometry_msgs::msg::Pose transformPoseToMoveItFrame(const geometry_msgs::msg::Pose& pose);

    // ----- Pilz blended sequence (only touched when vias are present) -----
    using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;

    // One LIN segment ending at T; radius 0 = exact stop
    moveit_msgs::msg::MotionSequenceItem makeLinItem(
        const std::array<double, 16>& T, double vel, double radius_m);

    rclcpp_action::Client<MoveGroupSequence>::SharedPtr           seq_client_;
    rclcpp_action::ClientGoalHandle<MoveGroupSequence>::SharedPtr seq_goal_handle_;
};

}  // namespace ur_motion
