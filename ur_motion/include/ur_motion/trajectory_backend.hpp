#pragma once

#include <chrono>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "ur_motion/motion_backend.hpp"

namespace ur_motion {

class TrajectoryBackend : public MoveJBackend {
   public:
    explicit TrajectoryBackend(rclcpp::Node::SharedPtr node);

    MotionResult moveJ(const std::vector<double>& joints, double vel) override;

   private:
    rclcpp::Node::SharedPtr node_;

    using FollowJT = control_msgs::action::FollowJointTrajectory;
    rclcpp_action::Client<FollowJT>::SharedPtr traj_client_;

    std::vector<std::string> joint_names_;
};

}  // namespace ur_motion
