#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <array>
#include <memory>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "ur_motion/motion_backend.hpp"

namespace ur_motion {

class MoveItBackend : public MoveLBackend {
   public:
    explicit MoveItBackend(rclcpp::Node::SharedPtr node);

    // Cartesian linear move using MoveIt (MoveIt 기반 직선 이동)
    MotionResult moveL(const std::array<double, 16>& T, double vel) override;

   private:
    rclcpp::Node::SharedPtr node_;

    // MoveIt interface (MoveIt 인터페이스) (planning group 이름을 자동 탐색한 뒤에 지연 생성하려는 설계)
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    // Planning group name
    std::string planning_group_name_;

    // Initialize MoveGroupInterface with correct planning group name
    bool initializeMoveGroup();
};

}  // namespace ur_motion
