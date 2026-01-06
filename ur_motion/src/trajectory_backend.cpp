#include "ur_motion/trajectory_backend.hpp"

#include "ur_motion/transform_utils.hpp"

namespace ur_motion {

TrajectoryBackend::TrajectoryBackend(rclcpp::Node::SharedPtr node)
    : node_(node) {
    traj_client_ = rclcpp_action::create_client<FollowJT>(
        node_,
        "/scaled_joint_trajectory_controller/follow_joint_trajectory");

    joint_names_ = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"};
}

MotionResult TrajectoryBackend::moveJ(const std::vector<double>& joints, double vel) {
    if (!traj_client_->wait_for_action_server(std::chrono::seconds(3))) {
        return {false, "Trajectory action server not available"};
    }

    // ---- velocity handling ----
    const double T_ref   = 2.5;   // reference time at vel = 1.0
    const double vel_min = 0.05;  // minimum velocity

    double v        = std::clamp(vel, vel_min, 1.0);  // clamp velocity
    double duration = T_ref / v;                      // time scaling

    // ---- build goal ----
    FollowJT::Goal goal;
    goal.trajectory.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions       = joints;
    p.time_from_start = rclcpp::Duration::from_seconds(duration);

    goal.trajectory.points.push_back(p);

    // ---- send goal ----
    auto goal_future = traj_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
        return {false, "Timeout while sending trajectory goal"};
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
        return {false, "Trajectory goal rejected"};
    }

    // ---- wait for result ----
    auto result_future = traj_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(60)) != std::future_status::ready) {
        return {false, "Timeout waiting for trajectory result"};
    }

    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        return {false, "Trajectory execution failed"};
    }

    return {true, "MoveJ done"};
}

}  // namespace ur_motion
