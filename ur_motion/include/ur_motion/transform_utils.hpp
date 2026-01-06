#pragma once

#include <array>
#include <geometry_msgs/msg/pose.hpp>

namespace ur_motion {

struct URPose {
    double x, y, z;
    double rx, ry, rz;
};

// -------- T matrix related --------
URPose                   tmatrixToURPose(const std::array<double, 16>& T);
geometry_msgs::msg::Pose tmatrixToPose(const std::array<double, 16>& T);

// -------- rotation conversions --------
void rotationMatrixToRPY(
    const double R[3][3],
    double& roll, double& pitch, double& yaw);

void rotationMatrixToRotVec(
    const double R[3][3],
    double& rx, double& ry, double& rz);

// -------- helpers --------
void normalizeRotationVector(double& rx, double& ry, double& rz);

}  // namespace ur_motion
