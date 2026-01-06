#include "ur_motion/transform_utils.hpp"

#include <cmath>

namespace ur_motion {

// Clamp helper for acos input (acos 입력값 클램프)
static double clamp(double v, double lo, double hi) {
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

void rotationMatrixToRPY(const double R[3][3], double& roll, double& pitch, double& yaw) {
    // ZYX convention: yaw-pitch-roll (ZYX 순서 기준)

    // pitch = atan2(-r20, sqrt(r00^2 + r10^2))
    pitch = std::atan2(-R[2][0], std::sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0]));

    // roll = atan2(r21, r22)
    roll = std::atan2(R[2][1], R[2][2]);

    // yaw = atan2(r10, r00)
    yaw = std::atan2(R[1][0], R[0][0]);
}

void rotationMatrixToRotVec(const double R[3][3], double& rx, double& ry, double& rz) {
    // Axis-angle from rotation matrix (회전행렬 -> 축-각)
    const double trace     = R[0][0] + R[1][1] + R[2][2];
    const double cos_theta = clamp((trace - 1.0) / 2.0, -1.0, 1.0);
    const double theta     = std::acos(cos_theta);

    // Small angle: return zero rotvec
    if (theta < 1e-9) {
        rx = ry = rz = 0.0;
        return;
    }

    const double sin_theta = std::sin(theta);

    // Rotation axis components
    const double ax = (R[2][1] - R[1][2]) / (2.0 * sin_theta);
    const double ay = (R[0][2] - R[2][0]) / (2.0 * sin_theta);
    const double az = (R[1][0] - R[0][1]) / (2.0 * sin_theta);

    // Rotation vector = axis * angle (회전벡터 = 축 * 각)
    rx = ax * theta;
    ry = ay * theta;
    rz = az * theta;

    normalizeRotationVector(rx, ry, rz);
}

void normalizeRotationVector(double& rx, double& ry, double& rz) {
    // Keep angle in [-pi, pi] by flipping axis if needed (각도를 [-pi, pi]로 정규화)
    const double angle = std::sqrt(rx * rx + ry * ry + rz * rz);
    if (angle < 1e-12)
        return;

    // If angle > pi, use equivalent rotation (angle' = 2pi - angle, axis' = -axis)
    // (각이 pi보다 크면 동치 회전으로 변환)
    if (angle > M_PI) {
        const double new_angle = 2.0 * M_PI - angle;
        const double scale     = -new_angle / angle;
        rx *= scale;
        ry *= scale;
        rz *= scale;
    }
}

URPose tmatrixToURPose(const std::array<double, 16>& T) {
    // T is assumed ROW-MAJOR 4x4
    // [ r00 r01 r02 tx
    //   r10 r11 r12 ty
    //   r20 r21 r22 tz
    //   0   0   0   1 ]
    URPose p{};

    // position (m)
    p.x = T[3];
    p.y = T[7];
    p.z = T[11];

    // rotation matrix
    double R[3][3] = {
        {T[0], T[1], T[2]},
        {T[4], T[5], T[6]},
        {T[8], T[9], T[10]}};

    // rotvec (rx, ry, rz)
    rotationMatrixToRotVec(R, p.rx, p.ry, p.rz);

    return p;
}

geometry_msgs::msg::Pose tmatrixToPose(const std::array<double, 16>& T) {
    // T is assumed ROW-MAJOR 4x4
    geometry_msgs::msg::Pose pose;

    pose.position.x = T[3];
    pose.position.y = T[7];
    pose.position.z = T[11];

    // Convert R -> quaternion (R을 쿼터니언으로 변환)
    // This is a compact numeric method (수치적으로 안정적인 방법)
    const double r00 = T[0], r01 = T[1], r02 = T[2];
    const double r10 = T[4], r11 = T[5], r12 = T[6];
    const double r20 = T[8], r21 = T[9], r22 = T[10];

    const double tr = r00 + r11 + r22;

    double qw, qx, qy, qz;
    if (tr > 0.0) {
        const double S = std::sqrt(tr + 1.0) * 2.0;
        qw             = 0.25 * S;
        qx             = (r21 - r12) / S;
        qy             = (r02 - r20) / S;
        qz             = (r10 - r01) / S;
    } else if ((r00 > r11) && (r00 > r22)) {
        const double S = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
        qw             = (r21 - r12) / S;
        qx             = 0.25 * S;
        qy             = (r01 + r10) / S;
        qz             = (r02 + r20) / S;
    } else if (r11 > r22) {
        const double S = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
        qw             = (r02 - r20) / S;
        qx             = (r01 + r10) / S;
        qy             = 0.25 * S;
        qz             = (r12 + r21) / S;
    } else {
        const double S = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
        qw             = (r10 - r01) / S;
        qx             = (r02 + r20) / S;
        qy             = (r12 + r21) / S;
        qz             = 0.25 * S;
    }

    pose.orientation.w = qw;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;

    return pose;
}

}  // namespace ur_motion
