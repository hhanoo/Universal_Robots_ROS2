/**
 * @file example_complete.cpp
 * @brief Complete example demonstrating all features
 *
 * Demonstrates:
 *   - MoveJ (joint space motion)
 *   - MoveL (Cartesian linear motion)
 *   - Speed slider control
 *   - Digital I/O control
 *   - State monitoring
 */

#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "ur_robot_client/ur_robot_client.hpp"

using namespace std::chrono_literals;

// Helper function to create T-matrix from position and rotation
std::array<double, 16> createTMatrix(double x, double y, double z,
                                     double rx, double ry, double rz) {
    std::array<double, 16> T;

    // Rotation matrices
    double cx = std::cos(rx), sx = std::sin(rx);
    double cy = std::cos(ry), sy = std::sin(ry);
    double cz = std::cos(rz), sz = std::sin(rz);

    // Combined rotation matrix (ZYX order)
    T[0] = cy * cz;
    T[1] = cz * sx * sy - cx * sz;
    T[2] = sx * sz + cx * cz * sy;
    T[3] = x;

    T[4] = cy * sz;
    T[5] = cx * cz + sx * sy * sz;
    T[6] = cx * sy * sz - cz * sx;
    T[7] = y;

    T[8]  = -sy;
    T[9]  = cy * sx;
    T[10] = cx * cy;
    T[11] = z;

    T[12] = 0.0;
    T[13] = 0.0;
    T[14] = 0.0;
    T[15] = 1.0;

    return T;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // URRobotClient spins itself in a background executor thread,
    // so no external executor is needed
    auto client = std::make_shared<URRobotClient>();

    // Wait for robot connection
    RCLCPP_INFO(client->get_logger(), "Waiting for robot connection...");

    // Manually check connection (client spins in background)
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        if (client->isConnected()) {
            break;
        }

        auto elapsed = std::chrono::steady_clock::now() - start_time;
        std::this_thread::sleep_for(100ms);
        if (elapsed > 10s) {
            RCLCPP_ERROR(client->get_logger(), "Failed to connect to robot (timeout)");
            rclcpp::shutdown();
            return 1;
        }
    }

    if (!client->isConnected()) {
        RCLCPP_ERROR(client->get_logger(), "Failed to connect to robot");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(client->get_logger(), "✅ Robot connected!");
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========================================");
    RCLCPP_INFO(client->get_logger(), "  Complete UR Control Example");
    RCLCPP_INFO(client->get_logger(), "========================================");
    RCLCPP_INFO(client->get_logger(), " ");

    std::this_thread::sleep_for(1s);

    // ========== 1. Speed Control ==========
    RCLCPP_INFO(client->get_logger(), "========== 1. Speed Control ==========");
    RCLCPP_INFO(client->get_logger(), "Setting speed to 30%% for safe operation...");
    {
        auto result = client->setSpeedSlider(0.3, 1.0).get();
        if (result.success == false) {
            RCLCPP_ERROR(client->get_logger(), "⚠️ Failed to set speed slider: %s", result.message.c_str());
        }
    }
    std::this_thread::sleep_for(1s);

    // ========== 2. MoveJ to Home ==========
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========== 2. MoveJ to Home ==========");
    std::vector<double> home = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};

    {
        auto result = client->moveJ(home, 0.3, 30.0).get();
        if (result.success) {
            RCLCPP_INFO(client->get_logger(), "✅ Reached HOME position");

            // Get current joint positions
            std::array<double, 6> current_joints = client->getJointPositions();
            RCLCPP_INFO(client->get_logger(), "Current joint positions:");
            for (size_t i = 0; i < 6; ++i) {
                RCLCPP_INFO(client->get_logger(),
                            "  Joint[%zu]: %.4f rad", i, current_joints[i]);
            }
        } else {
            RCLCPP_ERROR(client->get_logger(), "❌ Failed to reach HOME: %s", result.message.c_str());
        }
    }

    std::this_thread::sleep_for(2s);

    // ========== 3. Digital Output Control ==========
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========== 3. Digital Output Control ==========");
    RCLCPP_INFO(client->get_logger(), "Turning ON DO[0] (Standard output)...");
    {
        auto result = client->setDigitalOut(0, true, 1.0).get();
        if (result.success == false) {
            RCLCPP_ERROR(client->get_logger(), "⚠️ Failed to set DO[0]: %s", result.message.c_str());
        }
    }
    std::this_thread::sleep_for(1s);

    // ========== 4. MoveJ to Pre-Pick Position ==========
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========== 4. MoveJ to Pre-Pick ==========");
    std::vector<double> pre_pick = {0.5, -1.2, 1.0, -1.5, -1.57, 0.5};

    {
        auto result = client->moveJ(pre_pick, 0.3, 30.0).get();
        if (result.success) {
            RCLCPP_INFO(client->get_logger(), "✅ Reached Pre-Pick position");
        } else {
            RCLCPP_ERROR(client->get_logger(), "❌ Failed to reach Pre-Pick: %s", result.message.c_str());
        }
    }

    std::this_thread::sleep_for(1s);

    // ========== 5. MoveL Down (Simulated Pick) ==========
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========== 5. MoveL Down (Pick) ==========");

    // Create T-matrix for downward motion (example values)
    auto tmatrix_down = createTMatrix(
        -0.4, -0.2, 0.2,  // Position (x, y, z) in meters
        M_PI, 0.0, 0.0    // Rotation (rx, ry, rz) - tool pointing down
    );

    RCLCPP_INFO(client->get_logger(), "Moving down to pick position...");
    {
        auto result = client->moveL(tmatrix_down, 0.2, 30.0).get();
        if (result.success) {
            RCLCPP_INFO(client->get_logger(), "✅ Reached pick position");
        } else {
            RCLCPP_ERROR(client->get_logger(), "❌ Failed to reach pick position: %s", result.message.c_str());
        }
    }

    std::this_thread::sleep_for(1s);

    // ========== 6. Gripper Control (Simulated with DO) ==========
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========== 6. Gripper Control (DO[1]) ==========");
    RCLCPP_INFO(client->get_logger(), "Closing gripper (DO[1] = HIGH)...");
    {
        auto result = client->setDigitalOut(1, true, 1.0).get();
        if (result.success == false) {
            RCLCPP_ERROR(client->get_logger(), "⚠️ Failed to set DO[1]: %s", result.message.c_str());
        }
    }
    std::this_thread::sleep_for(1s);

    // ========== 7. MoveL Up ==========
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========== 7. MoveL Up ==========");

    auto tmatrix_up = createTMatrix(
        -0.4, -0.2, 0.4,  // Position (10cm higher)
        M_PI, 0.0, 0.0);

    RCLCPP_INFO(client->get_logger(), "Moving up with object...");
    {
        auto result = client->moveL(tmatrix_up, 0.2, 30.0).get();
        if (result.success) {
            RCLCPP_INFO(client->get_logger(), "✅ Moved up successfully");
        } else {
            RCLCPP_ERROR(client->get_logger(), "❌ Failed to move up: %s", result.message.c_str());
        }
    }

    std::this_thread::sleep_for(1s);

    // ========== 8. MoveJ to Place Position ==========
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========== 8. MoveJ to Place Position ==========");
    std::vector<double> place = {-0.5, -1.2, 1.0, -1.5, -1.57, -0.5};

    {
        auto result = client->moveJ(place, 0.3, 30.0).get();
        if (result.success) {
            RCLCPP_INFO(client->get_logger(), "✅ Reached Place position");
        } else {
            RCLCPP_ERROR(client->get_logger(), "❌ Failed to reach Place position: %s", result.message.c_str());
        }
    }

    std::this_thread::sleep_for(1s);

    // ========== 9. Release Object ==========
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========== 9. Release Object ==========");
    RCLCPP_INFO(client->get_logger(), "Opening gripper (DO[1] = LOW)...");
    {
        auto result = client->setDigitalOut(1, false, 1.0).get();
        if (result.success == false) {
            RCLCPP_ERROR(client->get_logger(), "⚠️ Failed to set DO[1]: %s", result.message.c_str());
        }
    }
    std::this_thread::sleep_for(1s);

    // ========== 10. Return to Home ==========
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========== 10. Return to Home ==========");

    {
        auto result = client->moveJ(home, 0.3, 30.0).get();
        if (result.success) {
            RCLCPP_INFO(client->get_logger(), "✅ Returned to HOME");
        } else {
            RCLCPP_ERROR(client->get_logger(), "❌ Failed to return to HOME: %s", result.message.c_str());
        }
    }

    // ========== 11. Turn OFF Outputs ==========
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========== 11. Cleanup ==========");
    RCLCPP_INFO(client->get_logger(), "Turning OFF all outputs...");
    {
        auto result = client->setDigitalOut(0, false, 1.0).get();
        if (result.success == false) {
            RCLCPP_ERROR(client->get_logger(), "⚠️ Failed to set DO[0]: %s", result.message.c_str());
        }
    }
    {
        auto result = client->setDigitalOut(1, false, 1.0).get();
        if (result.success == false) {
            RCLCPP_ERROR(client->get_logger(), "⚠️ Failed to set DO[1]: %s", result.message.c_str());
        }
    }

    // ========== 12. Reset Speed ==========
    RCLCPP_INFO(client->get_logger(), "Resetting speed to 100%%...");
    {
        auto result = client->setSpeedSlider(1.0, 1.0).get();
        if (result.success == false) {
            RCLCPP_ERROR(client->get_logger(), "⚠️ Failed to set speed slider: %s", result.message.c_str());
        }
    }

    std::this_thread::sleep_for(1s);

    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========================================");
    RCLCPP_INFO(client->get_logger(), "  ✅ Complete Example Finished!");
    RCLCPP_INFO(client->get_logger(), "========================================");
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "Demonstrated features:");
    RCLCPP_INFO(client->get_logger(), "  ✓ Speed slider control");
    RCLCPP_INFO(client->get_logger(), "  ✓ MoveJ (joint space motion)");
    RCLCPP_INFO(client->get_logger(), "  ✓ MoveL (Cartesian linear motion)");
    RCLCPP_INFO(client->get_logger(), "  ✓ Digital I/O control");
    RCLCPP_INFO(client->get_logger(), "  ✓ State monitoring");

    rclcpp::shutdown();

    return 0;
}
