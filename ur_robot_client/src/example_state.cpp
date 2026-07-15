/**
 * @file example_state.cpp
 * @brief Read-only robot state monitoring example
 *
 * Safe to run on a real robot (no motion, no output changes).
 * Prints every second:
 *   - Joint positions
 *   - TCP pose (via TF)
 *   - Speed slider / speed scaling
 *   - Digital I/O states
 *   - Program running (external control) state
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <thread>

#include "ur_robot_client/ur_robot_client.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // URRobotClient spins itself in a background executor thread,
    // so no external executor is needed
    auto client = std::make_shared<URRobotClient>();

    // Wait for robot connection
    RCLCPP_INFO(client->get_logger(), "Waiting for robot connection...");

    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !client->isConnected()) {
        if (std::chrono::steady_clock::now() - start_time > 10s) {
            RCLCPP_ERROR(client->get_logger(), "Failed to connect to robot (timeout)");
            rclcpp::shutdown();
            return 1;
        }
        std::this_thread::sleep_for(100ms);
    }

    RCLCPP_INFO(client->get_logger(), "✅ Robot connected!");
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========================================");
    RCLCPP_INFO(client->get_logger(), "  Robot State Monitoring (read-only)");
    RCLCPP_INFO(client->get_logger(), "  Press Ctrl+C to stop");
    RCLCPP_INFO(client->get_logger(), "========================================");

    while (rclcpp::ok()) {
        // Joint positions [rad]
        auto joints = client->getJointPositions();
        RCLCPP_INFO(client->get_logger(),
                    "Joints [rad]: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);

        // TCP pose (4x4 T-matrix row-major, translation = elements 3, 7, 11)
        if (client->isTcpPoseAvailable()) {
            auto tcp = client->getTcpPose();
            RCLCPP_INFO(client->get_logger(),
                        "TCP [m]     : x=%.3f, y=%.3f, z=%.3f",
                        tcp[3], tcp[7], tcp[11]);
        } else {
            RCLCPP_INFO(client->get_logger(), "TCP [m]     : not available (TF)");
        }

        // Speed slider (user-set) vs speed scaling (actual robot speed)
        RCLCPP_INFO(client->get_logger(),
                    "Speed       : slider=%.0f%%, scaling=%.0f%%",
                    client->getSpeedSlider() * 100.0,
                    client->getSpeedScaling() * 100.0);

        // Digital I/O (standard pins 0-7, printed as bit string)
        std::ostringstream di, dout;
        for (int pin = 0; pin < 8; ++pin) {
            di << (client->getDigitalIn(pin) ? '1' : '0');
            dout << (client->getDigitalOut(pin) ? '1' : '0');
        }
        RCLCPP_INFO(client->get_logger(),
                    "I/O [0-7]   : DI=%s, DO=%s",
                    di.str().c_str(), dout.str().c_str());

        // Program (external control) state — false means control lost
        // (e-stop / Local mode); the built-in watchdog auto-regains it
        RCLCPP_INFO(client->get_logger(),
                    "Program     : %s",
                    client->isProgramRunning() ? "RUNNING (control OK)"
                                               : "STOPPED (control lost)");

        RCLCPP_INFO(client->get_logger(), "----------------------------------------");
        std::this_thread::sleep_for(1s);
    }

    rclcpp::shutdown();

    return 0;
}
