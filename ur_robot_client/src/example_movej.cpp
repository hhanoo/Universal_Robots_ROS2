/**
 * @file example_movej.cpp
 * @brief MoveJ motion control example
 *
 * Demonstrates how to control UR robot using joint space motion (MoveJ)
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
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

    // Manually check connection (client spins in background)
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        if (client->isConnected()) {
            break;
        }

        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed > 10s) {
            RCLCPP_ERROR(client->get_logger(), "Failed to connect to robot (timeout)");
            rclcpp::shutdown();
            return 1;
        }

        std::this_thread::sleep_for(100ms);
    }

    if (!client->isConnected()) {
        RCLCPP_ERROR(client->get_logger(), "Failed to connect to robot");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(client->get_logger(), "✅ Robot connected!");
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========================================");
    RCLCPP_INFO(client->get_logger(), "  MoveJ Example: Joint Space Motion");
    RCLCPP_INFO(client->get_logger(), "========================================");
    RCLCPP_INFO(client->get_logger(), " ");

    // Example 1: Move to home position
    RCLCPP_INFO(client->get_logger(), "Example 1: Moving to HOME position");
    std::vector<double> home = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};

    {
        auto result = client->moveJ(home, 0.3, 30.0).get();
        if (result.success) {
            RCLCPP_INFO(client->get_logger(), "✅ Reached HOME position");
        } else {
            RCLCPP_ERROR(client->get_logger(), "❌ Failed to reach HOME position: %s", result.message.c_str());
        }
    }

    std::this_thread::sleep_for(1s);

    // Example 2: Move to target position
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "Example 2: Moving to TARGET position");
    std::vector<double> target = {0.5, -1.2, 1.0, -1.5, -1.57, 0.5};

    {
        auto result = client->moveJ(target, 0.5, 30.0).get();
        if (result.success) {
            RCLCPP_INFO(client->get_logger(), "✅ Reached TARGET position");
        } else {
            RCLCPP_ERROR(client->get_logger(), "❌ Failed to reach TARGET position: %s", result.message.c_str());
        }
    }

    std::this_thread::sleep_for(1s);

    // Example 3: Return to home
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "Example 3: Returning to HOME");

    {
        auto result = client->moveJ(home, 0.3, 30.0).get();
        if (result.success) {
            RCLCPP_INFO(client->get_logger(), "✅ Returned to HOME position");
        } else {
            RCLCPP_ERROR(client->get_logger(), "❌ Failed to return to HOME: %s", result.message.c_str());
        }
    }

    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========================================");
    RCLCPP_INFO(client->get_logger(), "  MoveJ Example Completed!");
    RCLCPP_INFO(client->get_logger(), "========================================");

    rclcpp::shutdown();

    return 0;
}