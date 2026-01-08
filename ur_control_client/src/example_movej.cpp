/**
 * @file example_movej.cpp
 * @brief MoveJ motion control example
 *
 * Demonstrates how to control UR robot using joint space motion (MoveJ)
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "ur_control_client/ur_control_client.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto client = std::make_shared<URControlClient>();

    // Spin in background thread
    std::thread spin_thread([client]() {
        rclcpp::spin(client);
    });

    // Wait for robot connection
    RCLCPP_INFO(client->get_logger(), "Waiting for robot connection...");
    while (rclcpp::ok() && !client->isConnected()) {
        std::this_thread::sleep_for(100ms);
    }

    if (!client->isConnected()) {
        RCLCPP_ERROR(client->get_logger(), "Failed to connect to robot");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    RCLCPP_INFO(client->get_logger(), "✅ Robot connected!");
    RCLCPP_INFO(client->get_logger(), "");
    RCLCPP_INFO(client->get_logger(), "========================================");
    RCLCPP_INFO(client->get_logger(), "  MoveJ Example: Joint Space Motion");
    RCLCPP_INFO(client->get_logger(), "========================================");
    RCLCPP_INFO(client->get_logger(), "");

    // Example 1: Move to home position
    RCLCPP_INFO(client->get_logger(), "Example 1: Moving to HOME position");
    std::vector<double> home = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};

    if (client->moveJ(home, 0.3, true)) {
        RCLCPP_INFO(client->get_logger(), "✅ Reached HOME position");
    } else {
        RCLCPP_ERROR(client->get_logger(), "❌ Failed to reach HOME position");
    }

    std::this_thread::sleep_for(1s);

    // Example 2: Move to target position
    RCLCPP_INFO(client->get_logger(), "");
    RCLCPP_INFO(client->get_logger(), "Example 2: Moving to TARGET position");
    std::vector<double> target = {0.5, -1.2, 1.0, -1.5, -1.57, 0.5};

    if (client->moveJ(target, 0.5, true)) {
        RCLCPP_INFO(client->get_logger(), "✅ Reached TARGET position");
    } else {
        RCLCPP_ERROR(client->get_logger(), "❌ Failed to reach TARGET position");
    }

    std::this_thread::sleep_for(1s);

    // Example 3: Return to home
    RCLCPP_INFO(client->get_logger(), "");
    RCLCPP_INFO(client->get_logger(), "Example 3: Returning to HOME");

    if (client->moveJ(home, 0.3, true)) {
        RCLCPP_INFO(client->get_logger(), "✅ Returned to HOME position");
    } else {
        RCLCPP_ERROR(client->get_logger(), "❌ Failed to return to HOME");
    }

    RCLCPP_INFO(client->get_logger(), "");
    RCLCPP_INFO(client->get_logger(), "========================================");
    RCLCPP_INFO(client->get_logger(), "  MoveJ Example Completed!");
    RCLCPP_INFO(client->get_logger(), "========================================");

    rclcpp::shutdown();
    spin_thread.join();

    return 0;
}
