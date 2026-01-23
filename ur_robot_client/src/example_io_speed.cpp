/**
 * @file example_io_speed.cpp
 * @brief I/O and Speed control example
 *
 * Demonstrates how to control:
 *   - Digital I/O (input/output)
 *   - Speed slider
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "ur_robot_client/ur_robot_client.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto client = std::make_shared<URRobotClient>();

    // Use executor to handle ROS2 spinning
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(client);

    // Run executor in separate thread
    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    // Wait for robot connection
    RCLCPP_INFO(client->get_logger(), "Waiting for robot connection...");

    // Manually check connection (executor is running)
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        if (client->isConnected()) {
            break;
        }

        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed > 10s) {
            RCLCPP_ERROR(client->get_logger(), "Failed to connect to robot (timeout)");
            executor.cancel();
            rclcpp::shutdown();
            spin_thread.join();
            return 1;
        }

        std::this_thread::sleep_for(100ms);
    }

    if (!client->isConnected()) {
        RCLCPP_ERROR(client->get_logger(), "Failed to connect to robot");
        executor.cancel();
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    RCLCPP_INFO(client->get_logger(), "✅ Robot connected!");
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========================================");
    RCLCPP_INFO(client->get_logger(), "  I/O and Speed Control Example");
    RCLCPP_INFO(client->get_logger(), "========================================");
    RCLCPP_INFO(client->get_logger(), " ");

    // Wait for I/O states to be received
    std::this_thread::sleep_for(1s);

    // ========== Speed Slider Control ==========
    RCLCPP_INFO(client->get_logger(), "========== Speed Slider Control ==========");

    // Get current speed scaling
    double current_speed = client->getSpeedScaling();
    RCLCPP_INFO(client->get_logger(), "Current speed scaling: %.1f%%", current_speed * 100.0);

    // Set speed to 50%
    RCLCPP_INFO(client->get_logger(), "Setting speed slider to 50%%...");
    {
        auto result = client->setSpeedSlider(0.5, 1.0).get();
        if (result.success) {
            RCLCPP_INFO(client->get_logger(), "✅ Speed slider set successfully");
        } else {
            RCLCPP_WARN(client->get_logger(), "❌ Failed to set speed slider");
        }
    }

    std::this_thread::sleep_for(2s);

    // Set speed to 100%
    RCLCPP_INFO(client->get_logger(), "Setting speed slider to 100%%...");
    {
        auto result = client->setSpeedSlider(1.0, 1.0).get();
        if (result.success) {
            RCLCPP_INFO(client->get_logger(), "✅ Speed slider set successfully");
        } else {
            RCLCPP_WARN(client->get_logger(), "❌ Failed to set speed slider");
        }
    }

    std::this_thread::sleep_for(2s);

    // ========== Digital I/O Control ==========
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========== Digital I/O Control ==========");

    // Read digital inputs (pins 0-7: Standard, 8-15: Configurable, 16-17: Tool)
    RCLCPP_INFO(client->get_logger(), "Reading digital inputs...");
    for (int pin = 0; pin < 18; ++pin) {
        bool        state = client->getDigitalIn(pin);
        std::string pin_type;
        if (pin <= 7)
            pin_type = "Standard";
        else if (pin <= 15)
            pin_type = "Configurable";
        else
            pin_type = "Tool";

        RCLCPP_INFO(client->get_logger(), "  DI[%2d] (%s): %s",
                    pin, pin_type.c_str(), state ? "HIGH" : "LOW");
    }

    std::this_thread::sleep_for(1s);

    // Set digital outputs
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "Setting digital outputs...");

    // Example: Set DO[0] to HIGH
    RCLCPP_INFO(client->get_logger(), "Setting DO[0] (Standard) to HIGH...");
    {
        auto result = client->setDigitalOut(0, true, 1.0).get();
        if (result.success) {
            RCLCPP_INFO(client->get_logger(), "✅ DO[0] set to HIGH");
        } else {
            RCLCPP_WARN(client->get_logger(), "❌ Failed to set DO[0]");
        }
    }

    std::this_thread::sleep_for(2s);

    // Set DO[0] to LOW
    RCLCPP_INFO(client->get_logger(), "Setting DO[0] to LOW...");
    {
        auto result = client->setDigitalOut(0, false, 1.0).get();
        if (result.success) {
            RCLCPP_INFO(client->get_logger(), "✅ DO[0] set to LOW");
        } else {
            RCLCPP_WARN(client->get_logger(), "❌ Failed to set DO[0]");
        }
    }

    std::this_thread::sleep_for(1s);

    // Example: Toggle multiple outputs
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "Toggling DO[0-3] (Standard outputs)...");

    for (int i = 0; i < 3; ++i) {
        RCLCPP_INFO(client->get_logger(), "Cycle %d/3:", i + 1);

        // Turn ON
        for (int pin = 0; pin < 4; ++pin) {
            auto result = client->setDigitalOut(pin, true, 1.0).get();
            std::this_thread::sleep_for(200ms);
        }

        // Turn OFF
        for (int pin = 0; pin < 4; ++pin) {
            auto result = client->setDigitalOut(pin, false, 1.0).get();
            std::this_thread::sleep_for(200ms);
        }
    }

    // Read digital outputs
    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "Reading digital outputs...");
    for (int pin = 0; pin < 18; ++pin) {
        bool        state = client->getDigitalOut(pin);
        std::string pin_type;
        if (pin <= 7)
            pin_type = "Standard";
        else if (pin <= 15)
            pin_type = "Configurable";
        else
            pin_type = "Tool";

        RCLCPP_INFO(client->get_logger(), "  DO[%2d] (%s): %s",
                    pin, pin_type.c_str(), state ? "HIGH" : "LOW");
    }

    RCLCPP_INFO(client->get_logger(), " ");
    RCLCPP_INFO(client->get_logger(), "========================================");
    RCLCPP_INFO(client->get_logger(), "  I/O and Speed Control Example Completed!");
    RCLCPP_INFO(client->get_logger(), "========================================");

    rclcpp::shutdown();
    spin_thread.join();

    return 0;
}
