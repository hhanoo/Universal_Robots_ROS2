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
    RCLCPP_INFO(client->get_logger(), "  I/O and Speed Control Example");
    RCLCPP_INFO(client->get_logger(), "========================================");
    RCLCPP_INFO(client->get_logger(), "");

    // Wait for I/O states to be received
    std::this_thread::sleep_for(1s);

    // ========== Speed Slider Control ==========
    RCLCPP_INFO(client->get_logger(), "========== Speed Slider Control ==========");

    // Get current speed scaling
    double current_speed = client->getSpeedScaling();
    RCLCPP_INFO(client->get_logger(), "Current speed scaling: %.1f%%", current_speed * 100.0);

    // Set speed to 50%
    RCLCPP_INFO(client->get_logger(), "Setting speed slider to 50%%...");
    if (client->setSpeedSlider(0.5)) {
        RCLCPP_INFO(client->get_logger(), "✅ Speed slider set successfully");
    } else {
        RCLCPP_WARN(client->get_logger(), "❌ Failed to set speed slider");
    }

    std::this_thread::sleep_for(2s);

    // Set speed to 100%
    RCLCPP_INFO(client->get_logger(), "Setting speed slider to 100%%...");
    if (client->setSpeedSlider(1.0)) {
        RCLCPP_INFO(client->get_logger(), "✅ Speed slider set successfully");
    } else {
        RCLCPP_WARN(client->get_logger(), "❌ Failed to set speed slider");
    }

    std::this_thread::sleep_for(2s);

    // ========== Digital I/O Control ==========
    RCLCPP_INFO(client->get_logger(), "");
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
    RCLCPP_INFO(client->get_logger(), "");
    RCLCPP_INFO(client->get_logger(), "Setting digital outputs...");

    // Example: Set DO[0] to HIGH
    RCLCPP_INFO(client->get_logger(), "Setting DO[0] (Standard) to HIGH...");
    if (client->setDigitalOut(0, true)) {
        RCLCPP_INFO(client->get_logger(), "✅ DO[0] set to HIGH");
    } else {
        RCLCPP_WARN(client->get_logger(), "❌ Failed to set DO[0]");
    }

    std::this_thread::sleep_for(2s);

    // Set DO[0] to LOW
    RCLCPP_INFO(client->get_logger(), "Setting DO[0] to LOW...");
    if (client->setDigitalOut(0, false)) {
        RCLCPP_INFO(client->get_logger(), "✅ DO[0] set to LOW");
    } else {
        RCLCPP_WARN(client->get_logger(), "❌ Failed to set DO[0]");
    }

    std::this_thread::sleep_for(1s);

    // Example: Toggle multiple outputs
    RCLCPP_INFO(client->get_logger(), "");
    RCLCPP_INFO(client->get_logger(), "Toggling DO[0-3] (Standard outputs)...");

    for (int i = 0; i < 3; ++i) {
        RCLCPP_INFO(client->get_logger(), "Cycle %d/3:", i + 1);

        // Turn ON
        for (int pin = 0; pin < 4; ++pin) {
            client->setDigitalOut(pin, true);
            std::this_thread::sleep_for(200ms);
        }

        // Turn OFF
        for (int pin = 0; pin < 4; ++pin) {
            client->setDigitalOut(pin, false);
            std::this_thread::sleep_for(200ms);
        }
    }

    // Read digital outputs
    RCLCPP_INFO(client->get_logger(), "");
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

    RCLCPP_INFO(client->get_logger(), "");
    RCLCPP_INFO(client->get_logger(), "========================================");
    RCLCPP_INFO(client->get_logger(), "  I/O and Speed Control Example Completed!");
    RCLCPP_INFO(client->get_logger(), "========================================");

    rclcpp::shutdown();
    spin_thread.join();

    return 0;
}
