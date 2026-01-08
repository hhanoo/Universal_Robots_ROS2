#ifndef UR_CONTROL_CLIENT_HPP
#define UR_CONTROL_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

// UR-specific messages and services
#include "ur_msgs/msg/io_states.hpp"
#include "ur_msgs/srv/set_io.hpp"
#include "ur_msgs/srv/set_speed_slider_fraction.hpp"

// Action definitions
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "ur_motion/action/move_j.hpp"
#include "ur_motion/action/move_l.hpp"

/**
 * @brief UR Robot Control Client
 *
 * Provides high-level interface for controlling UR robots:
 *   - MoveJ/MoveL motion control
 *   - Speed slider control
 *   - Digital I/O control
 *   - Robot state monitoring
 */
class URControlClient : public rclcpp::Node {
   public:
    URControlClient();
    ~URControlClient();

    // ========== Motion Control ==========
    /**
     * @brief Execute MoveJ (joint space motion)
     * @param joints 6 joint positions in radians
     * @param velocity Velocity scaling [0.01 ~ 1.0]
     * @param wait Wait for motion to complete
     * @return true if succeeded
     */
    bool moveJ(const std::vector<double>& joints, double velocity = 0.5, bool wait = true);

    /**
     * @brief Execute MoveL (Cartesian linear motion)
     * @param tmatrix 4x4 transformation matrix (row-major, 16 elements)
     * @param velocity Velocity scaling [0.01 ~ 1.0]
     * @param wait Wait for motion to complete
     * @return true if succeeded
     */
    bool moveL(const std::array<double, 16>& tmatrix, double velocity = 0.5, bool wait = true);

    // ========== Speed Control ==========
    /**
     * @brief Set speed slider fraction
     * @param fraction Speed slider [0.01 ~ 1.0]
     * @return true if succeeded
     */
    bool setSpeedSlider(double fraction);

    /**
     * @brief Get current speed scaling
     * @return Current speed scaling [0.0 ~ 1.0]
     */
    double getSpeedScaling() const;

    // ========== I/O Control ==========
    /**
     * @brief Set digital output pin
     * @param pin Pin number [0-17]
     *            0-7: Standard digital outputs
     *            8-15: Configurable digital outputs
     *            16-17: Tool digital outputs
     * @param value Output value (true=HIGH, false=LOW)
     * @return true if succeeded
     */
    bool setDigitalOut(int pin, bool value);

    /**
     * @brief Get digital input pin state
     * @param pin Pin number [0-17]
     * @return Pin state (true=HIGH, false=LOW)
     */
    bool getDigitalIn(int pin) const;

    /**
     * @brief Get digital output pin state
     * @param pin Pin number [0-17]
     * @return Pin state (true=HIGH, false=LOW)
     */
    bool getDigitalOut(int pin) const;

    // ========== State Monitoring ==========
    /**
     * @brief Check if robot is connected
     * @return true if connected
     */
    bool isConnected() const;

    /**
     * @brief Get latest joint positions
     * @param joints Output vector for 6 joint positions
     * @return true if valid data available
     */
    bool getJointPositions(std::vector<double>& joints) const;

   private:
    // ========== Action Clients ==========
    rclcpp_action::Client<ur_motion::action::MoveJ>::SharedPtr movej_client_;
    rclcpp_action::Client<ur_motion::action::MoveL>::SharedPtr movel_client_;

    // ========== Service Clients ==========
    rclcpp::Client<ur_msgs::srv::SetSpeedSliderFraction>::SharedPtr speed_slider_client_;
    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr                  set_io_client_;

    // ========== Subscribers ==========
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr       speed_scaling_sub_;
    rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr       io_states_sub_;

    // ========== State Data ==========
    sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
    rclcpp::Time                            last_joint_state_time_;
    bool                                    connected_;

    double speed_scaling_;  // Current speed scaling [0.0 ~ 1.0]

    std::array<bool, 18> digital_in_states_;   // Digital input states
    std::array<bool, 18> digital_out_states_;  // Digital output states

    // ========== Callbacks ==========
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void speedScalingCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void ioStatesCallback(const ur_msgs::msg::IOStates::SharedPtr msg);

    // ========== Helper Functions ==========
    bool waitForActionServer(const std::string& action_name, std::chrono::seconds timeout);
};

#endif  // UR_CONTROL_CLIENT_HPP
