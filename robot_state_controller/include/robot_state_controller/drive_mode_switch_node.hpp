// MIT License
//
// Copyright (c) Intelligent Systems Club
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once
#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_state_controller/state.hpp"
#include "robot_state_msgs/msg/drive_mode.hpp"
#include "robot_state_msgs/msg/state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/header.hpp"

namespace RobotStateController {
class DriveModeSwitch : public rclcpp::Node {
public:
    explicit DriveModeSwitch(rclcpp::NodeOptions options);

private:
    // Var
    bool last_switch_button_pressed_;
    State::System last_system_state_;
    State::DriveMode last_drive_mode_state_;

    // Params
    unsigned int switch_button_;
    rclcpp::TimerBase::SharedPtr param_update_timer_;
    void update_params();

    // Callbacks
    void robot_state_callback(const robot_state_msgs::msg::State::SharedPtr msg);
    void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void controller_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void navigation_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Pubs/Subs
    rclcpp::Subscription<robot_state_msgs::msg::State>::SharedPtr robot_state_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller_vel_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navigation_vel_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<robot_state_msgs::msg::DriveMode>::SharedPtr drive_mode_publisher_;
};
}  // namespace RobotStateController
