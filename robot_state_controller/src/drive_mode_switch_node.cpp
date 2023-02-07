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

#include "robot_state_controller/drive_mode_switch_node.hpp"

namespace RobotStateController {
DriveModeSwitch::DriveModeSwitch(rclcpp::NodeOptions options) : Node("drive_mode_switch", options) {
    using namespace std::chrono_literals;

    // Param
    this->declare_parameter<int>("switch_button", 8);
    param_update_timer_ = this->create_wall_timer(1000ms, std::bind(&DriveModeSwitch::update_params, this));

    // Subs/Pubs
    robot_state_subscription_ = this->create_subscription<robot_state_msgs::msg::State>(
        "/robot/state", 10, std::bind(&DriveModeSwitch::robot_state_callback, this, std::placeholders::_1));
    joystick_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&DriveModeSwitch::joystick_callback, this, std::placeholders::_1));
    controller_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&DriveModeSwitch::controller_vel_callback, this, std::placeholders::_1));
    navigation_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/nav_vel", 10, std::bind(&DriveModeSwitch::navigation_vel_callback, this, std::placeholders::_1));
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    drive_mode_publisher_ = this->create_publisher<robot_state_msgs::msg::DriveMode>("/robot/drive_mode", 10);
    // Init values
    last_system_state_ = State::System::ACTIVE;
    last_drive_mode_state_ = State::DriveMode::TELEOP;
    last_switch_button_pressed_ = false;
}
void DriveModeSwitch::update_params() { this->get_parameter("switch_button", switch_button_); }
void DriveModeSwitch::robot_state_callback(const robot_state_msgs::msg::State::SharedPtr msg) {
    // Save system state from incoming state information
    switch (msg->state) {
        case 0:
            last_system_state_ = State::System::KILL;
            break;
        case 1:
            last_system_state_ = State::System::PAUSE;
            break;
        case 2:
            last_system_state_ = State::System::ACTIVE;
            break;
        default:
            last_system_state_ = State::System::KILL;
    }
}
void DriveModeSwitch::joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Whether or not the switch button is currently pressed
    bool switch_button_pressed_ = msg->buttons[switch_button_] == 1;

    // Drive mode message to be published
    robot_state_msgs::msg::DriveMode drive_mode_msg;

    // Check if the switch button is being currently pressed against
    // whether or not the switch button was previously pressed.
    //
    // NOTE: This will debounce the switch button. If not done this way,
    // the drive mode will be rapidly switched at the joy publish frequency
    // as long as the button is being held. Even a normal button press
    // on the controller will lead to unpredictable drive mode state if
    // not triggered falling edge in this way.
    if (last_switch_button_pressed_ && !switch_button_pressed_) {
        // Update drive mode state
        switch (last_drive_mode_state_) {
            case State::DriveMode::TELEOP:
                last_drive_mode_state_ = State::DriveMode::AUTONOMOUS;
                drive_mode_msg.drive_mode = 1;
                break;
            case State::DriveMode::AUTONOMOUS:
                last_drive_mode_state_ = State::DriveMode::TELEOP;
                drive_mode_msg.drive_mode = 0;
                break;
        }
        // Publish drive mode
        drive_mode_publisher_->publish(drive_mode_msg);
    }

    // Update whether or not switch button was pressed for debouncing
    last_switch_button_pressed_ = switch_button_pressed_;
}
void DriveModeSwitch::controller_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Only publish the twist command from the controller
    // if the robot is in teleop and not killed
    if (last_drive_mode_state_ == State::DriveMode::TELEOP && last_system_state_ == State::System::ACTIVE) {
        cmd_vel_publisher_->publish(*msg);
    }
}
void DriveModeSwitch::navigation_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Only publish the twist command from navigation
    // if the robot is in teleop and not killed
    if (last_drive_mode_state_ == State::DriveMode::AUTONOMOUS && last_system_state_ == State::System::ACTIVE) {
        cmd_vel_publisher_->publish(*msg);
    }
}
}  // namespace RobotStateController
