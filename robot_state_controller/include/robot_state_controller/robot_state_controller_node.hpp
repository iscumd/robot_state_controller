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

#include "rclcpp/rclcpp.hpp"
#include "robot_state_controller/state.hpp"
#include "robot_state_msgs/msg/state.hpp"
#include "robot_state_msgs/srv/set_state.hpp"

namespace RobotStateController {
class RobotStateController : public rclcpp::Node {
public:
    explicit RobotStateController(rclcpp::NodeOptions options);

    /// Our current system state
    robot_state_msgs::msg::State state;

    void set_state_cb(const robot_state_msgs::srv::SetState::Request::SharedPtr state,
                      robot_state_msgs::srv::SetState::Response::SharedPtr resp);

private:
    // Used to publish the state on a timer
    rclcpp::TimerBase::SharedPtr system_state_timer_;
    void update_state();

    /// Publishes the system state on an interval
    rclcpp::Publisher<robot_state_msgs::msg::State>::SharedPtr system_state_publisher_;

    /// Service to modify the system state
    rclcpp::Service<robot_state_msgs::srv::SetState>::SharedPtr set_state_srv_;
};
}  // namespace RobotStateController
