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

#include "robot_state_controller/robot_state_controller_node.hpp"

namespace RobotStateController {

using namespace std::placeholders;

RobotStateController::RobotStateController(rclcpp::NodeOptions options) : Node("robot_state_controller", options) {
    this->state.state = robot_state_msgs::msg::State::ACTIVE;

    this->system_state_publisher_ = this->create_publisher<robot_state_msgs::msg::State>("/robot/state", 10);

    this->set_state_srv_ = this->create_service<robot_state_msgs::srv::SetState>(
        "/robot/set_state", std::bind(&RobotStateController::set_state_cb, this, _1, _2));

    using namespace std::chrono_literals;
    system_state_timer_ = this->create_wall_timer(50ms, std::bind(&RobotStateController::update_state, this));
}

void RobotStateController::update_state() { system_state_publisher_->publish(this->state); }

void RobotStateController::set_state_cb(const robot_state_msgs::srv::SetState::Request::SharedPtr state,
                                        robot_state_msgs::srv::SetState::Response::SharedPtr resp) {
    this->state = state->state;
}

}  // namespace RobotStateController
