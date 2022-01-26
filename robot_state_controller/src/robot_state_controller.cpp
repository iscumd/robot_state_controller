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

#include "robot_state_controller/robot_state_controller.hpp"

namespace RobotStateController
{
RobotStateController::RobotStateController(rclcpp::NodeOptions options)
: Node("robot_state_controller", options)
{
    system_state_publisher_ = this->create_publisher<robot_state_msgs::msg::State>(
        "/robot/state", 10
    );

    using namespace std::chrono_literals; // for 1000ms?
    system_state_timer_ = this->create_wall_timer(
      1000ms, std::bind(&RobotStateController::update_state, this)
    );
}
void RobotStateController::update_state()
{
    robot_state_msgs::msg::State system_state;
    system_state.state = 2;
    system_state_publisher_->publish(system_state);
}
} // namespace RobotStateController

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto lp_node = std::make_shared<RobotStateController::RobotStateController>(options);
    exec.add_node(lp_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}