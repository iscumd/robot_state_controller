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

#ifndef ROBOT_STATE_CONTROLLER__INITIAL_POINT_PUBLISHER_HPP_
#define ROBOT_STATE_CONTROLLER__INITIAL_POINT_PUBLISHER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robot_state_controller/state.hpp"
#include "robot_state_msgs/msg/drive_mode.hpp"

namespace RobotStateController
{
class InitialPointPublisher : public rclcpp::Node
{
public:
    explicit InitialPointPublisher(rclcpp::NodeOptions options);

private:
    // Param
    std::string robot_frame_;
    std::string map_frame_;
    rclcpp::TimerBase::SharedPtr param_update_timer_;
    void update_params();

    // Var
    bool is_initialized;
    State::DriveMode last_drive_mode_state_;

    // Callbacks
    void drive_mode_callback(const robot_state_msgs::msg::DriveMode::SharedPtr msg);

    // Pubs/Subs
    rclcpp::Subscription<robot_state_msgs::msg::DriveMode>::SharedPtr drive_mode_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr initial_point_publisher_;

    // TF
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};
} // namespace RobotStateController

#endif // ROBOT_STATE_CONTROLLER__INITIAL_POINT_PUBLISHER_HPP_