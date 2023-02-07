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

#include "robot_state_controller/initial_point_publisher.hpp"

using namespace std::chrono_literals;

namespace RobotStateController
{
InitialPointPublisher::InitialPointPublisher(rclcpp::NodeOptions options)
: Node("initial_point_publisher", options)
{
    // Params
    this->declare_parameter<std::string>("robot_frame", "base_footprint");
    this->declare_parameter<std::string>("map_frame", "map");
    param_update_timer_ = this->create_wall_timer(
      1000ms, std::bind(&InitialPointPublisher::update_params, this)
      );
    
    // Subs/Pubs
    drive_mode_subscription_ = this->create_subscription<robot_state_msgs::msg::DriveMode>(
        "/robot/drive_mode", 10,
        std::bind(&InitialPointPublisher::drive_mode_callback, this, std::placeholders::_1)
    );
    initial_point_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10
    );

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initial var state
    transform_tolerance_ = tf2::durationFromSec(0.1);
    
    is_initialized = false; // has initial_point been sent before
    last_drive_mode_state_ = State::DriveMode::TELEOP;
}
void InitialPointPublisher::update_params()
{
    this->get_parameter("robot_frame", robot_frame_);
    this->get_parameter("map_frame", map_frame_);
}
void InitialPointPublisher::drive_mode_callback(const robot_state_msgs::msg::DriveMode::SharedPtr msg)
{
    // Update drive mode state first
    switch (msg->drive_mode)
    {
        case 0:
            last_drive_mode_state_ = State::DriveMode::TELEOP;
            break;
        case 1:
            last_drive_mode_state_ = State::DriveMode::AUTONOMOUS;
            break;
    }

    // Check if we are switching in to autonomous and we have not already sent an initial_point
    if (last_drive_mode_state_ == State::DriveMode::AUTONOMOUS && !is_initialized)
    {
        // We need to publish the initial point at the position of the robot in the map frame
        // during the time which the drive mode is first transitioned to autonomous

        geometry_msgs::msg::PoseWithCovarianceStamped robot_pose;
        geometry_msgs::msg::PoseWithCovarianceStamped transformed_pose;

        // Dead-reckon the pose into the robot frame
        robot_pose.header.frame_id = map_frame_;
        robot_pose.header.stamp = this->get_clock()->now();
        robot_pose.pose.pose.position.x = 0.0; // object probably instantiates to 0.0, but included
        robot_pose.pose.pose.position.y = 0.0; // for clarity's sake when reading (shrug)
        robot_pose.pose.pose.position.z = 0.0;

        try
        {   // perform the tf transform and publish the resulting pose
            transformed_pose = tf_buffer_->transform(robot_pose, map_frame_, transform_tolerance_);
            initial_point_publisher_->publish(transformed_pose);
            is_initialized = true;
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                map_frame_.c_str(), robot_frame_.c_str(), ex.what()
            );
            return;
        }
    }
}
} // namespace RobotStateController

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto ipp_node = std::make_shared<RobotStateController::InitialPointPublisher>(options);
    exec.add_node(ipp_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
