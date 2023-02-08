#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "robot_state_controller/robot_state_controller_node.hpp"

//TODO unit test or something

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);

    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return res;
}