#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "robot_state_controller/robot_state_controller_node.hpp"

TEST(RSCTests, SetStateWorks) {
    rclcpp::NodeOptions opts;
    RobotStateController::RobotStateController node{opts};

    auto req = robot_state_msgs::srv::SetState::Request::SharedPtr{};
    auto resp = robot_state_msgs::srv::SetState::Response::SharedPtr{};
    req->state.state = robot_state_msgs::msg::State::KILL;

    node.set_state_cb(req, resp);

    EXPECT_EQ(node.state.state, robot_state_msgs::msg::State::KILL);
}

TEST(RSCTests, SetStateTiming) {
    // Create an executor to run our nodes on in the bg
    rclcpp::NodeOptions opts;
    auto node = std::make_shared<RobotStateController::RobotStateController>(opts);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    auto done_cv = std::condition_variable{};
    auto exe_mut = std::mutex{};
    auto done_flag = std::atomic<bool>{false};

    // Create another node that runs waiting for state to change
    auto other_node = std::make_shared<rclcpp::Node>("test_node");
    auto sub = other_node->create_subscription<robot_state_msgs::msg::State>(
        "/robot/state", 10, [&done_cv, &done_flag, &exe_mut](robot_state_msgs::msg::State::SharedPtr state) {
            if (state->state == robot_state_msgs::msg::State::KILL) {
                {
                    std::unique_lock lk{exe_mut};
                    done_flag.store(true);
                }
                done_cv.notify_one();
            }
        });
    auto client = other_node->create_client<robot_state_msgs::srv::SetState>("/robot/set_state");

    exec.add_node(other_node);

    // Spin nodes in background
    auto handle = std::thread{[&]() { exec.spin(); }};

    // Send estop
    auto start = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());
    auto req = robot_state_msgs::srv::SetState::Request::SharedPtr{};
    auto resp = client->async_send_request(req);

    // Wait until sub gets update
    resp.wait();
    {
        std::unique_lock lk{exe_mut};
        while (!done_flag.load()) {
            done_cv.wait(lk);
        }
    }

    auto done = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());

    // Stop exe
    exec.cancel();

    // Estop should be less than a second
    EXPECT_TRUE((done - start).count() < 1000);
}

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);

    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return res;
}