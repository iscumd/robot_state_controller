#include "rclcpp/rclcpp.hpp"
#include "robot_state_controller/robot_state_controller_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto rsc_node = std::make_shared<RobotStateController::RobotStateController>(options);
    exec.add_node(rsc_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}