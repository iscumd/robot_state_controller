#include "rclcpp/rclcpp.hpp"
#include "robot_state_controller/initial_point_publisher_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto ipp_node = std::make_shared<RobotStateController::InitialPointPublisher>(options);
    exec.add_node(ipp_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
