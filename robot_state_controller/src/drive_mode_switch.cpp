#include "rclcpp/rclcpp.hpp"
#include "robot_state_controller/drive_mode_switch_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto dms_node = std::make_shared<RobotStateController::DriveModeSwitch>(options);
    exec.add_node(dms_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
