#include "ros/ros.h"
#include "robot.cpp"
#include "std_msgs"
#include "geometry_msgs/Twist.h"

class RobotControllerNode {
private:
    Robot m_robot; // robot object: see robot.cpp
    bool m_startButtonDown; // used to trigger drive mode switch on falling edge
    ool m_enableLogging; // param to enable logging

    ros::NodeHandle m_nh;
    ros::Publisher m_robotStatePub; // publish robot state
    ros::Publisher m_driveModePub; // publish drive mode
    ros::Subscriber m_joystickSub; // get joystick inputs
    ros::Subscriber m_estopSub; // get estop signals


public:
    RobotControllerNode() {
        ros::init(argc, argv, "robot_state");
        nh.param("enable_logging", m_enableLogging, false);
        update();
    }

    /*
        after startup commands:
        STARTUP -> READY

        TODO: maybe check state of boot sensitive nodes?
    */
    void boot() {
        if (m_robot.getRobotState() != STARTUP) return;
        ROS_DEBUG_COND(enable_logging, "Booting...");
        m_robotStatePub = m_nh.advertise<std_msgs::String>("/state/robot", 5, true);
        m_joystickSub = m_nh.subscribe("/signal/drivemode", 0, joystickCallback);
        m_estopSub = m_nh.subscribe("/signal/estop", 0, estopCallback);
        m_robot.setRobotState(State::READY);
    }

    /*
        STARTUP -> BOOT
    */
    void update() {
        if (m_robot.getRobotState() == State::STARTUP) boot(); // run startup
        m_robotStatePub.publish(State::robotState2String(m_robot.getRobotState()));
        m_driveModePub.publish(State::driveMode2String(m_robot.getDriveMode());
        if (m_robot.getRobotState() != READY) return;
        // PUT COMMANDS WHICH RUN FOR KILL AND PAUSE HERE.
    }

    /* Toggles drive mode on Start press. The signal is true, when the button to
    switch drive mode is pressed. This triggers the state change below.
    */
    void joystickCallback(const std_msgs::Bool::ConstPtr& joySignal) {
        if (joySignal->data) {
            if (m_robot.getDriveMode() == State::MANUAL) {
                m_robot.setDriveMode(State::AUTONOMOUS);
                ROS_DEBUG_COND(enable_logging, "Robot set to auto.");
            } else if (m_robot.getDriveMode() == State::AUTONOMOUS) {
                m_robot.setDriveMode(State::MANUAL);
                ROS_DEBUG_COND(enable_logging, "Robot set to manual.");
            } else {
                ROS_DEBUG_COND(enable_logging, "Drive mode is not manual or auto."); //panic
            }
        }
    }

    /*
        signal true -> KILL
        signal false & killed -> STARTUP
    */
    void estopCallback(const std_msgs::Bool::ConstPtr& estopSignal) {
        if (estopSignal->data) {
            m_robot.kill();
            ROS_DEBUG_COND(enable_logging, "Robot killed.");
        }
        else if(m_robot.getRobotState() == State::KILL)
            m_robot.setRobotState(State::STARTUP);
            ROS_DEBUG_COND(enable_logging, "Robot unkilled.");
    }
};

int main(int argc, char **argv) {

    RobotControllerNode rcn;

    ros::Rate rate(10);
    while (ros::ok()) {
        rcn.update();
        spinOnce();
        rate.sleep();
    }
    return 0;
}
