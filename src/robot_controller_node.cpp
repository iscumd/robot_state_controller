#include "ros/ros.h"
#include "ros/console.h"
#include "robot.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

class RobotControllerNode {
private:
	Robot m_robot; // robot object: see robot.h
	bool m_enableLogging; // param to enable logging
	double m_waitTime; // how much time, if any, before the robot can move to READY state

	ros::NodeHandle m_nh;
	ros::Publisher m_robotStatePub; // publish robot state
	ros::Subscriber m_joystickSub; // get joystick inputs
	ros::Subscriber m_killSub; // get kill signals
	ros::Subscriber m_pauseSub; // get pause signals
	ros::Subscriber m_softPauseSub; // software pause


public:
	RobotControllerNode() : m_nh("~") {
		m_nh.param("enable_logging", m_enableLogging, false);
		m_nh.param("startup_wait_time", m_waitTime, 0.0);
		update();
	}

	/*
		after startup commands:
		STARTUP -> READY

		TODO: maybe check state of boot sensitive nodes?
	*/
	void boot() {
		if (m_robot.getRobotState() != State::STARTUP) return;
		ROS_DEBUG_COND(m_enableLogging, "Booting...");

		m_robotStatePub = m_nh.advertise<std_msgs::String>("/state/robot", 5, true);
		m_killSub = m_nh.subscribe<std_msgs::Bool>("/signal/kill", 0, &RobotControllerNode::killCallback, this);
		m_pauseSub = m_nh.subscribe<std_msgs::Bool>("/signal/pause", 0, &RobotControllerNode::pauseCallback, this);
		m_softPauseSub = m_nh.subscribe<std_msgs::Bool>("/signal/soft_pause", 0, &RobotControllerNode::softPauseCallback, this);

		if (m_waitTime > 0.0) ros::Duration(m_waitTime).sleep();
		ROS_DEBUG_COND(m_enableLogging, "Ready.");
		m_robot.setRobotState(State::READY);
	}

	/*
		if in STARTUP, boot
	*/
	void update() {
		if (m_robot.getRobotState() == State::STARTUP) boot(); // run startup

		std_msgs::String state; // create msgs for state
		state.data = State::robotStateToString(m_robot.getRobotState()); // set state msg

		m_robotStatePub.publish(state); // publish state 
	}

	/*
		signal true -> KILL
		signal false & killed -> STARTUP
	*/
	void killCallback(const std_msgs::Bool::ConstPtr& killSignal) {
		if (killSignal->data) {
			m_robot.setRobotState(State::KILL);
			ROS_DEBUG_COND(m_enableLogging, "Robot killed.");
		} else if(m_robot.getRobotState() == State::KILL) {
			m_robot.setRobotState(State::STARTUP);
			ROS_DEBUG_COND(m_enableLogging, "Robot unkilled.");
		}
	}

	/*
		signal true -> PAUSE
		signal false & paused -> READY
	*/
	void pauseCallback(const std_msgs::Bool::ConstPtr& pauseSignal) {
		if (pauseSignal->data) {
			m_robot.setRobotState(State::PAUSE);
			ROS_DEBUG_COND(m_enableLogging, "Robot paused.");
		} else if(m_robot.getRobotState() == State::PAUSE) {
			m_robot.setRobotState(State::READY);
			ROS_DEBUG_COND(m_enableLogging, "Robot unpaused");
		}
	}

	/*
		signal true -> PAUSE
		signal false & paused -> READY
	*/
	void softPauseCallback(const std_msgs::Bool::ConstPtr& softPauseSignal) {
		if (softPauseSignal->data) {
			m_robot.setRobotState(State::SOFTPAUSE);
			ROS_DEBUG_COND(m_enableLogging, "Robot software paused.");
		} else if(m_robot.getRobotState() == State::SOFTPAUSE) {
			m_robot.setRobotState(State::READY);
			ROS_DEBUG_COND(m_enableLogging, "Robot software unpaused");
		}
	}
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_state_controller"); // just put the ros init in main, its easier that way
	RobotControllerNode rcn;

	ros::Rate rate(10);
	while (ros::ok()) {
		rcn.update();
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
