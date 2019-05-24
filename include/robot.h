#include "string"

class State {

public:
	/*
	Startup state is the default state, which signals that the robot is
	still in the process of booting up
	Ready state means the robot is available to take commands
	Kill is when the robot is emergency stopped and power is cut to the motors
	Pause is soft kill, which means that power is not cut, but the robot
	will not take commands
	*/
	enum RobotState {
		STARTUP,
		READY,
		KILL,
		PAUSE,
		SOFTPAUSE
	};

	static std::string robotStateToString(State::RobotState robotState) {
		switch(robotState) {
		case STARTUP: return "startup";
		case READY: return "ready";
		case KILL: return "kill";
		case SOFTPAUSE: return "softpause";
		case PAUSE:
		default: return "pause";
		}
	}
};

class Robot {
private:
	float m_batteryLevel;
	State::RobotState m_robotState;

public:
	Robot() {
		m_batteryLevel = 0.0;
		m_robotState = State::STARTUP;
	}

	Robot(float batteryLevel, State::RobotState robotState) {
		m_batteryLevel = batteryLevel;
		m_robotState = robotState;
	}

	int getBatteryLevel() {
		return m_batteryLevel;
	}

	void setBatteryLevel(float batteryLevel) {
		m_batteryLevel = batteryLevel;
	}

	State::RobotState getRobotState() {
		return m_robotState;
	}

	void setRobotState(State::RobotState robotState) {
		m_robotState = robotState;
	}

	~Robot() {}
};
