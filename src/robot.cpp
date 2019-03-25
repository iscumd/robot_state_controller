#include "string"

namespace ISC {

namespace State {
    /*
    Manual mode is when the robot is getting direct input from the joystick
    Autonomous mode activates self navigation
    */
    enum DriveMode {
        MANUAL,
        AUTONOMOUS
    };

    static std::string driveMode2String(State::DriveMode driveMode) {
        switch(driveMode) {
        case AUTONOMOUS: return "auto";
        case MANUAL:
        default: return "manual";
        }
    }

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
        PAUSE
    };

    static std::string robotState2String(State::RobotState robotState) {
        switch(robotState) {
        case STARTUP: return "startup";
        case READY: return "ready";
        case KILL: return "kill";
        case PAUSE:
        default: return "pause";
        }
    }
}

class Robot {
private:
    float m_batteryLevel;
    DriveMode m_driveMode;
    RobotState m_robotState;

public:
    Robot() {
        m_batteryLevel = 0f;
        m_driveMode = MANUAL;
        m_robotState = STARTUP;
    }

    Robot(float batteryLevel, DriveMode driveMode, RobotState robotState) {
        m_batteryLevel = batteryLevel;
        m_driveMode = driveMode;
        m_robotState = robotState;
    }

    void kill() {
        m_robotState = State::KILL;
    }

    int getBatteryLevel() {
        return m_batteryLevel;
    }

    void setBatteryLevel(float& batteryLevel) {
        m_batteryLevel = batteryLevel;
    }

    DriveMode getDriveMode() {
        return m_driveMode;
    }

    void setDriveMode(DriveMode& driveMode) {
        m_driveMode = driveMode;
    }

    RobotState getRobotState() {
        return m_robotState;
    }

    void setRobotState(RobotState& robotState) {
        m_driveMode = driveMode;
    }

    ~Robot() {}
};
}
