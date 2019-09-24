#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <isc_joy/xinput.h>

#include <string>

/*
	Manual mode is when the robot is getting direct input from the joystick
	Autonomous mode activates self navigation
*/
namespace drive_mode { // <enum>::<identifier> doesn't compile outside of MSVC. i did not know that. -matt
	enum state {
		MANUAL,
		AUTONOMOUS
	};
	
	std::string drive_mode_to_string(drive_mode::state mode) {
		switch(mode) {
		case AUTONOMOUS: return "auto";
		case MANUAL:
		default: return "manual";
		}
	}
}

// variables

drive_mode::state mode;

bool start_pressed = false;

std::string robot_state;
std::string kill_state;
std::string normal_state;

ros::Publisher control_pub;
ros::Publisher drive_mode_pub;

ros::Subscriber state_sub;
ros::Subscriber auto_sub;
ros::Subscriber manual_sub;
ros::Subscriber joystick_sub;

// functions

void publish_drive_mode() {
	std_msgs::String msg;
	msg.data = drive_mode::drive_mode_to_string(mode);
	drive_mode_pub.publish(msg);
}

void joystick_cb(const isc_joy::xinput::ConstPtr &joy) {
	if(joy->Start) {
		start_pressed = true;
	} else if (start_pressed && !joy->Start) {
		start_pressed = false;

		if (mode == drive_mode::AUTONOMOUS) {
			mode = drive_mode::MANUAL;
		} else if (mode == drive_mode::MANUAL && robot_state == normal_state) {
			mode = drive_mode::AUTONOMOUS;
		}

		publish_drive_mode();
	}
}

void state_cb(const std_msgs::String::ConstPtr &state) {
	robot_state = state->data;

	if (robot_state == kill_state && mode == drive_mode::AUTONOMOUS) {
		mode = drive_mode::MANUAL;
		publish_drive_mode();
	}
}

void auto_control_cb(const geometry_msgs::Twist::ConstPtr &control) {
	if (mode == drive_mode::AUTONOMOUS) {
		control_pub.publish(*control);
	}
}

void manual_control_cb(const geometry_msgs::Twist::ConstPtr &control) {
	if (mode == drive_mode::MANUAL) {
		control_pub.publish(*control);
	}
}

// main

int main(int argc, char **argv) {
	ros::init(argc, argv, "drive_mode_switch");
	
	ros::NodeHandle nh;

	// get params for state names
	nh.param("kill_state_string", kill_state, std::string("kill"));
	nh.param("ready_state_string", normal_state, std::string("ready"));

	// set up controls in and control out, and drive mode pub / state sub
	control_pub = nh.advertise<geometry_msgs::Twist>("control_vel", 1);
	drive_mode_pub = nh.advertise<std_msgs::String>("drive_mode", 1);

	state_sub = nh.subscribe("robot_state", 1, &state_cb);
	joystick_sub = nh.subscribe("joystick", 1, &joystick_cb);
	auto_sub = nh.subscribe("auto_control_vel", 1, &auto_control_cb);
	manual_sub = nh.subscribe("manual_control_vel", 1, &manual_control_cb);

	ros::spin();

	return 0;
}
