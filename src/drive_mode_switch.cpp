#include <ros/ros.h>
#include <_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <isc_joy/xinput.h>
#include "stallDetectionHeader.h"

#include <string>

//imports for stall detection
#include <boost/chrono.hpp>
#include <iostream>
#include <deque>
#include <queue>
#include <math.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <string.h>

//https://stackoverflow.com/a/4974588 but use boost instead of chrono

typedef boost::chrono::high_resolution_clock Clock;
typedef boost::chrono::milliseconds Milliseconds;

/*
	Manual mode is when the robot is getting direct input from the joystick
	Autonomous mode activates self navigation
	Stalling is for when the tires slip or when the robot otherwise can't drive forward
*/
namespace drive_mode { // <enum>::<identifier> doesn't compile outside of MSVC. i did not know that. -matt
	enum state {
		MANUAL,
		AUTONOMOUS,
		STALLING
	};

	std::string drive_mode_to_string(drive_mode::state mode) {
		switch (mode) {
		case AUTONOMOUS: return "auto";
		case STALLING: return "stall";
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
ros::Publisher stallVelocityPub

ros::Subscriber state_sub;
ros::Subscriber auto_sub;
ros::Subscriber manual_sub;
ros::Subscriber stall_sub;
ros::Subscriber joystick_sub;

StallDetection stallDetector;
geometry_msgs::Twist stallVelocity;

// functions

void publish_drive_mode() {
	std_msgs::String msg;
	msg.data = drive_mode::drive_mode_to_string(mode);
	drive_mode_pub.publish(msg);
}

void joystick_cb(const isc_joy::xinput::ConstPtr &joy) {
	if (joy->Start) {
		start_pressed = true;
	}
	else if (start_pressed && !joy->Start) {
		start_pressed = false;

		//if the robot is in autonomous mode or stalling and we press start, switch to manual
		if (mode == drive_mode::AUTONOMOUS || mode == drive_mode::STALLING) {
			mode = drive_mode::MANUAL;
		}
		else if (mode == drive_mode::MANUAL && robot_state == normal_state) {
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

void stall_control_cb(const geometry_msgs::Twist::ConstPtr &control) {
	if (mode == drive_mode::STALLING) {
		control_pub.publish(*control);

	}

}

//Above function overridden, switching back to autonomous mode
//This should only be called when in AUTONOMOUS or STALLING modes
void stall_control_cb() {

	//prevent EStop from backing up with this
		//TODO: delete once EStop bool exists
		bool EStop = false;

	//check if we're stalling
	if ( (mode == drive_mode::STALLING)  && (mode != drive_mode::MANUAL) ) {
		//if we are, try going forward again
		//if we're going slower than we expect or we're estopped...
		if ( (EStop) || (stallDetector.getExpectedVelocity() <= stallDetector.getMaxSpeed() ) )//if we are, try going forward again
		{
			//TODO: test getExpectedVelocity in EStop scenarios for backups

			//if we can go forward, switch back to drive_mode::AUTONOMOUS;
			//ie, do nothing
		}
		//otherwise back up, then switch back to drive_mode::AUTONOMOUS;
		else
		{
			stallDetector.doBackup(stallVelocity);

		}
		mode = drive_mode::AUTONOMOUS;
	}

	publish_drive_mode();
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
	stall_sub = nh.subscribe("stall_control_vel", 1, &stall_control_cb);

	//stall detection
	{
		ros::NodeHandle handledNode;
		double reverseDurationInMilli = stallDetector.getReverseDurationInMilli();
		boost::posix_time::time_duration td = milliseconds(stallDetector.reverseDurationInMilli);
		stallVelocityPub = handledNode.advertise<geometry_msgs::Twist>("/stall/velocity", 1000);

		while (ros::ok())
		{
			//spinOnce is a callback function that cycles through all
			//the callback functions waiting to be called in the queue 
			ros::spin();
			boost::posix_time::ptime lastStuckTime(stallDetector.getLastStuckTime() + td);
			boost::posix_time::ptime now(second_clock::local_time());
			if (stallDetector.getStallStatus())
			{
				if (lastStuckTime >= now)
				{
					mode = drive_mode::STALLING;
					stall_control_cb();
					stallDetector.doBackup(stallVelocity);
					stallVelocityPub.publish(stallVelocity);
				}
			}
		}
		
	}

	return 0;
}
