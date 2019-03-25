cmake_minimum_required(VERSION 2.8.3)
project(robot_state_controller)

find(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    isc_joy
    roscpp
    rosconsole
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
