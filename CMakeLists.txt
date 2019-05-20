cmake_minimum_required(VERSION 2.8.3)
project(robot_state_controller)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
	geometry_msgs
	isc_joy
)

include_directories(
	include
    ${catkin_INCLUDE_DIRS}
)

## build the state controller
add_executable(robot_state_controller src/robot_controller_node.cpp)
add_dependencies(robot_state_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_state_controller ${catkin_LIBRARIES})

## build the drive mode switch
add_executable(drive_mode_switch src/drive_mode_switch/drive_mode_switch.cpp)
add_dependencies(drive_mode_switch ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drive_mode_switch ${catkin_LIBRARIES})
