cmake_minimum_required(VERSION 2.8.3)
project(robot_state_controller)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
)

include_directories(
	include
    ${catkin_INCLUDE_DIRS}
)

add_executable(robot_state_controller src/robot_controller_node.cpp)
add_dependencies(robot_state_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_state_controller ${catkin_LIBRARIES})
