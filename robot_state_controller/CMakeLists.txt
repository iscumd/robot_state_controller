cmake_minimum_required(VERSION 3.5)
project(robot_state_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_state_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(robot_state_controller src/robot_state_controller.cpp)
ament_target_dependencies(
  robot_state_controller
  robot_state_msgs
  rclcpp
  std_msgs
)
target_include_directories(robot_state_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(drive_mode_switch src/drive_mode_switch.cpp)
ament_target_dependencies(
  drive_mode_switch
  robot_state_msgs
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)
target_include_directories(drive_mode_switch PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(initial_point_publisher src/initial_point_publisher.cpp)
ament_target_dependencies(
  initial_point_publisher
  robot_state_msgs
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_include_directories(initial_point_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS robot_state_controller
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS drive_mode_switch
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS initial_point_publisher
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
