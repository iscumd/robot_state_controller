cmake_minimum_required(VERSION 3.5)
project(robot_state_controller)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

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

add_executable(robot_state_controller src/robot_state_controller.cpp src/robot_state_controller_node.cpp)
set(robot_state_controller_deps
        robot_state_msgs
        rclcpp
        std_msgs
        )

ament_target_dependencies(
        robot_state_controller
        ${robot_state_controller_deps}
)
target_include_directories(robot_state_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(drive_mode_switch src/drive_mode_switch.cpp src/drive_mode_switch_node.cpp)
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

add_executable(initial_point_publisher src/initial_point_publisher.cpp src/initial_point_publisher_node.cpp)
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

if (BUILD_TESTING)
    # Manually invoke clang format so it actually uses our file
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

    ### NOTE: these unit tests are currently disabled due to the custom service messages causing a segfault in ROS ###

    #find_package(ament_cmake_gtest REQUIRED)

    # Add unit tests
    #ament_add_gtest(robot_state_controller-test
    #        test/robot_state_controller_unit.cpp
    #        src/robot_state_controller_node.cpp
    #        )
    #ament_target_dependencies(robot_state_controller-test ${robot_state_controller_deps})
    #target_include_directories(robot_state_controller-test PUBLIC
    #        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    #        $<INSTALL_INTERFACE:include>)
endif ()

ament_package()
