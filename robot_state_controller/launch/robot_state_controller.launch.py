# MIT License
#
# Copyright (c) 2022 Intelligent Systems Club
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # ROS packages
    get_package_share_directory('robot_state_controller')

    # Launch arguments
    switch_button = LaunchConfiguration('switch_button', default='8')
    init_value = LaunchConfiguration('init_value', default='teleop')

    robot_state_controller = Node(
        package='robot_state_controller',
        executable='robot_state_controller',
        name='robot_state_controller',
        output='screen'
        )

    drive_mode_switch = Node(
        package='robot_state_controller',
        executable='drive_mode_switch',
        name='drive_mode_switch',
        output='screen',
        parameters=[{
            'switch_button': switch_button,
            'init_value': init_value
        }])

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('switch_button',
                              default_value='8',
                              description='Joy button which triggers a drive mode switch event'),
        DeclareLaunchArgument('init_value',
                              default_value='teleop',
                              description='Starting state of drive mode switch'),

        # Nodes
        robot_state_controller,
        drive_mode_switch,
    ])
