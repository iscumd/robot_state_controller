# robot_state_controller
A simple robot state controller for ISC robots. It includes states for manual and auto drive mode, which is controlled by a signal from the controller. It includes various states for the robot, such as an  emergency stop state to be used with software and electrical emergency stop. There are also states to be used to boot the robot, and signal that it the robot is ready to take commands.

## Installation
Clone the repository into your ros workspace and `colcon build`. If you're using this package with an existing ISC robot, then it should be included in the robot's main .repos file.
