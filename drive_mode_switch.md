# drive_mode_switch
From package '[robot_state_controller](https://github.com/iscumd/robot_state_controller)'
# File
`./src/drive_mode_switch.cpp`

## Summary 
 Switches over a stream of autonomous drive commands and teleop drive commands, selecting the one matching the current robots state.
It is expected that a joystick provides the teleop source, and thus a button is configured that will swap the robot from autonomous
to teleop operation when pressed.
This node is aware of robot state, and will stop forwarding either stream if the robot is not in the ACTIVE state.

This node will boot in ACTIVE and TELEOP.

## Topics

### Publishes
- `/robot/cmd_vel`: Control messages from whichever stream is currently selected.
- `/robot/drive_mode`: Current drive mode of the system. AUTONOMOUS or TELEOP.

### Subscribes
- `/robot/state`: Current robot state. Will stop publishing /robot/cmd_vel when not ACTIVE.
- `/joy`: Joy topic to listen for drive mode switch button.
- `/nav_vel`: Twist commands from the autonomous system. This will be forwarded to /robot/cmd_vel if the drive mode is AUTONOMOUS
- `/cmd_vel`: Twist commands from the teleop source. TThis will be forwarded to /robot/cmd_vel if the drive mode is TELEOP

