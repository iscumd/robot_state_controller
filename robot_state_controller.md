# robot_state_controller
From package '[robot_state_controller](https://github.com/iscumd/robot_state_controller)'
# File
`./src/robot_state_controller.cpp`

## Summary 
 Encpasulates the current operating state of the robot. These are states like: ACTIVE, KILLED, PAUSED ect. These can be
modified by a service, and consumed via a publisher.

## Topics

### Publishes
- `/robot/state`: The current state of the robot, published on a set interval (currently 100hz)
- `/robot/set_state`: A service that sets the robots internal state to the passed state. Note that this node will do nothing but republish this state for other nodes.

