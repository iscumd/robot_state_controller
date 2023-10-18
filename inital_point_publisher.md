# inital_point_publisher
From package '[robot_state_controller](https://github.com/iscumd/robot_state_controller)'
# File
`./src/inital_point_publisher.cpp`

## Summary 
 Publishes the inital pose of the robot in a map when entering auton for the first time. This is intended to be used with
the rest of the robot_state_controller utilities and something like nav2.

Semantically, this node will transform pose (0,0,0) in the robot frame into the map frame, then publish.

## Topics

### Publishes
- `/initalpose`: Topic on which the inital pose will be published

### Subscribes
- `/robot/drive_mode`: Drive mode to query. Will publish inital pose the first time this switches to AUTONOMOUS.

