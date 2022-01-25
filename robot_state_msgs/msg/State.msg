# This message holds system state which is used internally to the robot state robot_state_controller

# Possible system states
byte KILL=0
byte PAUSE=1
byte ACTIVE=2

# System state enumerated above
byte state

# Possible error states
byte NONE=0
byte SENSOR_TIMEOUT=1
byte CONTROLLER_TIMEOUT=2
byte BATTERY_LOW=3

# Error state enumerated above
byte error