# Runtime mode ownership and transition state.

uint8 MODE_UNKNOWN=0
uint8 MODE_MAINTENANCE=1
uint8 MODE_INACTIVE=2
uint8 MODE_TELEOP=3
uint8 MODE_AUTONOMY=4
uint8 MODE_AI_POLICY=5
uint8 MODE_FAULT=6
uint8 MODE_SHUTDOWN=7

std_msgs/Header header
string robot_id

uint8 requested_mode
uint8 active_mode
string owner
string transition_reason
builtin_interfaces/Time active_since
