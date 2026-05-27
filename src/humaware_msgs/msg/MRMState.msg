# Minimal risk maneuver state.

uint8 STATE_NONE=0
uint8 STATE_STOP=1
uint8 STATE_STABILIZE=2
uint8 STATE_SIT=3
uint8 STATE_POWER_LIMITED=4
uint8 STATE_REQUEST_TELEOP=5
uint8 STATE_FAULT=6

std_msgs/Header header
string robot_id

uint8 state
string reason
builtin_interfaces/Duration timeout
bool operator_intervention_requested
