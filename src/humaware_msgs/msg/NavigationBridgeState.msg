# State emitted by a navigation bridge.

uint8 STATE_UNKNOWN=0
uint8 STATE_IDLE=1
uint8 STATE_ACTIVE=2
uint8 STATE_BLOCKED=3
uint8 STATE_STALE=4
uint8 STATE_DISABLED=5

std_msgs/Header header
string robot_id

uint8 state
uint8 active_mode
uint8 safety_state
uint8 locomotion_state

bool bridge_enabled
bool output_enabled

string input_topic
string output_topic

builtin_interfaces/Time last_input_time
builtin_interfaces/Time last_output_time
builtin_interfaces/Duration command_age

string reason
