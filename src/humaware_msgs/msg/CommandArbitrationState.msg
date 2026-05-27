# State emitted by the command arbiter.

uint8 SOURCE_NONE=0
uint8 SOURCE_SAFETY=1
uint8 SOURCE_TELEOP=2
uint8 SOURCE_AUTONOMY=3
uint8 SOURCE_AI_POLICY=4
uint8 SOURCE_DEMO=5

std_msgs/Header header
string robot_id

uint8 active_source
string active_source_name
uint8 active_mode
uint8 safety_state

bool output_enabled
bool stop_command_published

builtin_interfaces/Time last_command_time
builtin_interfaces/Duration command_age

string reason
