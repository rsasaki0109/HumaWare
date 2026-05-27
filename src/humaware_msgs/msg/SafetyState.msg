# Operational safety state.

uint8 STATE_UNKNOWN=0
uint8 STATE_OK=1
uint8 STATE_WARN=2
uint8 STATE_FAULT=3
uint8 STATE_ESTOP=4
uint8 STATE_MRM=5

std_msgs/Header header
string robot_id

uint8 state
bool estop_engaged
bool mrm_active

builtin_interfaces/Time last_heartbeat

string[] active_faults
string[] active_warnings
