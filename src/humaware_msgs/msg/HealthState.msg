# Aggregated runtime health summary.

uint8 HEALTH_UNKNOWN=0
uint8 HEALTH_OK=1
uint8 HEALTH_WARN=2
uint8 HEALTH_ERROR=3
uint8 HEALTH_STALE=4

std_msgs/Header header
string robot_id

uint8 state
string summary

uint8 mode
uint8 safety_state
uint8 locomotion_state
uint8 command_source
uint8 nav2_bridge_state

bool command_output_enabled
bool nav2_output_enabled

string[] active_faults
string[] active_warnings
string[] stale_topics
