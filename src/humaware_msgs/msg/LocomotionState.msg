# Locomotion runtime state.

uint8 STATE_UNKNOWN=0
uint8 STATE_INACTIVE=1
uint8 STATE_STANDING=2
uint8 STATE_WALKING=3
uint8 STATE_TURNING=4
uint8 STATE_STOPPING=5
uint8 STATE_RECOVERING=6
uint8 STATE_FAULT=7

std_msgs/Header header
string robot_id
uint8 state

string active_adapter

float32 commanded_linear_velocity_mps
float32 commanded_angular_velocity_radps
float32 max_safe_linear_velocity_mps
float32 max_safe_angular_velocity_radps

bool balance_required
bool navigation_control_active
string[] active_constraints
