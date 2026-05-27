# Top-level runtime state for a humanoid robot.

std_msgs/Header header
string robot_id

uint8 mode
uint8 locomotion_state
uint8 safety_state
uint8 balance_state

float32 battery_percentage
float32 cpu_temperature_c
bool operator_present

geometry_msgs/PoseWithCovariance base_pose
geometry_msgs/TwistWithCovariance base_twist

string[] active_capabilities
string[] active_faults
