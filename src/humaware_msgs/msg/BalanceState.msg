# Balance state of a humanoid body.

uint8 STATE_UNKNOWN=0
uint8 STATE_STABLE=1
uint8 STATE_WARNING=2
uint8 STATE_UNSTABLE=3
uint8 STATE_FALLEN=4
uint8 STATE_RECOVERING=5

std_msgs/Header header
string robot_id
uint8 state

geometry_msgs/Vector3 center_of_mass
geometry_msgs/Vector3 zero_moment_point

float32 roll_rad
float32 pitch_rad
float32 yaw_rad
float32 stability_margin

string[] active_warnings
