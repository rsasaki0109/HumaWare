# Foot contact and support information.

std_msgs/Header header
string robot_id

bool left_contact
bool right_contact

float32 left_normal_force_n
float32 right_normal_force_n

geometry_msgs/Point left_center_of_pressure
geometry_msgs/Point right_center_of_pressure

float32 left_slip_ratio
float32 right_slip_ratio
