# Runtime capability contract exposed by a humanoid robot.

uint8 STATE_UNKNOWN=0
uint8 STATE_UNAVAILABLE=1
uint8 STATE_IDLE=2
uint8 STATE_EXECUTING=3
uint8 STATE_DEGRADED=4
uint8 STATE_FAULT=5

string name
uint8 state

string required_mode
string owner_node
string[] required_hardware

string input_type
string output_type
builtin_interfaces/Duration timeout

string[] safety_constraints
string recovery_behavior
