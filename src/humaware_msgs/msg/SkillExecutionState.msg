# Last skill execution request handled by the skill server.

uint8 STATUS_UNKNOWN=0
uint8 STATUS_REJECTED=1
uint8 STATUS_ACCEPTED=2
uint8 STATUS_EXECUTING=3
uint8 STATUS_COMPLETED=4

std_msgs/Header header
string robot_id

string execution_id
string capability_name
string requester

uint8 status
string message

builtin_interfaces/Time accepted_at
builtin_interfaces/Duration timeout

bool command_published
