# Last runtime mode transition request and outcome.

uint8 OUTCOME_UNKNOWN=0
uint8 OUTCOME_ACCEPTED=1
uint8 OUTCOME_REJECTED=2

std_msgs/Header header
string robot_id

uint8 previous_mode
uint8 requested_mode
uint8 active_mode

string requester
string reason
uint8 outcome
string message
bool takeover
