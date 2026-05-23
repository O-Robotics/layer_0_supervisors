###############################################################################
# FSMStatus.msg
#
# Purpose:
#   Supervisor status snapshot for the AMR Sweeper FSM.
#
# Format requested by user:
#   builtin_interfaces/Time stamp
#   string current_state
#   string current_lifecycle_state
#   uint16 current_profile
#   uint16 transitioning_to_profile
#   string transition_status
#   string last_requester
#   uint8 last_request_priority
#   uint8 effective_priority_gate
#   float32 priority_age_sec
#   string last_message
###############################################################################

builtin_interfaces/Time stamp

string current_state
string current_lifecycle_state
uint16 current_profile

uint16 transitioning_to_profile
string transition_status

string last_requester
uint8 last_request_priority
uint8 effective_priority_gate
float32 priority_age_sec
string last_message