###############################################################################
# FSMState.msg
#
# Purpose:
#   Lightweight FSM state publication for UI dashboards and logging.
#
# Fields:
#   stamp           Timestamp from supervisor clock.
#   current_state   High-level FSM state name (e.g. "RUNNING").
#   current_profile Active profile id for the current_state.
###############################################################################

builtin_interfaces/Time stamp

string current_state
uint16 current_profile
