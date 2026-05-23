# Represents a single scheduled execution window
# Published by scheduler for mission execution / Foxglove consumption

string robot_id               # Robot identifier (e.g., RBT-01)
string uid                    # Unique event ID from ICS (stable across recurrences)
string window_type            # WORK or NO_WORK
builtin_interfaces/Time start_time   # Window start
builtin_interfaces/Time end_time     # Window end
string mission_id             # Mission to execute (only for WORK windows)
string description            # Human-readable summary
