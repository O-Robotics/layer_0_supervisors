# amr_sweeper_safety_msgs

```bash
ros2 interface show amr_sweeper_safety_msgs/msg/SafetyStop
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package defines the shared ROS safety-stop message used across the AMR Sweeper stack.

## Interfaces
- `SafetyStop.msg`

## Overview
`amr_sweeper_safety_msgs` is the safety interface package inside the shared `amr_sweeper_msgs/` folder. It provides the stop-event message used by controller-layer safety publishers and consumers.

## Notes
- `SafetyStop.msg` carries stop timestamp, sender, and human-readable reason text.
