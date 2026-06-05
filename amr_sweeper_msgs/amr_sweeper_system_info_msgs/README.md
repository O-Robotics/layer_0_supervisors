# amr_sweeper_system_info_msgs

```bash
ros2 interface show amr_sweeper_system_info_msgs/msg/SystemState
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package defines the shared ROS system-information message used by the AMR Sweeper monitoring path.

## Interfaces
- `SystemState.msg`

## Overview
`amr_sweeper_system_info_msgs` is the system-information interface package inside the shared `amr_sweeper_msgs/` folder. It provides the message published by the layer 1 system monitoring node.

## Notes
- `SystemState.msg` carries robot identity, health, compute, storage, and connectivity status fields.
