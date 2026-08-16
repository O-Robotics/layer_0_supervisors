# amr_sweeper_msgs

```bash
find src/layer_0_supervisors/amr_sweeper_msgs -maxdepth 2 -name package.xml
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_safety_msgs`
- `amr_sweeper_system_info_msgs`

## Purpose
This directory groups the shared AMR Sweeper interface packages under one layer 1 folder.

## Overview
`amr_sweeper_msgs` is a package container folder rather than a buildable ROS package. It keeps the interface packages together while preserving separate package names for safety and system-information messages.

## Notes
- `amr_sweeper_safety_msgs` lives in `src/layer_0_supervisors/amr_sweeper_msgs/amr_sweeper_safety_msgs/`.
- `amr_sweeper_system_info_msgs` lives in `src/layer_0_supervisors/amr_sweeper_msgs/amr_sweeper_system_info_msgs/`.
