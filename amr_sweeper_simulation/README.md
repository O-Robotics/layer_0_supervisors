# amr_sweeper_simulation

```bash
ros2 launch amr_sweeper_simulation amr_sweeper_simulation.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_description`
- `amr_sweeper_fsm`
- `amr_sweeper_gnss`

## Purpose
This package owns the Gazebo-based AMR Sweeper simulation setup. It contains the simulation launch entrypoint, Gazebo worlds, bridge configuration, and helper scripts used by simulation runs.

## Main Launch File
`launch/amr_sweeper_simulation.launch.py`

## Overview
The simulation launch starts Gazebo, spawns the robot from the shared description package, bridges Gazebo sensor and pose topics into ROS 2, and keeps the normal ROS 2 controller stack active so public robot-facing topics stay consistent between simulation and hardware. Public topics such as `/amr_sweeper/drive_controller/odom`, `/amr_sweeper/imu/data_raw`, `/amr_sweeper/depth_camera/depth/image_rect_raw`, and `/amr_sweeper/gnss/*` stay consistent between simulation and hardware.

## Notes
- World files, bridge mappings, and simulation helper scripts now live in this package instead of `amr_sweeper_bringup`.
- Gazebo-side transport topics now live under `/gazebo/*`, such as `/gazebo/imu/data_raw`, `/gazebo/depth_camera/*`, and `/gazebo/sweeping_controller/cmd_vel_drive`, while `/amr_sweeper/drive_controller/odom` is published only by the normal ROS 2 drive controller.
- `amr_sweeper_bringup` still owns top-level orchestration and system-level rosbag recording; it now includes this package when `use_simulation:=true`.
