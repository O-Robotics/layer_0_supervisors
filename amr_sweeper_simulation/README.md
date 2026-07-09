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
The simulation launch starts Gazebo, spawns the robot from the shared description package, bridges Gazebo topics into ROS 2, and uses explicit `/gazebo/*` transport topics inside Gazebo and bridges them onto `/amr_sweeper/simulation/*` in ROS before the hardware-layer adapters publish the final robot-facing topics expected by the rest of the stack. Public robot-facing topics such as `/amr_sweeper/imu/data_raw`, `/amr_sweeper/depth_camera/depth/image_rect_raw`, and `/amr_sweeper/gnss/*` therefore stay consistent between simulation and hardware.

## Notes
- World files, bridge mappings, and simulation helper scripts now live in this package instead of `amr_sweeper_bringup`.
- Gazebo-side transport topics now live under `/gazebo/*`, such as `/gazebo/imu/data_raw`, `/gazebo/depth_camera/*`, `/gazebo/drive_controller/odom`, and `/gazebo/sweeping_controller/cmd_vel_drive`, while the bridged ROS-side simulation topics stay under `/amr_sweeper/simulation/*` and the final public topics remain identical to the real robot stack.
- `amr_sweeper_bringup` still owns top-level orchestration and system-level rosbag recording; it now includes this package when `use_simulation:=true`.
