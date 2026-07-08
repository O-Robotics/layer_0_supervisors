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
The simulation launch starts Gazebo, spawns the robot from the shared description package, bridges Gazebo topics into ROS 2, and exposes simulation-only transport topics under `/amr_sweeper/simulation/*`. Public robot-facing topics such as `/amr_sweeper/gnss/*` remain owned by the hardware-layer packages so the rest of the stack can run unchanged in simulation and on the real robot.

## Notes
- World files, bridge mappings, and simulation helper scripts now live in this package instead of `amr_sweeper_bringup`.
- The bridge namespace keeps simulation plumbing under `/amr_sweeper/simulation/*`, including pose, IMU, and depth-camera inputs.
- `amr_sweeper_bringup` still owns top-level orchestration and system-level rosbag recording; it now includes this package when `use_simulation:=true`.
