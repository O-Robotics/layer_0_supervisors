# layer_0_supervisors

```
ros2 launch amr_sweeper_bringup amr_sweeper_bringup.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_bringup`
- `amr_sweeper_simulation`
- `amr_sweeper_fsm`
- `amr_sweeper_mission_executor`
- `amr_sweeper_scheduler`
- `amr_sweeper_vda5050_parser`
- `amr_sweeper_interface_server`
- `amr_sweeper_layer_1_hardware_bringup`
- `amr_sweeper_layer_2_controllers_bringup`
- `amr_sweeper_layer_3_navigation_bringup`

## Purpose
This repository is the supervision and orchestration layer for the AMR Sweeper. It owns the robot-level FSM, mission activation APIs, the always-on operator web server, schedule-driven handoff into RUNNING, and background VDA5050 mission preparation.

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`
- `use_profile`: default `001`
- `tick_period_ms`: default `100`
- `state_params_file`: default `amr_sweeper_fsm/config/state_parameters.yaml`

## Overview
Layer 0 sits above layers 1, 2, and 3. It decides which stack profile should be active, requests lifecycle transitions through the FSM supervisor, prepares mission execution folders, routes manual and scheduled missions into the correct RUNNING profile, and keeps VDA5050 mission artifacts ready before navigation needs them.

The main packages in this layer are:
- `amr_sweeper_bringup`: top-level layer 0 launch that starts the FSM, mission executor, scheduler, VDA5050 parser, and web server
- `amr_sweeper_simulation`: Gazebo simulation package with worlds, bridge mappings, and simulation helper scripts
- `amr_sweeper_fsm`: robot-level supervisor plus the `INITIALIZING`, `IDLING`, `RUNNING`, `CHARGING`, and `FAULT` lifecycle state nodes
- `amr_sweeper_mission_executor`: mission APIs, built-in manual mission routing, execution-folder preparation, and mission finalization back to `IDLING`
- `amr_sweeper_scheduler`: iCalendar-based work scheduling and scheduled mission handoff into the mission executor
- `amr_sweeper_vda5050_parser`: background parsing and artifact generation for VDA5050 mission files under `/missions`
- `amr_sweeper_interface_server`: always-on HTTP operator dashboard for mission launch, status, and VDA5050 upload

## Notes
- The default command launches the full layer 0 supervisor stack, including the always-on web server.
- Profile `001` is the full startup validation path and must not execute missions, profile `101` is the normal `IDLING` stack, profile `201` and related `2xx` profiles are mission-time `RUNNING` stacks, and profile `400` is the terminal empty `FAULT` profile.
- Layer 0 is responsible for coordinating the lower layers, not for driving hardware directly.
- The bridge profiles `000 -> 100 -> 200 -> 300 -> 400` are empty pass-through steps.
- Runtime mission history and generated artifacts are maintained under `/missions`.
