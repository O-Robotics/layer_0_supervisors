# layer_0_supervisors

```
ros2 launch amr_sweeper_fsm amr_sweeper_layer_0_fsm.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_fsm`
- `amr_sweeper_mission_executor`
- `amr_sweeper_scheduler`
- `amr_sweeper_vda5050_parser`
- `amr_sweeper_layer_1_hardware_bringup`
- `amr_sweeper_layer_2_controllers_bringup`
- `amr_sweeper_layer_3_navigation_bringup`

## Purpose
This repository is the supervision and orchestration layer for the AMR Sweeper. It owns the robot-level FSM, mission activation APIs, schedule-driven handoff into RUNNING, and background VDA5050 mission preparation.

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`
- `start_profile`: default `001`
- `tick_period_ms`: default `100`
- `state_params_file`: default `amr_sweeper_fsm/config/state_parameters.yaml`

## Overview
Layer 0 sits above layers 1, 2, and 3. It decides which stack profile should be active, requests lifecycle transitions through the FSM supervisor, prepares mission execution folders, routes manual and scheduled missions into the correct RUNNING profile, and keeps VDA5050 mission artifacts ready before navigation needs them.

The main packages in this layer are:
- `amr_sweeper_fsm`: robot-level supervisor plus the `INITIALIZING`, `IDLING`, `RUNNING`, `CHARGING`, and `FAULT` lifecycle state nodes
- `amr_sweeper_mission_executor`: mission APIs, built-in manual mission routing, execution-folder preparation, and mission finalization back to `IDLING`
- `amr_sweeper_scheduler`: iCalendar-based work scheduling and scheduled mission handoff into the mission executor
- `amr_sweeper_vda5050_parser`: background parsing and artifact generation for VDA5050 mission files under `/missions`

## Notes
- The default command launches the layer 0 FSM entrypoint; the remaining layer 0 packages are then started by FSM profiles as needed.
- Profile `001` is the full startup validation path, profile `101` is the normal `IDLING` stack, profile `201` and related `2xx` profiles are mission-time `RUNNING` stacks, and profile `400` is the reduced `FAULT` stack.
- Layer 0 is responsible for coordinating the lower layers, not for driving hardware directly.
- `FAULT` now uses a reduced layer 1 bringup and should not start the layer 2 controller bringup, layer 3 navigation bringup, or `ros2_control` controller stack.
- Runtime mission history, active aliases, and generated artifacts are maintained under `/missions`.
