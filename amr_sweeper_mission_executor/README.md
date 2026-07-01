# amr_sweeper_mission_executor

Provides mission activation APIs that choose the correct RUNNING profile for scheduled and manual missions.

Services:
- `list_executable_missions`
- `list_manual_missions`
- `upload_vda5050_mission`
- `create_recorded_mission`
- `prepare_manual_mission`
- `execute_mission`
- `end_mission`

HTTP operator UI:
- the HTTP operator UI now lives in `amr_sweeper_interface_server`
- it still uses the mission executor services to list missions, execute them, stop the active mission, and upload VDA5050 mission JSON payloads

Profile routing:
- scheduled VDA5050 missions -> RUNNING profile `201`
- `builtin_manual_mapping` or `execution_mode: "manual_mapping"` -> RUNNING profile `225`
- `builtin_local_pattern` -> RUNNING profile `210`
- `builtin_teleop` or `execution_mode: "teleoperation"` -> RUNNING profile `220`

The node can accept scheduler-prepared execution folders, or prepare built-in manual missions on
its own before requesting the FSM transition to `RUNNING`.

Responsibilities:
- accept scheduled activations from the scheduler
- accept external/manual activation calls for built-in missions
- source built-in manual mission templates from `amr_sweeper_navigation/missions`
- support one rerecordable built-in working-area capture mission: `RecordMap`
- build VDA5050 mission artifacts on demand when scheduled missions are not ready yet
- read synced schedules and VDA5050 mission payloads from `/missions/database`
- prepare mission-specific execution folders before the FSM handoff
- write execution history under `/missions/logs` using `<mission_id>_vda5050.json`, `<mission_id>_path_planned.geojson`, and per-run `<mission_id>_<run_timestamp>_context.json` files inside each mission folder
- honor mission start requests with `record_rosbag=true` by launching `ros2 bag record` from layer 0 and saving the bag under `<mission_run_directory>/artifacts/rosbag`
- publish the most recent completed `RecordMap` output into `/missions/logs/latest_recorded_map/` so operators can reuse or overwrite the latest working-area recording
- append manual execution entries into the schedule log and stamp scheduled entries with actual start time
- finalize mission runs with actual end time, outcome, and an FSM return to `IDLING`
- calculate the traveled-path length from `<mission_id>_<run_timestamp>_path_actual.geojson` and append it into mission end metadata
- subscribe to `safety_msgs/stop` and append dedicated `SAFETY` VEVENT log entries into the schedule
- watch `drive_controller/odom` during teleop missions and `localization/odometry_fused` during manual mapping missions, and automatically end either mission type after 5 minutes without motion
- record teleop traveled path into the active run folder's `<mission_id>_<run_timestamp>_path_actual.geojson`
- record `RecordMap` GNSS points into `<mission_id>_<run_timestamp>_path_navsat.geojson` so the latest perimeter can be previewed on a satellite map

Rosbag recording:
- the default mission-recording topic allowlist lives in `config/record_mission_rosbag.yaml`
- topics listed there are recorded with `ros2 bag record --regex ...`, so missing topics do not crash mission launch
- the list is focused on runtime mission performance: commands, odometry, localization, planning, mapping, and other mission-execution signals
- `record_mission_rosbag:=true` enables mission rosbag recording by default for executions launched through this stack.
