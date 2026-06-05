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
- `builtin_manual_mapping` or `execution_mode: "manual_mapping"` -> RUNNING profile `202`
- `builtin_local_pattern` -> RUNNING profile `203`
- `builtin_teleop` or `execution_mode: "teleoperation"` -> RUNNING profile `204`

The node can accept scheduler-prepared execution folders, or prepare built-in manual missions on
its own before requesting the FSM transition to `RUNNING`.

Responsibilities:
- accept scheduled activations from the scheduler
- accept external/manual activation calls for built-in missions
- source built-in manual mission templates from `amr_sweeper_default_missions`
- support one rerecordable built-in working-area capture mission: `RecordMap`
- build VDA5050 mission artifacts on demand when scheduled missions are not ready yet
- read synced schedules and VDA5050 mission payloads from `/missions_from_db`
- prepare mission-specific execution folders before the FSM handoff
- write execution history under `/missions_log` using per-run `execution_context.json` files inside each mission folder
- publish the most recent completed `RecordMap` output into `/missions_log/latest_recorded_map/` so operators can reuse or overwrite the latest working-area recording
- append manual execution entries into the schedule log and stamp scheduled entries with actual start time
- finalize mission runs with actual end time, outcome, and an FSM return to `IDLING`
- calculate the traveled-path length from `actual_path.geojson` and append it into mission end metadata
- subscribe to `safety_msgs/stop` and append dedicated `SAFETY` VEVENT log entries into the schedule
- watch `drive_controller/odom` during teleop missions and `odometry/fused` during manual mapping missions, and automatically end either mission type after 5 minutes without motion
- record teleop traveled path into the active run folder's `actual_path.geojson`
- record `RecordMap` GNSS points into `actual_path_navsat.geojson` so the latest perimeter can be previewed on a satellite map
