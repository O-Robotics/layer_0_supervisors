# amr_sweeper_bringup

Launches the AMR Sweeper layer 0 supervisor stack from a single entrypoint.

Packages started by this bringup:
- `amr_sweeper_simulation` when `use_simulation:=true`
- `amr_sweeper_fsm`
- `amr_sweeper_mission_executor`
- `amr_sweeper_scheduler`
- `amr_sweeper_mission_builder`
- `amr_sweeper_interface_server`

Main launch:
- `ros2 launch amr_sweeper_bringup amr_sweeper_bringup.launch.py`

Rosbag recording:
- `amr_sweeper_bringup` owns the system-level rosbag recorder launch logic.
- its default topic allowlist lives in `config/record_system_rosbag.yaml` and focuses on FSM, supervisor, hardware health, and whole-system status topics.
- launch it with `record_system_rosbag:=true` to capture a bringup-level system bag.
- pass `record_mission_rosbag:=true` to make mission execution record bags by default.

Simulation ownership:
- `amr_sweeper_bringup` includes `amr_sweeper_simulation` when `use_simulation:=true`.
- Gazebo worlds, bridge mappings, and simulation helper scripts now live in `../amr_sweeper_simulation`.

Test schedule mode:
- `use_test:=false` by default keeps `missions/database` free of checked-in test fixtures.
- `use_test:=true` makes bringup pass `test_schedule_ics_path` to the mission executor and scheduler when `schedule_ics_path` is otherwise empty.
- The same `use_test:=true` path also makes FSM-managed layer 3 mapping fall back to generic test artifacts under `/tmp/amr_sweeper_test_artifacts` when no mission execution folder is active.
