# amr_sweeper_bringup

Launches the AMR Sweeper layer 0 supervisor stack from a single entrypoint.

Packages started by this bringup:
- `amr_sweeper_fsm`
- `amr_sweeper_mission_executor`
- `amr_sweeper_scheduler`
- `amr_sweeper_vda5050_parser`
- `amr_sweeper_interface_server`

Main launch:
- `ros2 launch amr_sweeper_bringup amr_sweeper_bringup.launch.py`

Test schedule mode:
- `use_test:=false` by default keeps `src/missions_from_db` free of checked-in test fixtures.
- `use_test:=true` makes bringup pass `src/layer_0_supervisors/tests/schedule_20260000T000000Z.ics` to the mission executor and scheduler when `schedule_ics_path` is otherwise empty.
- The same `use_test:=true` path also makes FSM-managed layer 3 mapping fall back to generic test artifacts under `src/layer_3_navigation/tests` when no mission execution folder is active.
