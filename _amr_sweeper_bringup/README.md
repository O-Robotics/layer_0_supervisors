# amr_sweeper_bringup

Launches the AMR Sweeper layer 0 supervisor stack from a single entrypoint.

Packages started by this bringup:
- `amr_sweeper_fsm`
- `amr_sweeper_mission_executor`
- `amr_sweeper_scheduler`
- `amr_sweeper_vda5050_parser`
- `amr_sweeper_web_server`

Main launch:
- `ros2 launch amr_sweeper_bringup amr_sweeper_bringup.launch.py`
