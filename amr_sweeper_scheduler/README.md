# amr_sweeper_scheduler

```bash
ros2 launch amr_sweeper_scheduler amr_sweeper_scheduler.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_fsm`
- `amr_sweeper_mission_executor`
- `amr_sweeper_vda5050_parser`
- `amr_sweeper_navigation`

## Purpose

Reads an iCalendar schedule (`.ics`, RFC 5545), resolves scheduled mission entries to VDA5050 mission files, and coordinates the `IDLING` to `RUNNING` handoff with the FSM.

## Main Launch File

`launch/amr_sweeper_scheduler.launch.py`

## Available Launch Files

- `amr_sweeper_scheduler.launch.py`

## Launch Arguments

- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`
- `schedule_ics_path`: default `""`
- `missions_directory`: default `missions/database`
- `default_schedule_filename`: default `""`
- `mission_file_extension`: default `.json`
- `robot_id`: default `""`
- `mission_executor_execute_service`: default `execute_mission`
- `mission_executor_prepare_service`: default `prepare_manual_mission`
- `trigger_running_on_work_window`: default `true`
- `retry_attempts_before_error`: default `3`
- `fatal_after_consecutive_errors`: default `10`
- `config_file`: default `config/amr_sweeper_scheduler.yaml`
- `robot_config_env_path` in `config/amr_sweeper_scheduler.yaml`: default `/opt/robot_config/robot_config.global.env`

## Overview

The scheduler is intended to run as a lightweight ROS 2 node launched by FSM state nodes, primarily in `IDLING`.

- `SchedulerNode` polls the schedule file at `schedule_poll_interval_sec` and reloads on file mtime change by default.
- `IcalParserMinimal` parses the RFC 5545 subset used by the robot schedule, including `UID`, `DTSTART`, `DURATION` or `DTEND`, `RRULE`, `X-ROBOT-ID`, `X-SCHEDULE-TYPE`, and `X-MISSION-ID`.
- Each `VEVENT` maps to a time window from `DTSTART` plus `DURATION` or `DTEND`.
- `RRULE` entries are expanded within `horizon_hours`.
- `NO_WORK` windows override `WORK` windows.
- Overlapping `WORK` windows are handled deterministically by expanded start order.
- Missed windows are not backfilled by default.
- A `WORK` window is actionable only when its mission file resolves in `missions_directory`.
- If mission artifacts are missing or stale, the scheduler asks `amr_sweeper_vda5050_parser` to rebuild the mission before requesting `RUNNING`.
- Scheduled execution requests are forwarded to `amr_sweeper_mission_executor/execute_mission`.
- Manual preparation remains available through `prepare_mission_execution`, which forwards to `amr_sweeper_mission_executor/prepare_manual_mission`.
- Planned windows are published on `scheduler/planned_windows` as JSON in `std_msgs/String`.
- Scheduler status messages are emitted through rosout and, optionally, on `<trigger_topic_name>` as `std_msgs/String`.
- The scheduler exposes `reload_schedule` as a `std_srvs/Trigger` service.
- The node retries schedule setup automatically on each poll cycle and escalates repeated failures from warning to error to fatal.
- The node emits a one-time `Scheduler is now running` message after the first healthy schedule load.

## Notes

- The scheduler auto-discovers the newest `missions/database/schedule_<timestamp>.ics` when `schedule_ics_path` is empty.
- When launched with the default namespace, relative topics and services resolve under `/amr_sweeper/`.
- `robot_id` should usually be left empty so the node derives `AMR-Sweeper_000xx` from `ROBOT_NUMBER` in `/opt/robot_config/robot_config.global.env`.
- `X-ROBOT-ID` values in the schedule should match the derived robot ID format, for example `AMR-Sweeper_00012`.
- Required schedule fields are `UID`, `DTSTART` with `TZID`, and `DURATION` or `DTEND`.
- Project schedule fields are `X-ROBOT-ID`, `X-SCHEDULE-TYPE`, and `X-MISSION-ID` for `WORK` entries.
- Supported schedule types are `WORK`, `NO_WORK`, and `SAFETY`.
- All `DTSTART` values should include a `TZID` matching the site configuration.
- The scheduler evaluates time in the local site timezone represented by the schedule.
- Runtime log enrichment may add `X-ACTUAL-START-UTC`, `X-ACTUAL-END-UTC`, `X-ACTUAL-DURATION-SECONDS`, and `X-RUNTIME-STATUS`.
- Safety-stop events are appended as dedicated `SAFETY` `VEVENT`s so the same schedule file can act as both future plan and runtime log.
- Mission files are expected under `missions/database`, and execution history is written under `missions/logs`.
- A synced mission is typically staged as `missions/database/<mission_id>/<mission_id>.json`.
- Each execution creates a timestamped folder under `missions/logs/<mission_id>/`.
- If exactly one mission JSON exists, the scheduler can use it as a fallback during initial testing.
- The recommended convention is for `X-MISSION-ID` to match the mission folder and mission filename stem.
- Built-in manual missions such as `3x3Sweep`, `SpotSweep`, `RecordMap`, and `Teleop` come from `amr_sweeper_navigation/missions`.
- Status examples include `SCHED_ICS_NOT_FOUND path=/...`, `SCHED_ICS_LOAD_FAILED reason=...`, and `SCHED_ICS_LOADED events=4`.
- `SCHED_ICS_LOAD_FAILED reason=ICS contains no VEVENTs` is emitted as a warning.
- `SCHED_ICS_LOADED events=<...>; schedule=<...>; robot_id=<...>` is emitted after a successful schedule load.
- `Scheduler is now running` is emitted once after the node comes up cleanly.
- `SCHED_SELF_RECOVERY ...` reports escalating recovery attempts before the node goes fatal.
- The maintained parameter defaults live in `config/amr_sweeper_scheduler.yaml`.
- Package config files should follow the workspace YAML style guide and stay under `config/`.


