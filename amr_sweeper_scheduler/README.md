# amr_sweeper_scheduler

ROS 2 scheduler for AMR Sweeper. The package reads an iCalendar schedule (`.ics`, RFC 5545), resolves VDA5050
missions from `/missions`, and coordinates the IDLING-to-RUNNING handoff with the FSM.

This repository contains a single ROS 2 package (no workspace in-tree). Clone it into your existing `src/` and build with
your normal colcon flow.

## Integration with AMR Sweeper FSM

The scheduler is intended to be **launched by FSM state nodes**, primarily in `IDLING`.
To support FSM supervision, the node provides:

- **Tunable parameters** (launch arguments) controlling reload intervals, strictness, and publishing.
- **ROS logs ("rosout triggers")** with a configurable prefix for machine parsing by the FSM.
- Optional **trigger topic** publishing string events for FSM monitoring.
- A compatibility `prepare_mission_execution` service that forwards manual mission preparation into the mission executor.

See `config/architecture.md` and `config/schedule_semantics.md`.

## Missions Folder Workflow

- Default schedule discovery: newest `src/missions_from_db/schedule_<timestamp>.ics`
- Default mission search directory: `src/missions_from_db`
- DB-synced schedule and VDA5050 mission payloads live under `/missions_from_db`
- Execution history and runtime aliases live under `/missions_log`
- Each synced mission is staged under its own folder, for example `src/missions_from_db/polygon_test_20260523T000000Z/polygon_test_20260523T000000Z.json`
- Each execution creates a timestamped subfolder under `src/missions_log/<mission_id>/`, for example `src/missions_log/polygon_test_20260523T000000Z/20260524T211500Z/`
- Work windows publish both `mission_id` and resolved `mission_path` when a matching VDA5050 JSON file is found.
- If exactly one mission JSON exists in `/missions`, the scheduler will use it as a fallback during initial testing.
- The recommended convention is for `X-MISSION-ID` to match the mission folder and mission filename stem, for example `polygon_test_20260523T000000Z` for `src/missions_from_db/polygon_test_20260523T000000Z/polygon_test_20260523T000000Z.json`.
- When a WORK window becomes active, the scheduler:
  - checks whether `/missions_from_db/<order_id>_<timestamp>.json` or `/missions_from_db/<order_id>_<timestamp>/` exists for that mission
  - asks `amr_sweeper_mission_executor/execute_mission` to execute the selected mission
  - the mission executor builds VDA5050 artifacts when needed, prepares the execution folder, refreshes active aliases, and requests the FSM transition to the correct RUNNING profile

For manual operation, prefer the mission executor APIs directly:
- `list_manual_missions`
- `prepare_manual_mission`
- `execute_mission`

Built-in manual missions such as `3x3Sweep`, `SpotSweep`, `RecordMap`, and `Teleop` are now sourced from
`amr_sweeper_default_missions` instead of living directly under `/missions`.

The scheduler still exposes `prepare_mission_execution` as a compatibility API, but it now delegates
the real preparation work to `amr_sweeper_mission_executor/prepare_manual_mission`.

Example:

```bash
ros2 service call /prepare_mission_execution \
  amr_sweeper_scheduler/srv/PrepareMissionExecution \
  "{mission_id: 'polygon_test_20260523T000000Z'}"
```

### Required VEVENT properties

Each `VEVENT` should include:
- `DTSTART` with timezone (e.g., `DTSTART;TZID=Europe/Copenhagen:20260310T100000`)
- `DURATION` (preferred) or `DTEND`
- Optional `RRULE` for recurrence
- `UID` for stable identity

### Project-specific X-properties

- `X-ROBOT-ID`: e.g. `RBT-01`
- `X-SCHEDULE-TYPE`: `WORK`, `NO_WORK`, or `SAFETY`
- `X-MISSION-ID`: e.g. `polygon_test_20260523T000000Z` (for `WORK` only)

Runtime log notes:
- mission execution events may be enriched with `X-ACTUAL-START-UTC`, `X-ACTUAL-END-UTC`, `X-ACTUAL-DURATION-SECONDS`, and `X-RUNTIME-STATUS`
- safety-stop events are appended as dedicated `SAFETY` VEVENTs so the same schedule file acts as both future plan and runtime log

## Build (standard colcon)

From your ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 run amr_sweeper_scheduler scheduler_node --ros-args \
  -p missions_directory:=src/missions_from_db \
  -p robot_id:=RBT-01 \
  -p schedule_poll_interval_sec:=60 \
  -p strict_validation:=true
```

## Parameters

The maintained parameter defaults live in `config/scheduler.yaml`.

YAML style guide for package config:
- Match the formatting pattern used in `amr_sweeper_gnss/config/amr_sweeper_gnss_ntrip_client.yaml`.
- Use a single section headline comment in the form `# -- Section Name -------------------------------------`.
- Keep all parameter keys aligned to the same colon column within the file.
- Keep all inline comments aligned to the same column within the file.
- Add an inline comment for every parameter, including the meaning and the default value.

Trigger log format example:
- `FSM_TRIGGER SCHED_ICS_NOT_FOUND path=/...`
- `FSM_TRIGGER SCHED_ICS_LOAD_FAILED reason=...`
- `FSM_TRIGGER SCHED_ICS_LOADED events=4`

## License

Apache-2.0 (see `LICENSE`).
