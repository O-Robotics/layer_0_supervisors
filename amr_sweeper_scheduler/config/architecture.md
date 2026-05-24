# Architecture

## Goals

- Run as a lightweight ROS 2 node launched by the FSM, primarily in `IDLING`.
- Keep CPU and memory use low by only reloading the schedule when the file changes.
- Provide clean monitoring signals for the FSM through rosout triggers and an optional trigger topic.

## Components

- `SchedulerNode`
  - polls the `.ics` file at `schedule_poll_interval_sec`
  - reloads on mtime change by default
  - parses VEVENTs into a `ScheduleModel`
  - expands time windows within `horizon_hours`
  - applies `NO_WORK` blackout filtering
  - resolves `X-MISSION-ID` values to VDA5050 mission files in `/missions`
  - asks `amr_sweeper_mission_executor/execute_mission` to handle scheduled mission activation
  - exposes a compatibility `prepare_mission_execution` API that forwards manual preparation into `amr_sweeper_mission_executor/prepare_manual_mission`
  - publishes planned windows as JSON for downstream mission execution
  - emits FSM triggers on load and validation events

- `MissionExecutorNode`
  - accepts scheduled execution requests from the scheduler
  - accepts external/manual mission activation requests
  - classifies missions into scheduled, manual mapping, manual routed, and teleoperation flows
  - checks whether VDA5050 mission artifacts are ready
  - calls `amr_sweeper_vda5050_parser/build_current_mission` when a scheduled mission must be rebuilt
  - prepares execution folders and active aliases
  - requests the FSM transition to the correct `RUNNING` profile

- `IcalParserMinimal`
  - parses the strict RFC 5545 subset used by the robot schedule
  - handles `UID`, `DTSTART`, `DURATION` or `DTEND`, `RRULE`, `X-ROBOT-ID`,
    `X-SCHEDULE-TYPE`, and `X-MISSION-ID`

- blackout filtering
  - removes WORK windows that overlap active NO_WORK windows

## ROS interfaces

- topic `planned_windows` (`std_msgs/String`): JSON payload of expanded schedule windows
- topic `<trigger_topic_name>` (`std_msgs/String`): optional FSM monitoring events
- service `reload_schedule` (`std_srvs/Trigger`): immediate schedule reload
- service `prepare_mission_execution` (`amr_sweeper_scheduler/PrepareMissionExecution`): compatibility manual preparation API forwarded to the mission executor
