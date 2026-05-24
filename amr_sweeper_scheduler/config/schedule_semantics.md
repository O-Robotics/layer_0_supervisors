# Schedule semantics

## Definitions

- WORK window: time interval during which the robot may execute a mission
- NO_WORK window: blackout interval during which the robot must not execute a mission

## Interpretation

- Each `VEVENT` maps to a time window from `DTSTART` plus `DURATION` or `DTEND`.
- If a `VEVENT` has an `RRULE`, the scheduler expands occurrences within the configured horizon.
- Within a WORK window the robot should run as much as practical, subject to charging and safety.
- Before the FSM enters `RUNNING`, the scheduler verifies that the selected mission artifacts exist and are current.

## Overlaps and precedence

- NO_WORK overrides WORK.
- If multiple WORK windows overlap, the current behavior is deterministic by expanded start order.

## Missed windows

- Missed windows are not backfilled by default.
- Actuals and telemetry can report them later without mutating the planned schedule.

## Mission readiness

- A WORK window is actionable only when its mission file resolves in `/missions`.
- If the mission exists but its generated artifacts are missing or stale, the scheduler asks
  `amr_sweeper_vda5050_parser` to rebuild the mission before requesting `RUNNING`.

## Time zone

- All `DTSTART` values must include a `TZID` matching the site configuration.
- The scheduler evaluates time in the local site timezone represented by the schedule.
