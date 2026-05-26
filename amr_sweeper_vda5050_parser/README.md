# amr_sweeper_vda5050_parser

Builds mission artifacts from VDA5050 mission files in `/missions_from_db`.

This package is intended to run asynchronously in `IDLING` so the robot can keep its
mission artifacts up to date before the scheduler asks the FSM to enter `RUNNING`.

## What It Does

- Watches `/missions_from_db` for VDA5050 mission `.json` files
- Stages each valid mission into its own subfolder at `/missions_from_db/<order_id>_<timestamp>/`
- Builds per-mission artifacts:
  - `/missions_from_db/<order_id>_<timestamp>/<order_id>_<timestamp>.json`
  - `/missions_from_db/<order_id>_<timestamp>/<order_id>_<timestamp>_costmap.pgm`
  - `/missions_from_db/<order_id>_<timestamp>/<order_id>_<timestamp>_costmap.yaml`
  - `/missions_from_db/<order_id>_<timestamp>/<order_id>_<timestamp>_path.geojson`
- Leaves runtime selection to the exact per-mission artifacts referenced from `execution_context.json`
- Exposes `build_current_mission` so the mission executor can force-parse and build the selected mission

## Launch

```bash
ros2 launch amr_sweeper_vda5050_parser amr_sweeper_vda5050_parser.launch.py
```

The maintained parameter defaults live in `config/amr_sweeper_vda5050_parser.yaml`.

YAML style guide for package config:
- Match the formatting pattern used in `amr_sweeper_gnss/config/amr_sweeper_gnss_ntrip_client.yaml`.
- Use a single section headline comment in the form `# -- Section Name -------------------------------------`.
- Keep all parameter keys aligned to the same colon column within the file.
- Keep all inline comments aligned to the same column within the file.
- Add an inline comment for every parameter, including the meaning and the default value.

## Default Runtime Role

- `INITIALIZING`: used as part of the full-stack startup check
- `IDLING`: runs continuously to detect new missions and refresh artifacts
- `RUNNING`: not required, because artifacts should already be prepared

## Key Parameters

- `mission_path`: optional active mission file used for an on-demand rebuild; leave empty for auto-discovery
- `missions_directory`: folder containing incoming VDA5050 mission files and per-mission subfolders
- `mission_file_extension`: mission file suffix to scan for
- `costmap_output_basename`: legacy compatibility parameter, no longer used for runtime aliases
- `coverage_path_basename`: legacy compatibility parameter, no longer used for runtime aliases
- `auto_build_on_start`: build the active mission immediately on startup
- `watch_for_updates`: keep scanning for new or changed mission files
