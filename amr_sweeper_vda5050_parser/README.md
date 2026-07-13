# amr_sweeper_vda5050_parser

Builds mission artifacts from VDA5050 mission files in `/missions/database`.

This package is intended to run asynchronously in `IDLING`, but the default behavior
is lazy: source missions are discovered without staging artifacts, and artifacts are
built when a mission is selected or when the mission executor requests a rebuild.

## What It Does

- Watches `/missions/database` for VDA5050 mission `.json` files
- Stages the selected mission into its own subfolder under the configured `missions_log_directory`
- Builds selected per-mission artifacts:
  - `<missions_log_directory>/<order_id>/<order_id>_vda5050.json`
  - `<missions_log_directory>/<order_id>/<order_id>_static_costmap.pgm`
  - `<missions_log_directory>/<order_id>/<order_id>_static_costmap.yaml`
  - `<missions_log_directory>/<order_id>/<order_id>_path_planned.geojson`
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
- `auto_build_on_start`: build the active mission immediately on startup
- `watch_for_updates`: keep checking the selected mission for updates
- `build_discovered_missions`: opt in to eagerly building every discovered mission; default is `false`
