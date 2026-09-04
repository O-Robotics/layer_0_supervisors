# amr_sweeper_mission_builder

Builds mission artifacts from VDA5050 mission package folders in `/missions/database`.
It also hosts a separate Gaussian splat builder node for post-processing captured
mission runs.

This package is intended to run asynchronously in `IDLING`, but the default behavior
is lazy: source missions are discovered without staging artifacts, and artifacts are
built when a mission is selected or when the mission executor requests a rebuild.

## What It Does

- Watches `/missions/database/<mission_id>/order.json` VDA5050 package folders
- Requires `map_georeference.json` beside every `order.json`
- Optionally applies `zoneSet.json` beside the order; `BLOCKED` zones become no-go costmap areas
- Stages the selected mission into its own subfolder under the configured `missions_log_directory`
- Builds selected per-mission artifacts:
  - `<missions_log_directory>/<order_id>/<order_id>_vda5050.json`
  - `<missions_log_directory>/<order_id>/map_georeference.json`
  - `<missions_log_directory>/<order_id>/zoneSet.json` when present
  - `<missions_log_directory>/<order_id>/<order_id>_static_costmap.pgm`
  - `<missions_log_directory>/<order_id>/<order_id>_static_costmap.yaml`
  - `<missions_log_directory>/<order_id>/<order_id>_path_planned.geojson`
- Leaves runtime selection to the exact per-mission artifacts referenced from `execution_context.json`
- Exposes `build_current_mission` so the mission executor can force-parse and build the selected mission
- Exposes `build_gaussian_splat` from `gaussian_splat_builder_node` to build tiled splat artifacts
  from layer 3 Gaussian capture manifests
- Trains Gaussian splat tiles with CUDA `gsplat`; the worker fails clearly when PyTorch,
  `gsplat`, or a CUDA device is unavailable

## Launch

```bash
ros2 launch amr_sweeper_mission_builder amr_sweeper_mission_builder.launch.py
```

The maintained parameter defaults live in `config/amr_sweeper_mission_builder.yaml`.

YAML style guide for package config:
- Match the formatting pattern used in `amr_sweeper_gnss/config/amr_sweeper_gnss_ntrip_client.yaml`.
- Use a single section headline comment in the form `# -- Section Name -------------------------------------`.
- Keep all parameter keys aligned to the same colon column within the file.
- Keep all inline comments aligned to the same column within the file.
- Add an inline comment for every parameter, including the meaning and the default value.

## Default Runtime Role

- `INITIALIZING`: used as part of the full-stack startup check
- `IDLING`: runs continuously to detect new missions and refresh artifacts; also starts
  `gaussian_splat_builder_node` in default IDLING profiles
- `CHARGING`: starts `gaussian_splat_builder_node` so captured mission runs can be processed while charging
- `RUNNING`: not required, because artifacts should already be prepared

## Key Parameters

- `mission_path`: optional active mission file used for an on-demand rebuild; leave empty for auto-discovery
- `missions_directory`: folder containing VDA5050 mission package subfolders
- `simulations_directory`: allowed root for Gaussian capture manifests written by simulation runs
- `supported_vda5050_versions`: accepted VDA5050 major-version-3 message versions
- `auto_build_on_start`: build the active mission immediately on startup
- `watch_for_updates`: keep checking the selected mission for updates
- `build_discovered_missions`: opt in to eagerly building every discovered mission; default is `false`
- `launch_gaussian_splat_builder_node`: launch the separate Gaussian splat builder node; default is `false`
- `default_tile_size_meters`: default Gaussian splat tile size when a request omits it
- `max_training_image_dimension`: downscale limit used by the CUDA training worker
- `worker_python_executable`: optional Python interpreter for the CUDA training worker
- `worker_cuda_home`: optional CUDA toolkit root for Python wheel-provided `nvcc`
