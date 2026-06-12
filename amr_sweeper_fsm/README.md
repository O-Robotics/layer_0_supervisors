# amr_sweeper_fsm

ROS 2 package that implements a **robot-level finite state machine (FSM)** using:

```bash
ros2 launch amr_sweeper_fsm amr_sweeper_fsm.launch.py
```

- a non-lifecycle **supervisor** node (`supervisor_node`), and
- one **LifecycleNode** per FSM state:
  - `initializing_state_node`
  - `idling_state_node`
  - `running_state_node`
  - `charging_state_node`
  - `fault_state_node`

The supervisor accepts state-change requests (with priority metadata), drives ROS 2 lifecycle transitions for the state nodes, and publishes FSM status topics based on configurable publish rules.

---

## Repository layout (as shipped)

```
amr_sweeper_fsm/
├── launch/
│   └── amr_sweeper_fsm.launch.py
├── config/
│   ├── state_parameters.yaml
│   └── profiles/
│       ├── initializing_profiles.yaml
│       ├── idling_profiles.yaml
│       ├── running_profiles.yaml
│       ├── charging_profiles.yaml
│       └── fault_profiles.yaml
├── msg/
│   ├── FSMState.msg
│   └── FSMStatus.msg
├── srv/
│   └── RequestState.srv
├── src/
│   ├── _supervisor/
│   │   ├── supervisor_node.cpp
│   │   ├── state_node_base.cpp
│   │   └── process_manager.cpp
│   └── <state implementations>
```

---

## Launch

The primary launch file starts the supervisor and all lifecycle state nodes:

```bash
ros2 launch amr_sweeper_fsm amr_sweeper_fsm.launch.py
```

### Launch arguments

`launch/amr_sweeper_fsm.launch.py` defines the following launch arguments:

- `namespace` (default: `amr_sweeper`)  
  Top-level namespace for all nodes.

- `use_sim_time` (default: `false`)  
  Passed to all nodes. When `true`, the FSM follows ROS time from `/clock`.

- `use_profile` (default: `001`)  
  Passed to the supervisor as `desired_profile` (integer). This selects the startup profile id.

- `tick_period_ms` (default: `100`)  
  Supervisor tick period in milliseconds.

- `state_params_file` (default: `<package_share>/config/state_parameters.yaml`)  
  ROS parameters file for supervisor + state nodes. The default is resolved with
  `ament_index_python.get_package_share_directory("amr_sweeper_fsm")`.

Example (5 second tick, start profile 201, custom namespace):

```bash
ros2 launch amr_sweeper_fsm amr_sweeper_fsm.launch.py \
  namespace:=robot1 \
  use_profile:=201 \
  tick_period_ms:=5000
```

---

## Configuration model

### 1) `config/state_parameters.yaml`

This single ROS parameters file configures:

- **Supervisor publish rules** under `/**/supervisor.ros__parameters.publish.rules`.

  In the provided default config, the supervisor publishes:
  - `fsm_state` (`amr_sweeper_fsm/msg/FSMState`)
  - `fsm_status` (`amr_sweeper_fsm/msg/FSMStatus`)

  (These are *relative* names; with the default namespace they become:
  `/amr_sweeper/fsm_state` and `/amr_sweeper/fsm_status`.)

- **Per-state fault handling**, under each `/**/<state>_state.ros__parameters.faults`.

- **Per-state profile file path**, under each `/**/<state>_state.ros__parameters.profiles.file`.

  The per-state profile file paths in `state_parameters.yaml` are provided as **paths relative to the package share directory**
  (e.g. `config/profiles/running_profiles.yaml`). The launch file passes only `state_parameters.yaml`; the state nodes load
  their per-state profile files based on these references.

### 2) `config/profiles/*_profiles.yaml`

Each state has its own profile file with a list of profiles:

- `profiles[].profile.id` (uint16)
- optional `transitions` fields (e.g., auto transition / fault transition targets)
- `processes` list describing what to start/monitor in that profile, including:
  - `startup.exec` and `startup.args`
  - readiness checks (`startup.ready[]` supporting at least `topic` and `service` targets)
  - restart and shutdown policy
  - optional `rosout_triggers`

The shipped profile catalog is currently:

- `000`: INITIALIZING bridge profile with no processes; auto-requests `100`
- `001`: default INITIALIZING full-stack startup validation
- `002`: alternate INITIALIZING validation profile
- `003`: INITIALIZING debug bringup profile that starts the full stack and stays idle without auto-jumping to `101` or `400`
- `100`: IDLING bridge profile with no processes; auto-requests `200`
- `101`: default IDLING profile with layer 1 hardware bringup, `amr_sweeper_vda5050_parser`, and `amr_sweeper_scheduler`
- `110`: IDLING test profile for `fsm_tester_node`
- `200`: RUNNING bridge profile with no processes; auto-requests `300`
- `201`: default RUNNING mission execution profile
- `210`: RUNNING built-in local-missions profile
- `211`: RUNNING auto-start `3x3Sweep` profile
- `212`: RUNNING auto-start `SpotSweep` profile
- `220`: RUNNING manual teleoperation profile
- `225`: RUNNING manual RecordMap profile with localization/SLAM
- `300`: CHARGING bridge profile with no processes; auto-requests `400`
- `301`: default CHARGING profile
- `400`: default FAULT profile with no processes and no further transition
- `401`: FAULT empty/manual fallback profile

`000`, `100`, `200`, and `300` are empty bridge profiles that auto-request `100`, `200`, `300`, and `400` respectively.
`003` starts the same full-stack bringup path as `001`, but it keeps those processes non-blocking for debugging and remains in `INITIALIZING`.


---

## Interfaces

### Topics (configured via publish rules)

Message definitions live in `msg/`:

- `amr_sweeper_fsm/msg/FSMState`
  - `stamp`
  - `current_state` (string like `"RUNNING"`)
  - `current_profile` (uint16)

- `amr_sweeper_fsm/msg/FSMStatus`
  - `stamp`
  - `current_state`
  - `current_lifecycle_state`
  - `current_profile`
  - `transitioning_to_profile`
  - `transition_status`
  - `last_requester`, `last_request_priority`, `effective_priority_gate`, `priority_age_sec`
  - `last_message`

The default config publishes them on `fsm_state` and `fsm_status` once per second.

### Service: request a state/profile

Service definition: `srv/RequestState.srv`

The supervisor exposes a service named **`request_state`** (relative name), i.e. by default:

- `/amr_sweeper/request_state` (with `namespace:=amr_sweeper`)

Request fields include:
- `target_state`: one of `"INITIALIZING"`, `"IDLING"`, `"RUNNING"`, `"CHARGING"`, `"FAULT"`
- `target_lifecycle`: `""` / `"Active"` to activate, or `"Inactive"` to configure only
- `target_profile_id`: uint16 profile id
- metadata: `requester`, `priority`, `force`, `reason`
- `mission_execution_directory`: exact scheduler-prepared RUNNING execution folder, or empty for non-mission transitions

Example:

```bash
ros2 service call /amr_sweeper/request_state amr_sweeper_fsm/srv/RequestState "{target_state: 'RUNNING', target_lifecycle: 'Active', target_profile_id: 201, requester: 'cli', priority: 200, force: false, reason: 'manual switch', mission_execution_directory: '/abs/path/to/missions/order_20260523T120000Z/20260523T121500Z'}"
```

---

## Profile id “bands”

`RequestState.srv` documents the intended convention that each FSM state has a “*00” bridge/base profile id:
- `000` for INITIALIZING
- `100` for IDLING
- `200` for RUNNING
- `300` for CHARGING
- `400` for FAULT

In the current configuration, the configured default profiles are `001`, `101`, `201`, `301`, and `400`. The `*00` profiles are empty bridge/base profiles chained as `000 -> 100 -> 200 -> 300 -> 400`.

---

## Quick introspection

With default namespace (`amr_sweeper`):

```bash
# Supervisor status streams (per config/state_parameters.yaml)
ros2 topic echo /amr_sweeper/fsm_state
ros2 topic echo /amr_sweeper/fsm_status

# See current desired profile parameter (supervisor)
ros2 param get /amr_sweeper/supervisor desired_profile

# Lifecycle state of a specific FSM state node
ros2 lifecycle get /amr_sweeper/initializing_state
```

---

## Build

Typical colcon build:

```bash
colcon build --packages-select amr_sweeper_fsm
source install/setup.bash
```

---

## Notes

- The supervisor tick period is configurable via `tick_period_ms` in the launch file (default: 100 ms).
- Publish periods are configured via `publish.rules` in `config/state_parameters.yaml` and are decoupled from the supervisor tick.
- RUNNING profiles `201`, `210`, and `225` pass `runtime.mission_execution_directory` into layer 3 bringup via the `{mission_execution_directory}` placeholder when a request provides it. Profiles `211` and `212` can also self-start their built-in missions directly via explicit mission launch arguments.
