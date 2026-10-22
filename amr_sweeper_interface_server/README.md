# amr_sweeper_interface_server

Provides the AMR Sweeper operator interface stack as a standalone layer 0 package.

Structure:
- `scripts/backend_node.py` is the local-only backend entry point. It owns ROS topics/services, mission files, schedule files, and the raw JSONL API over a Unix domain socket.
- `scripts/frontend_http_node.py` is the network-facing web frontend. It serves the local operator pages and adapts `/api/v1` HTTP requests to backend JSONL requests.
- `scripts/mqtt_bridge_node.py` is a TODO sibling process for a future VDA5050 MQTT bridge. It must route through backend IPC, like the frontend, and must not write mission files or call ROS services directly.

Responsibilities:
- expose a local dashboard over TCP through the frontend node
- keep the backend API reachable only through local IPC
- show FSM state, GNSS latitude/longitude, battery state, and the active mission run folder
- execute built-in manual missions and saved autonomous missions through `amr_sweeper_mission_executor`
- stop the active mission through `amr_sweeper_mission_executor`
- upload/import a VDA5050 mission package into `/missions/database/<mission_id>` so it becomes executable
- provide a dedicated `/record-map` workflow page for starting `RecordMap`, previewing the latest GNSS overlay on satellite imagery, and saving named autonomous missions from the latest recorded map
- let operators select a mission on the Missions page, preview its geometry before start, and choose per-mission launch preferences such as `Record rosbag`

API:
- `GET /api/v1/status`
- `GET /api/v1/missions`
- `GET /api/v1/missions/{mission_id}/download`
- `POST /api/v1/missions/{mission_id}/execute`
- `POST /api/v1/missions/upload-vda5050`
- `POST /api/v1/mission/stop`
- `POST /api/v1/system/reinitialize`
- `POST /api/v1/safety/stop`
- `POST /api/v1/safety/clear`
- `GET /api/v1/schedule`
- `POST /api/v1/schedule/entry`
- `POST /api/v1/schedule/entry/delete`
- `GET /api/v1/map-data`
- `GET /api/v1/record-map`
- `POST /api/v1/record-map/start`
- `POST /api/v1/record-map/stop`
- `POST /api/v1/record-map/save-mission`

All frontend interfaces should use the backend raw JSONL API over the Unix domain socket and should not call ROS2 services/topics directly.

Backend IPC:
- Default socket: `/tmp/amr_sweeper_interface_backend.sock`
- Launch override: `backend_socket_path:=/path/to/socket`
- Cloud listener override: `INTERFACE_BACKEND_SOCKET_PATH=/path/to/socket`
- Request framing: one compact JSON object plus `\n`, for example `{"action":"GET_STATUS","payload":{}}`
- Response framing: one compact JSON object plus `\n`, including `success`, `message`, and `status_code`
- The socket is created with owner-only permissions and removed on backend shutdown.

Default operator URL:
- `http://192.168.2.1:8080`

Launch:
- `ros2 launch amr_sweeper_interface_server amr_sweeper_interface_server.launch.py`

Network setup:
- Target robot Ethernet address: `192.168.2.1`
- Make sure that port 8080/tcp is opened on the Jetson for access from Wi-Fi clients connected to robots Wi-Fi access points.  
  Run `ufw status` as root.
- If port is not opened, open it by executing following command as root:  
  `ufw allow from 192.168.2.0/28 to 192.168.2.1 port 8080 proto tcp`.
- No changes should be done on the RUTX11 router.

Notes:
- only `frontend_http_node.py` binds to `0.0.0.0:8080` by default so operators on the router LAN can reach it
- `backend_node.py` must not bind TCP; it listens only on the Unix domain socket
- the default public URL assumes the Jetson uses the Ethernet address `192.168.2.1`
- plain HTTP should only be exposed by frontend nodes on the trusted robot network; use HTTPS/auth at a gateway, reverse proxy, VPN, or tunnel before exposing the API outside that boundary
- operators connect through the router's Wi-Fi network; the Jetson itself is expected to be wired to the router over Ethernet
- this assumes the robot uplink and router LAN use the `192.168.2.0/24` subnet
- the Missions page stores the selected mission and its launch preferences in browser local storage, so toggles such as `Record rosbag` persist per mission on that client
- when `Record rosbag` is enabled for a started mission, the backend passes that flag into `amr_sweeper_mission_executor`, which records the configured rosbag topics into `<mission_run_directory>/artifacts/rosbag`

## Xbox controller in the browser

The Teleop page supports an Xbox 360 controller connected to the **computer running
the browser**. Linux may show it as `Microsoft Corp. X-Box 360 controller` at
`/dev/input/js0`. The page reads the browser Gamepad API, not that device file;
no Python joystick library, ROS joy node, or device access on the robot is needed.
All commands continue through `/api/v1/teleop/command` and the existing backend IPC.
The runtime change is entirely in `scripts/frontend_http_node.py`.

### Deploying the local update

This local version was prepared from upstream `main` at
`516b087465ebc20c358fa0c8530c6c2f4b79f983` (2026-09-21). Use the matching current
backend, which supports the one-stick/two-stick command fields. Copy the updated
`scripts/frontend_http_node.py` into the package in the robot's ROS workspace,
then rebuild from that workspace root:

```sh
colcon build --packages-select amr_sweeper_interface_server
source install/setup.bash
```

Restart the frontend using the deployment's existing launch/service procedure
and reload the Teleop browser page. No backend source changes are required for
the Xbox controller update alone. The Brushes direction option below also
requires the updated backend and a backend restart. No files need to be uploaded
to GitHub.

### Connection and controls

1. Try the existing `http://192.168.2.1:8080/teleop` URL first. The page uses
   Gamepad access when the browser permits it, without requiring HTTPS itself.
   Some browsers restrict this API on ordinary HTTP; if the page reports it as
   unavailable or blocked, see the browser setting below, use HTTPS with a
   trusted certificate, or access the interface through localhost.
   For an existing SSH-enabled robot, an optional localhost tunnel can be opened
   **on the browser computer**:

   ```sh
   ssh -N -L 127.0.0.1:8080:127.0.0.1:8080 <robot-user>@192.168.2.1
   ```

   Replace `<robot-user>` with the robot's SSH login, keep the tunnel running, and
   browse to `http://localhost:8080/teleop`. If local port 8080 is occupied, change
   the first port to 18080 and browse to `http://localhost:18080/teleop`.
2. Plug in the controller, focus the page, and press a controller button once so
   the browser exposes it. Use a browser that reports `mapping: "standard"` for
   the Xbox controller. Unsupported mappings are ignored. The page does not
   display joystick status messages.
3. **Options > Joystick** is **On** by default. Start **Teleop** or **Record Map**.
   Mouse, touchpad and touch controls remain available with Joystick **On**.
   Turn Joystick **Off** to disable only the controller. If the browser does not
   provide the Gamepad API, mouse/touch control remains available.
4. Center both sticks and release **LB**, then hold **LB** (left bumper) while
   moving a stick. Release LB to request an immediate zero command.

For a dedicated development computer on a trusted robot LAN, Chrome/Chromium
also provides a per-origin testing exception. Open
`chrome://flags/#unsafely-treat-insecure-origin-as-secure`, enter
`http://192.168.2.1:8080` (without `/teleop`), enable the flag, and relaunch the
browser. This keeps the existing HTTP URL and needs no SSH tunnel or server
change. Configure it separately on each operator computer. The exception grants
secure-context features to that origin; it does not encrypt HTTP traffic.
See the [Chromium documentation](https://www.chromium.org/Home/chromium-security/deprecating-powerful-features-on-insecure-origins/#testing-powerful-features).

| Control | Behavior |
| --- | --- |
| Left stick up/down | Forward/reverse |
| Left stick left/right | Turn left/right, using the existing mouse directions |
| Right stick | Tool control when **Two Stick** is On |
| LB | Hold to enable both sticks; release to stop |
| X / B | Increase / decrease Drive Speed by 5 percentage points per press |
| Y / A | Increase / decrease Tool Speed (brushes) by 5 percentage points per press |
| Drive Speed / Tool Speed | Existing sliders and backend scaling remain in effect |
| Mouse/touchpad/touch drag | Takes priority over the controller until the on-screen sticks return to centre |
| Joystick Off | Disable controller input; mouse/touch remains available |

Camera is On by default and starts streaming when Teleop or Record Map is active.
Drive Speed and Tool Speed both start at 75%. The on-screen circles and speed
bars scale with the available pane, at 72.25% of the full pane-fitting size.

On computers, a running Teleop mission shows **LIGHTS**, **CAMERA**,
**2 STICK MODE** and **BRUSHES** shortcuts below the lower Stop button, with the
same button size and font. They share the existing Options actions and enabled
colors; hovering shows the current value. The Options menu remains available.
Compact touch-first screens (shorter screen dimension at most 820 pixels, coarse
primary pointer, and no fine pointer) keep the existing smartphone interface.
Detection also runs when screen size or pointer capabilities change, and a
narrow browser window on a computer still gets the shortcuts. Shortcuts appear
only while Teleop is running, and hide during mission transitions or stopping.

The A/B/X/Y speed buttons work before a mission and while driving; LB is not
required to adjust a speed. Each press changes the corresponding existing scale
once, within 0–100%, updates its bar, and briefly shows the percentage. Holding a
button does not repeat, and pressing increase and decrease together has no effect.
The page must be focused, visible and receiving robot status. Keys held when the
controller connects or the page regains focus must be released before they count
as a new press. Mouse/touch drags on a stick or speed bar take priority over these
keys. The buttons do not start a mission or move the robot by themselves.

The existing one-stick mode remains the default. Enable **Two Stick** to operate
the drive and tools independently. The sticks have a 15% radial dead zone, with
the remaining travel scaled continuously to full range. Commands are sampled at
the existing 50 ms interval, with at most one command request in flight. Movement
commands keep that cadence; repeated idle zero commands are limited to one every
200 ms (five per second). Returning to centre and explicit stop events bypass
the idle limit and use the shared stop queue immediately.

**Options > Brushes** selects the one-stick spinning direction and starts at
**OUT** each time the page loads. Tap or click to select **IN**: forward driving
spins the brushes inward, and reverse driving spins them outward. The setting
applies to mouse, touch and browser gamepad input; two-stick tool control stays
independent. IN reverses the standard one-stick drive-to-tool mapping while
keeping drive and tool speed scales independent. Deploy both
`scripts/frontend_http_node.py` and `scripts/backend_node.py` for this option.
The backend uses the existing explicit tool path for IN; no controller changes
are required with the standard mapping. Custom sweeping-controller mapping
gains or offsets would need matching changes to the IN mapping in the backend.

Both input methods can stay enabled, but only one source controls the robot at
a time. Starting an on-screen drag stops any active controller command and
takes ownership of both sticks. The controller cannot overwrite a drag or its
return-to-centre animation. Once all drags finish, centre both controller sticks
and release LB before using the controller again. Holding LB through a mouse
drag will not automatically resume controller motion when the mouse is released.
An idle controller connecting, disconnecting or becoming unavailable does not
cancel mouse input.

During controller driving, disconnecting, releasing LB, losing focus, hiding or
leaving the page, stale robot status and command errors clear controller input
and request zero motion. Changing the joystick setting or stick mode also clears
both sticks. Starting controller driving again requires centred sticks and
released LB. Controller buttons do not start missions automatically.

Mouse input retains the upstream command mapping, readiness checks and release
animation regardless of the Joystick setting. Controller focus/status checks
and command timeouts apply to controller driving. Normal stops from mouse, touch
and gamepad input share a queue with Joystick On or Off, including browsers where
the Gamepad API is unavailable. An in-flight command is followed by zero before
another source can send movement. Awaiting a stop waits for its queued request
to finish. Stop, Safety Stop and stick-mode changes clear held controls with
either Joystick setting. Backend mission-stop and Safety Stop requests proceed
immediately while zero commands drain through the queue. The unload beacon
remains a best-effort fallback.

Browser stop requests still require a working connection. A browser crash, lost
network, or an aborted request cannot guarantee delivery or server processing
order; the robot's independent command timeout and emergency stop must handle
those cases. This frontend change does not add a robot-side watchdog.

### Local verification

For an input mismatch that cannot be reproduced locally, open the Teleop URL
with `?drive_debug=1`, for example `http://192.168.2.1:8080/teleop?drive_debug=1`.
This enables a Forward/Turn percentage readout and a **Save drive log** button
inside **Options** at the bottom of the screen.
Reproduce the issue, release the controls, and save `drive_input_capture.json`
before closing or reloading the page. The log keeps the last 2,400 command
requests (about two minutes), including raw mouse coordinates or gamepad axes,
normalized inputs, the actual HTTP payloads, responses and errors. It records
only this browser page and does not measure physical robot movement. Saving
does not send or replay motion commands. Recording is off on the normal URL.

The earlier `drive_capture.jsonl` is a ROS snapshot capture. Its half-second
snapshots do not contain browser inputs or identify which publisher sent each
message; missing straight commands in that file alone do not establish whether
the operator attempted straight movement.

The browser test renders the actual Python HTML template without importing ROS,
uses Playwright with simulated gamepad input and a mock HTTP backend, and checks
the outgoing commands and input interruption behavior:

```sh
node --test amr_sweeper_interface_server/test/test_teleop_gamepad.cjs
python3 amr_sweeper_interface_server/test/test_teleop_brush_direction.py
```

Run from the repository root with Python 3, Node.js and Playwright/Chromium
available. Set `PYTHON` to a Python executable if `python3` is unavailable, and
optionally `PLAYWRIGHT_CHANNEL=msedge` to use installed Edge. These are test-only
dependencies. Physical controller detection, ROS execution and robot motion
still need validation on the deployment system.

References: [Gamepad standard mapping](https://w3c.github.io/gamepad/#remapping),
[browser Gamepad API](https://developer.mozilla.org/en-US/docs/Web/API/Gamepad_API).
