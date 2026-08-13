# amr_sweeper_interface_server

Provides the AMR Sweeper operator interface stack as a standalone layer 0 package.

Structure:
- `scripts/backend_node.py` is the local-only backend entry point. It owns ROS topics/services, mission files, schedule files, and the raw JSONL API over a Unix domain socket.
- `scripts/frontend_http_node.py` is the network-facing web frontend. It serves the local operator pages and adapts `/api/v1` HTTP requests to backend JSONL requests.

Responsibilities:
- expose a local dashboard over TCP through the frontend node
- keep the backend API reachable only through local IPC
- show FSM state, GNSS latitude/longitude, battery state, and the active mission run folder
- execute built-in manual missions and saved autonomous missions through `amr_sweeper_mission_executor`
- stop the active mission through `amr_sweeper_mission_executor`
- upload a VDA5050 mission JSON payload into `/missions/database` so it becomes executable
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
