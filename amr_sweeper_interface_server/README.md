# amr_sweeper_interface_server

Provides the AMR Sweeper operator backend/frontend web stack as a standalone layer 0 package.

Structure:
- `scripts/backend_node.py` contains the ROS/backend state, schedule, mission-control, and data API logic.
- `scripts/frontend_http_node.py` contains the HTTP server, route handling, and rendered frontend pages on top of the backend node.

Responsibilities:
- expose a local dashboard over HTTP
- show FSM state, GNSS latitude/longitude, battery state, and the active mission run folder
- execute built-in manual missions and saved autonomous missions through `amr_sweeper_mission_executor`
- stop the active mission through `amr_sweeper_mission_executor`
- upload a VDA5050 mission JSON payload into `/missions_from_db` so it becomes executable
- provide a dedicated `/record-map` workflow page for starting `RecordMap`, previewing the latest GNSS overlay on satellite imagery, and saving named autonomous missions from the latest recorded map

Default operator URL:
- `http://192.168.2.1:8080`

Launch:
- `ros2 launch amr_sweeper_interface_server amr_sweeper_interface_server.launch.py`

Network setup:
- Target robot Ethernet address: `192.168.2.1`
- Make sure that port 8080/tcp is opened on the Jetson for access from Wi-Fi clients connected to robots Wi-Fi access points. Run `ufw status` as root.
- If port is not opened, open it by executing following command as root: `ufw allow from 192.168.2.0/27 to 192.168.2.1 port 8080 proto tcp`.

Notes:
- the node binds to `0.0.0.0:8080` by default so operators on the router LAN can reach it
- the default public URL assumes the Jetson uses the Ethernet address `192.168.2.1/24`
- operators connect through the router's Wi-Fi network; the Jetson itself is expected to be wired to the router over Ethernet
- this assumes the robot uplink and router LAN use the `192.168.2.0/24` subnet
