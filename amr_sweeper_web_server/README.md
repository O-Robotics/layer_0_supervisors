# amr_sweeper_web_server

Provides the AMR Sweeper HTTP operator UI as a standalone layer 0 package.

Responsibilities:
- expose a local dashboard over HTTP
- show FSM state, GNSS latitude/longitude, battery state, and the active mission run folder
- execute built-in manual missions and saved autonomous missions through `amr_sweeper_mission_executor`
- stop the active mission through `amr_sweeper_mission_executor`
- upload a VDA5050 mission JSON payload into the runtime missions directory so it becomes executable

Default operator URL:
- `http://192.168.2.1:8080`

Launch:
- `ros2 launch amr_sweeper_web_server amr_sweeper_web_server.launch.py`

Notes:
- the node binds to `0.0.0.0:8080` by default so operators on the robot Wi-Fi can reach it
- the default public URL assumes the robot uses the static Wi-Fi address `192.168.2.1/24`
