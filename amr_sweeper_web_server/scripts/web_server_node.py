#!/usr/bin/env python3

from __future__ import annotations

import json
import errno
import threading
import urllib.parse
from html import escape
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import rclpy
from amr_sweeper_fsm.msg import FSMState, FSMStatus
from amr_sweeper_mission_executor.srv import (
    EndMission,
    ExecuteMission,
    ListExecutableMissions,
    UploadVda5050Mission,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, NavSatFix


def _resolve_path(configured_path: str) -> Path:
    path = Path(configured_path).expanduser()
    if path.is_absolute():
        return path
    return Path.cwd() / path


class MissionThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class MissionWebServerNode(Node):
    def __init__(self) -> None:
        super().__init__("web_server_node")

        self._http_host = self.declare_parameter("http_host", "0.0.0.0").value
        self._http_port = int(self.declare_parameter("http_port", 8080).value)
        self._site_title = self.declare_parameter("site_title", "AMR Sweeper Mission Control").value
        self._public_base_url = self.declare_parameter(
            "public_base_url",
            "http://192.168.2.1:8080",
        ).value
        self._missions_log_directory = self.declare_parameter(
            "missions_log_directory",
            "src/missions_log",
        ).value
        self._active_execution_pointer_filename = self.declare_parameter(
            "active_execution_pointer_filename",
            "active_execution.json",
        ).value
        self._list_missions_service = self.declare_parameter(
            "list_missions_service",
            "list_executable_missions",
        ).value
        self._execute_mission_service = self.declare_parameter(
            "execute_mission_service",
            "execute_mission",
        ).value
        self._upload_vda5050_mission_service = self.declare_parameter(
            "upload_vda5050_mission_service",
            "upload_vda5050_mission",
        ).value
        self._end_mission_service = self.declare_parameter(
            "end_mission_service",
            "end_mission",
        ).value
        self._fsm_state_topic = self.declare_parameter("fsm_state_topic", "fsm_state").value
        self._fsm_status_topic = self.declare_parameter("fsm_status_topic", "fsm_status").value
        self._gnss_topic = self.declare_parameter("gnss_topic", "gnss/navsat").value
        self._battery_topic = self.declare_parameter("battery_topic", "battery_state").value

        self._list_missions_client = self.create_client(
            ListExecutableMissions,
            self._list_missions_service,
        )
        self._execute_mission_client = self.create_client(
            ExecuteMission,
            self._execute_mission_service,
        )
        self._upload_vda5050_mission_client = self.create_client(
            UploadVda5050Mission,
            self._upload_vda5050_mission_service,
        )
        self._end_mission_client = self.create_client(
            EndMission,
            self._end_mission_service,
        )

        self._state_lock = threading.Lock()
        self._latest_fsm_state: dict[str, Any] | None = None
        self._latest_fsm_status: dict[str, Any] | None = None
        self._latest_navsat: dict[str, Any] | None = None
        self._latest_battery: dict[str, Any] | None = None

        self.create_subscription(FSMState, self._fsm_state_topic, self._handle_fsm_state, 10)
        self.create_subscription(FSMStatus, self._fsm_status_topic, self._handle_fsm_status, 10)
        self.create_subscription(NavSatFix, self._gnss_topic, self._handle_navsat, 10)
        self.create_subscription(BatteryState, self._battery_topic, self._handle_battery, 10)

        self._http_server: ThreadingHTTPServer | None = None

    def start_http_server(self) -> None:
        handler = self._build_handler()
        try:
            self._http_server = MissionThreadingHTTPServer((self._http_host, self._http_port), handler)
        except OSError as exc:
            if exc.errno == errno.EADDRINUSE:
                raise RuntimeError(
                    f"HTTP listen address {self._http_host}:{self._http_port} is already in use. "
                    "Another web server instance may still be running."
                ) from exc
            raise
        self.get_logger().info(
            f"Mission web server listening on http://{self._http_host}:{self._http_port}"
        )

    def stop_http_server(self) -> None:
        if self._http_server is None:
            return
        self._http_server.shutdown()
        self._http_server.server_close()
        self._http_server = None

    def serve_forever(self) -> None:
        if self._http_server is None:
            raise RuntimeError("HTTP server not initialized")
        self._http_server.serve_forever()

    def _build_handler(self):
        node = self

        class MissionWebRequestHandler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802
                parsed = urllib.parse.urlparse(self.path)
                if parsed.path == "/":
                    self._send_html(node.render_index_html())
                    return
                if parsed.path == "/api/status":
                    self._send_json(HTTPStatus.OK, node.status_snapshot())
                    return
                if parsed.path == "/api/missions":
                    try:
                        self._send_json(HTTPStatus.OK, node.list_executable_missions())
                    except Exception as exc:  # noqa: BLE001
                        self._send_json(HTTPStatus.BAD_GATEWAY, {"success": False, "message": str(exc)})
                    return
                self._send_json(HTTPStatus.NOT_FOUND, {"success": False, "message": "Not found"})

            def do_POST(self) -> None:  # noqa: N802
                parsed = urllib.parse.urlparse(self.path)
                payload = self._read_json_body()

                if parsed.path.startswith("/api/missions/") and parsed.path.endswith("/execute"):
                    mission_segment = parsed.path[len("/api/missions/"):-len("/execute")]
                    mission_id = urllib.parse.unquote(mission_segment.rstrip("/"))
                    if not mission_id:
                        self._send_json(
                            HTTPStatus.BAD_REQUEST,
                            {"success": False, "message": "mission_id is required"},
                        )
                        return
                    try:
                        response = node.execute_manual_mission(mission_id, payload)
                        self._send_json(HTTPStatus.OK, response)
                    except Exception as exc:  # noqa: BLE001
                        self._send_json(HTTPStatus.BAD_GATEWAY, {"success": False, "message": str(exc)})
                    return

                if parsed.path == "/api/missions/upload-vda5050":
                    try:
                        response = node.upload_vda5050_mission(payload)
                        self._send_json(HTTPStatus.OK, response)
                    except Exception as exc:  # noqa: BLE001
                        self._send_json(HTTPStatus.BAD_GATEWAY, {"success": False, "message": str(exc)})
                    return

                if parsed.path == "/api/stop":
                    try:
                        response = node.stop_active_mission(payload)
                        self._send_json(HTTPStatus.OK, response)
                    except Exception as exc:  # noqa: BLE001
                        self._send_json(HTTPStatus.BAD_GATEWAY, {"success": False, "message": str(exc)})
                    return

                self._send_json(HTTPStatus.NOT_FOUND, {"success": False, "message": "Not found"})

            def log_message(self, format: str, *args: Any) -> None:
                node.get_logger().info(f"HTTP {self.address_string()} - {format % args}")

            def _read_json_body(self) -> dict[str, Any]:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0:
                    return {}
                raw_body = self.rfile.read(length)
                if not raw_body:
                    return {}
                try:
                    decoded = json.loads(raw_body.decode("utf-8"))
                except json.JSONDecodeError as exc:
                    raise RuntimeError(f"Invalid JSON body: {exc}") from exc
                if not isinstance(decoded, dict):
                    raise RuntimeError("JSON body must be an object")
                return decoded

            def _send_html(self, body: str) -> None:
                encoded = body.encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

            def _send_json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
                encoded = json.dumps(payload).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

        return MissionWebRequestHandler

    def _handle_fsm_state(self, message: FSMState) -> None:
        with self._state_lock:
            self._latest_fsm_state = {
                "stamp": self._time_to_dict(message.stamp),
                "current_state": message.current_state,
                "current_profile": int(message.current_profile),
            }

    def _handle_fsm_status(self, message: FSMStatus) -> None:
        with self._state_lock:
            self._latest_fsm_status = {
                "stamp": self._time_to_dict(message.stamp),
                "current_state": message.current_state,
                "current_lifecycle_state": message.current_lifecycle_state,
                "current_profile": int(message.current_profile),
                "transitioning_to_profile": int(message.transitioning_to_profile),
                "transition_status": message.transition_status,
                "last_requester": message.last_requester,
                "last_request_priority": int(message.last_request_priority),
                "effective_priority_gate": int(message.effective_priority_gate),
                "priority_age_sec": float(message.priority_age_sec),
                "last_message": message.last_message,
            }

    def _handle_navsat(self, message: NavSatFix) -> None:
        with self._state_lock:
            self._latest_navsat = {
                "stamp": self._time_to_dict(message.header.stamp),
                "latitude": float(message.latitude),
                "longitude": float(message.longitude),
                "altitude": float(message.altitude),
                "status": int(message.status.status),
                "service": int(message.status.service),
                "position_covariance_type": int(message.position_covariance_type),
            }

    def _handle_battery(self, message: BatteryState) -> None:
        with self._state_lock:
            self._latest_battery = {
                "stamp": self._time_to_dict(message.header.stamp),
                "percentage": None if message.percentage != message.percentage else float(message.percentage),
                "voltage": float(message.voltage),
                "current": float(message.current),
                "charge": float(message.charge),
                "capacity": float(message.capacity),
                "design_capacity": float(message.design_capacity),
                "temperature": float(message.temperature),
                "power_supply_status": int(message.power_supply_status),
                "power_supply_health": int(message.power_supply_health),
                "power_supply_technology": int(message.power_supply_technology),
                "present": bool(message.present),
            }

    def list_executable_missions(self) -> dict[str, Any]:
        response = self._call_service(
            self._list_missions_client,
            ListExecutableMissions.Request(),
            timeout_sec=5.0,
            service_name=self._list_missions_service,
        )
        missions = []
        for mission_id, mission_type, execution_mode, running_profile_id, is_manual, artifacts_ready in zip(
            response.mission_ids,
            response.mission_types,
            response.execution_modes,
            response.running_profile_ids,
            response.is_manual,
            response.artifacts_ready,
        ):
            missions.append(
                {
                    "mission_id": mission_id,
                    "mission_type": mission_type,
                    "execution_mode": execution_mode,
                    "running_profile_id": int(running_profile_id),
                    "is_manual": bool(is_manual),
                    "artifacts_ready": bool(artifacts_ready),
                }
            )
        return {
            "success": bool(response.success),
            "message": response.message,
            "missions": missions,
        }

    def execute_manual_mission(self, mission_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        request = ExecuteMission.Request()
        request.mission_id = mission_id
        request.mission_execution_directory = str(payload.get("mission_execution_directory", ""))
        request.mission_window_start = str(payload.get("mission_window_start", ""))
        request.mission_window_end = str(payload.get("mission_window_end", ""))
        request.requester = str(payload.get("requester", "web_server_node"))
        request.priority = int(payload.get("priority", 200))
        request.force = bool(payload.get("force", False))
        request.reason = str(payload.get("reason", "manual mission requested from HTTP UI"))

        response = self._call_service(
            self._execute_mission_client,
            request,
            timeout_sec=15.0,
            service_name=self._execute_mission_service,
        )
        return {
            "success": bool(response.success),
            "message": response.message,
            "mission_id": mission_id,
            "mission_execution_directory": response.mission_execution_directory,
            "execution_context_file": response.execution_context_file,
            "running_profile_id": int(response.running_profile_id),
        }

    def upload_vda5050_mission(self, payload: dict[str, Any]) -> dict[str, Any]:
        request = UploadVda5050Mission.Request()
        request.mission_id = str(payload.get("mission_id", ""))
        request.mission_json = str(payload.get("mission_json", ""))
        request.overwrite_existing = bool(payload.get("overwrite_existing", False))

        response = self._call_service(
            self._upload_vda5050_mission_client,
            request,
            timeout_sec=10.0,
            service_name=self._upload_vda5050_mission_service,
        )
        return {
            "success": bool(response.success),
            "message": response.message,
            "mission_id": response.mission_id,
            "mission_file": response.mission_file,
            "mission_folder": response.mission_folder,
            "mission_type": response.mission_type,
            "running_profile_id": int(response.running_profile_id),
        }

    def stop_active_mission(self, payload: dict[str, Any]) -> dict[str, Any]:
        request = EndMission.Request()
        request.mission_id = str(payload.get("mission_id", ""))
        request.reason = str(payload.get("reason", "mission stop requested from HTTP UI"))
        request.outcome = str(payload.get("outcome", "aborted"))
        request.requester = str(payload.get("requester", "web_server_node"))
        request.priority = int(payload.get("priority", 200))
        request.force = bool(payload.get("force", False))
        request.request_idling = bool(payload.get("request_idling", True))

        response = self._call_service(
            self._end_mission_client,
            request,
            timeout_sec=15.0,
            service_name=self._end_mission_service,
        )
        return {
            "success": bool(response.success),
            "message": response.message,
            "mission_execution_directory": response.mission_execution_directory,
            "execution_context_file": response.execution_context_file,
        }

    def status_snapshot(self) -> dict[str, Any]:
        with self._state_lock:
            fsm_state = dict(self._latest_fsm_state) if self._latest_fsm_state is not None else None
            fsm_status = dict(self._latest_fsm_status) if self._latest_fsm_status is not None else None
            navsat = dict(self._latest_navsat) if self._latest_navsat is not None else None
            battery = dict(self._latest_battery) if self._latest_battery is not None else None

        active_execution = self._load_active_execution()
        return {
            "success": True,
            "site_title": self._site_title,
            "public_base_url": self._public_base_url,
            "fsm_state": fsm_state,
            "fsm_status": fsm_status,
            "position": navsat,
            "battery": battery,
            "active_execution": active_execution,
        }

    def _load_active_execution(self) -> dict[str, Any] | None:
        pointer_path = _resolve_path(self._missions_log_directory) / self._active_execution_pointer_filename
        if not pointer_path.exists():
            return None
        try:
            return json.loads(pointer_path.read_text(encoding="utf-8"))
        except Exception as exc:  # noqa: BLE001
            return {"error": f"Failed to read active execution pointer: {exc}", "path": str(pointer_path)}

    def _call_service(self, client, request, timeout_sec: float, service_name: str):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Service '{service_name}' is unavailable")

        future = client.call_async(request)
        completed = threading.Event()
        result_holder: dict[str, Any] = {}

        def _done_callback(done_future) -> None:
            try:
                result_holder["response"] = done_future.result()
            except Exception as exc:  # noqa: BLE001
                result_holder["exception"] = exc
            finally:
                completed.set()

        future.add_done_callback(_done_callback)
        if not completed.wait(timeout_sec):
            raise RuntimeError(f"Timed out waiting for service '{service_name}' response")
        if "exception" in result_holder:
            raise RuntimeError(f"Service '{service_name}' failed: {result_holder['exception']}")
        return result_holder["response"]

    @staticmethod
    def _time_to_dict(stamp) -> dict[str, int]:
        return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}

    def render_index_html(self) -> str:
        title = escape(self._site_title)
        public_base_url = escape(self._public_base_url)
        return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{title}</title>
  <style>
    :root {{
      --bg: #f4f1ea;
      --card: #fffaf2;
      --ink: #10212b;
      --muted: #5d6c74;
      --accent: #0f766e;
      --accent-strong: #115e59;
      --warn: #b45309;
      --danger: #b91c1c;
      --line: #d9d3c7;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "Segoe UI", "Helvetica Neue", sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(15, 118, 110, 0.12), transparent 30%),
        linear-gradient(160deg, #f5f1e7 0%, #ece8df 100%);
    }}
    main {{
      max-width: 1100px;
      margin: 0 auto;
      padding: 24px;
    }}
    h1, h2 {{
      margin: 0 0 12px;
      letter-spacing: 0.02em;
    }}
    .hero {{
      padding: 24px;
      border-bottom: 4px solid var(--accent);
      background: rgba(255, 250, 242, 0.92);
      border-radius: 20px;
      box-shadow: 0 16px 40px rgba(16, 33, 43, 0.08);
    }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
      gap: 16px;
      margin-top: 18px;
    }}
    .card {{
      background: rgba(255, 250, 242, 0.96);
      border: 1px solid var(--line);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 10px 28px rgba(16, 33, 43, 0.06);
    }}
    .stat {{
      font-size: 1.8rem;
      font-weight: 700;
      margin: 4px 0 8px;
    }}
    .muted {{
      color: var(--muted);
      font-size: 0.95rem;
    }}
    .mission-list {{
      display: grid;
      gap: 12px;
      margin-top: 16px;
    }}
    .mission {{
      border: 1px solid var(--line);
      border-radius: 16px;
      padding: 14px;
      background: #fffdf8;
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 14px;
      flex-wrap: wrap;
    }}
    button {{
      border: 0;
      border-radius: 999px;
      padding: 10px 16px;
      font-size: 0.95rem;
      cursor: pointer;
      color: white;
      background: var(--accent);
    }}
    button:hover {{ background: var(--accent-strong); }}
    button.stop {{ background: var(--danger); }}
    button:disabled {{
      cursor: not-allowed;
      background: #98a3aa;
      color: #eef2f4;
    }}
    button.stop:disabled {{
      background: #98a3aa;
    }}
    pre {{
      margin: 0;
      white-space: pre-wrap;
      word-break: break-word;
      font-size: 0.85rem;
      color: var(--muted);
    }}
    .banner {{
      margin-top: 16px;
      padding: 12px 14px;
      border-radius: 12px;
      display: none;
    }}
    .banner.show {{ display: block; }}
    .banner.ok {{ background: rgba(15, 118, 110, 0.12); color: var(--accent-strong); }}
    .banner.error {{ background: rgba(185, 28, 28, 0.12); color: var(--danger); }}
  </style>
</head>
<body>
  <main>
    <section class="hero">
      <h1>{title}</h1>
      <p class="muted">Open <strong>{public_base_url}</strong> on the robot Wi-Fi for manual mission launch, saved autonomous mission execution, VDA5050 upload, and live runtime status.</p>
      <div id="banner" class="banner"></div>
    </section>

    <section class="grid">
      <article class="card">
        <h2>FSM</h2>
        <div id="fsm-state" class="stat">Waiting...</div>
        <div id="fsm-profile" class="muted">Profile: -</div>
        <div id="fsm-transition" class="muted">Transition: -</div>
      </article>
      <article class="card">
        <h2>Active Mission</h2>
        <div id="active-mission" class="stat">No Active Missions</div>
        <div id="active-directory" class="muted">Run folder: -</div>
        <div style="margin-top: 12px;">
          <button class="stop" id="stop-button" disabled>Stop Mission</button>
        </div>
      </article>
      <article class="card">
        <h2>Position</h2>
        <div id="position-lat" class="stat">--</div>
        <div id="position-lon" class="muted">Longitude: --</div>
        <div id="position-alt" class="muted">Altitude: --</div>
      </article>
      <article class="card">
        <h2>Battery</h2>
        <div id="battery-percent" class="stat">--</div>
        <div id="battery-voltage" class="muted">Voltage: --</div>
        <div id="battery-current" class="muted">Current: --</div>
      </article>
    </section>

    <section class="card" style="margin-top: 18px;">
      <h2>Executable Missions</h2>
      <div id="mission-list" class="mission-list"></div>
    </section>

    <section class="card" style="margin-top: 18px;">
      <h2>Upload VDA5050 Mission</h2>
      <div class="muted">Paste a VDA5050 mission JSON document. You can optionally provide a mission id; otherwise `orderId` is used.</div>
      <div style="display: grid; gap: 12px; margin-top: 14px;">
        <input id="upload-file" type="file" accept=".json,application/json" style="padding: 12px; border-radius: 12px; border: 1px solid var(--line); background: white;">
        <input id="upload-mission-id" type="text" placeholder="Optional mission id" style="padding: 12px; border-radius: 12px; border: 1px solid var(--line);">
        <label class="muted"><input id="upload-overwrite" type="checkbox"> Overwrite existing mission with same id</label>
        <textarea id="upload-json" rows="14" placeholder='{{"orderId":"field_block_12","nodes":[...],"edges":[...]}}' style="width: 100%; padding: 12px; border-radius: 12px; border: 1px solid var(--line); font-family: monospace;"></textarea>
        <div>
          <button id="upload-button">Upload Mission</button>
        </div>
      </div>
    </section>

    <section class="card" style="margin-top: 18px;">
      <h2>Raw Status</h2>
      <pre id="raw-status">{{}}</pre>
    </section>
  </main>

  <script>
    const banner = document.getElementById('banner');
    const missionList = document.getElementById('mission-list');
    const rawStatus = document.getElementById('raw-status');

    function setBanner(kind, message) {{
      banner.className = `banner show ${{kind}}`;
      banner.textContent = message;
      setTimeout(() => {{
        banner.className = 'banner';
        banner.textContent = '';
      }}, 5000);
    }}

    async function loadStatus() {{
      const response = await fetch('/api/status', {{ cache: 'no-store' }});
      const data = await response.json();
      const fsm = data.fsm_status || data.fsm_state || {{}};
      const position = data.position || {{}};
      const battery = data.battery || {{}};
      const active = data.active_execution || {{}};
      const hasActiveMission = Boolean(
        active &&
        active.mission_id &&
        active.active !== false
      );

      document.getElementById('fsm-state').textContent = fsm.current_state || 'Unknown';
      document.getElementById('fsm-profile').textContent = `Profile: ${{fsm.current_profile ?? '-'}}`;
      document.getElementById('fsm-transition').textContent = `Transition: ${{fsm.transition_status || '-'}}`;

      document.getElementById('position-lat').textContent =
        position.latitude !== undefined ? `Lat: ${{Number(position.latitude).toFixed(7)}}` : '--';
      document.getElementById('position-lon').textContent =
        position.longitude !== undefined ? `Longitude: ${{Number(position.longitude).toFixed(7)}}` : 'Longitude: --';
      document.getElementById('position-alt').textContent =
        position.altitude !== undefined ? `Altitude: ${{Number(position.altitude).toFixed(2)}} m` : 'Altitude: --';

      document.getElementById('battery-percent').textContent =
        battery.percentage !== null && battery.percentage !== undefined
          ? `${{Math.round(Number(battery.percentage) * 100)}}%`
          : '--';
      document.getElementById('battery-voltage').textContent =
        battery.voltage !== undefined ? `Voltage: ${{Number(battery.voltage).toFixed(2)}} V` : 'Voltage: --';
      document.getElementById('battery-current').textContent =
        battery.current !== undefined ? `Current: ${{Number(battery.current).toFixed(2)}} A` : 'Current: --';

      document.getElementById('active-mission').textContent =
        hasActiveMission ? active.mission_id : 'No Active Missions';
      document.getElementById('active-directory').textContent =
        `Run folder: ${{hasActiveMission ? (active.mission_run_directory || '-') : '-'}}`;
      document.getElementById('stop-button').disabled = !hasActiveMission;

      rawStatus.textContent = JSON.stringify(data, null, 2);
    }}

    async function loadMissions() {{
      const response = await fetch('/api/missions', {{ cache: 'no-store' }});
      const data = await response.json();
      missionList.innerHTML = '';

      for (const mission of data.missions || []) {{
        const item = document.createElement('div');
        item.className = 'mission';
        item.innerHTML = `
          <div>
            <strong>${{mission.mission_id}}</strong><br>
            <span class="muted">${{mission.is_manual ? 'Manual' : 'Autonomous'}} | Type: ${{mission.mission_type || '-'}} | Mode: ${{mission.execution_mode || '-'}} | RUNNING profile: ${{mission.running_profile_id}} | Artifacts: ${{mission.artifacts_ready ? 'ready' : 'pending build'}}</span>
          </div>
          <div>
            <button data-mission-id="${{mission.mission_id}}">Execute</button>
          </div>
        `;
        item.querySelector('button').addEventListener('click', async () => {{
          const executeResponse = await fetch(`/api/missions/${{encodeURIComponent(mission.mission_id)}}/execute`, {{
            method: 'POST',
            headers: {{ 'Content-Type': 'application/json' }},
            body: JSON.stringify({{}})
          }});
          const executeData = await executeResponse.json();
          setBanner(executeData.success ? 'ok' : 'error', executeData.message || 'Mission request completed');
          await loadStatus();
        }});
        missionList.appendChild(item);
      }}
    }}

    document.getElementById('upload-file').addEventListener('change', async (event) => {{
      const file = event.target.files && event.target.files[0];
      if (!file) {{
        return;
      }}
      const text = await file.text();
      document.getElementById('upload-json').value = text;
    }});

    document.getElementById('upload-button').addEventListener('click', async () => {{
      const missionId = document.getElementById('upload-mission-id').value;
      const missionJson = document.getElementById('upload-json').value;
      const overwriteExisting = document.getElementById('upload-overwrite').checked;
      const response = await fetch('/api/missions/upload-vda5050', {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/json' }},
        body: JSON.stringify({{
          mission_id: missionId,
          mission_json: missionJson,
          overwrite_existing: overwriteExisting
        }})
      }});
      const data = await response.json();
      setBanner(data.success ? 'ok' : 'error', data.message || 'Upload completed');
      if (data.success) {{
        document.getElementById('upload-mission-id').value = '';
      }}
      await loadMissions();
    }});

    document.getElementById('stop-button').addEventListener('click', async () => {{
      const response = await fetch('/api/stop', {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/json' }},
        body: JSON.stringify({{}})
      }});
      const data = await response.json();
      setBanner(data.success ? 'ok' : 'error', data.message || 'Stop request completed');
      await loadStatus();
    }});

    async function refresh() {{
      try {{
        await Promise.all([loadStatus(), loadMissions()]);
      }} catch (error) {{
        setBanner('error', error.message || 'Failed to reach mission web server');
      }}
    }}

    refresh();
    setInterval(loadStatus, 2000);
    setInterval(loadMissions, 10000);
  </script>
</body>
</html>
"""


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = MissionWebServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.start_http_server()
        node.serve_forever()
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"Mission web server startup failed: {exc}")
        return 1
    finally:
        node.stop_http_server()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
