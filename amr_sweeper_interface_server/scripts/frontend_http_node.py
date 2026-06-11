#!/usr/bin/env python3

from __future__ import annotations

import errno
import json
import threading
import urllib.parse
from html import escape
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor

from backend_node import MissionBackendNode


class MissionThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class MissionFrontendHttpNode(MissionBackendNode):
    def __init__(self) -> None:
        super().__init__("frontend_http_node")
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
                if parsed.path == "/calendar":
                    self._send_html(node.render_calendar_html())
                    return
                if parsed.path == "/map":
                    self._send_html(node.render_map_html())
                    return
                if parsed.path == "/developer":
                    self._send_html(node.render_developer_html())
                    return
                if parsed.path == "/record-map":
                    self._send_html(node.render_record_map_html())
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
                if parsed.path.startswith("/api/missions/") and parsed.path.endswith("/download"):
                    mission_segment = parsed.path[len("/api/missions/"):-len("/download")]
                    mission_id = urllib.parse.unquote(mission_segment.rstrip("/"))
                    if not mission_id:
                        self._send_json(
                            HTTPStatus.BAD_REQUEST,
                            {"success": False, "message": "mission_id is required"},
                        )
                        return
                    try:
                        mission_path = node.mission_file_path(mission_id)
                        self._send_download(mission_path, "application/json; charset=utf-8")
                    except Exception as exc:  # noqa: BLE001
                        self._send_json(HTTPStatus.NOT_FOUND, {"success": False, "message": str(exc)})
                    return
                if parsed.path == "/api/schedule":
                    try:
                        query = urllib.parse.parse_qs(parsed.query)
                        week = query.get("week", [""])[0]
                        self._send_json(HTTPStatus.OK, node.schedule_snapshot(week))
                    except Exception as exc:  # noqa: BLE001
                        self._send_json(HTTPStatus.BAD_GATEWAY, {"success": False, "message": str(exc)})
                    return
                if parsed.path == "/api/map-data":
                    try:
                        self._send_json(HTTPStatus.OK, node.map_snapshot())
                    except Exception as exc:  # noqa: BLE001
                        self._send_json(HTTPStatus.BAD_GATEWAY, {"success": False, "message": str(exc)})
                    return
                if parsed.path == "/api/record-map":
                    try:
                        self._send_json(HTTPStatus.OK, node.record_map_snapshot())
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

                if parsed.path == "/api/reboot":
                    try:
                        response = node.request_reinitialize(payload)
                        self._send_json(HTTPStatus.OK, response)
                    except Exception as exc:  # noqa: BLE001
                        self._send_json(HTTPStatus.BAD_GATEWAY, {"success": False, "message": str(exc)})
                    return
                if parsed.path == "/api/safety/clear":
                    try:
                        response = node.clear_safety_stop(payload)
                        self._send_json(HTTPStatus.OK, response)
                    except Exception as exc:  # noqa: BLE001
                        self._send_json(HTTPStatus.BAD_GATEWAY, {"success": False, "message": str(exc)})
                    return
                if parsed.path == "/api/safety/stop":
                    try:
                        response = node.trigger_safety_stop(payload)
                        self._send_json(HTTPStatus.OK, response)
                    except Exception as exc:  # noqa: BLE001
                        self._send_json(HTTPStatus.BAD_GATEWAY, {"success": False, "message": str(exc)})
                    return
                if parsed.path == "/api/record-map/start":
                    try:
                        response = node.start_record_map(payload)
                        self._send_json(HTTPStatus.OK, response)
                    except Exception as exc:  # noqa: BLE001
                        self._send_json(HTTPStatus.BAD_GATEWAY, {"success": False, "message": str(exc)})
                    return
                if parsed.path == "/api/record-map/stop":
                    try:
                        response = node.stop_record_map(payload)
                        self._send_json(HTTPStatus.OK, response)
                    except Exception as exc:  # noqa: BLE001
                        self._send_json(HTTPStatus.BAD_GATEWAY, {"success": False, "message": str(exc)})
                    return
                if parsed.path == "/api/record-map/save-mission":
                    try:
                        response = node.create_recorded_mission(payload)
                        self._send_json(HTTPStatus.OK, response)
                    except Exception as exc:  # noqa: BLE001
                        self._send_json(HTTPStatus.BAD_GATEWAY, {"success": False, "message": str(exc)})
                    return
                if parsed.path == "/api/schedule/entry":
                    try:
                        response = node.save_planned_schedule_entry(payload)
                        self._send_json(HTTPStatus.OK, response)
                    except Exception as exc:  # noqa: BLE001
                        self._send_json(HTTPStatus.BAD_GATEWAY, {"success": False, "message": str(exc)})
                    return
                if parsed.path == "/api/schedule/entry/delete":
                    try:
                        response = node.delete_planned_schedule_entry(payload)
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

            def _send_file(self, path: Path, content_type: str) -> None:
                if not path.exists() or not path.is_file():
                    self._send_json(HTTPStatus.NOT_FOUND, {"success": False, "message": "Asset not found"})
                    return
                encoded = path.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "public, max-age=3600")
                self.send_header("Content-Length", str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

            def _send_download(self, path: Path, content_type: str) -> None:
                if not path.exists() or not path.is_file():
                    self._send_json(HTTPStatus.NOT_FOUND, {"success": False, "message": "Mission file not found"})
                    return
                encoded = path.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(encoded)))
                self.send_header("Content-Disposition", f'attachment; filename="{path.name}"')
                self.end_headers()
                self.wfile.write(encoded)

        return MissionWebRequestHandler

    def render_index_html(self) -> str:
        title = escape(self._site_title)
        public_base_url = escape(self._public_base_url)
        return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{title}</title>
  <link
    rel="stylesheet"
    href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
    integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY="
    crossorigin=""
  >
  <style>
    :root {{
      --bg: #1b1e20;
      --bg-alt: #2b2f31;
      --card: rgba(54, 58, 60, 0.94);
      --card-strong: rgba(42, 46, 48, 0.98);
      --panel: rgba(88, 92, 94, 0.48);
      --ink: #f5f1df;
      --muted: #c4bb98;
      --accent: #fdca0f;
      --accent-strong: #ffe06b;
      --warn: #fdca0f;
      --danger: #ff7b5c;
      --line: rgba(253, 202, 15, 0.22);
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "Avenir Next", "Segoe UI", sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(253, 202, 15, 0.18), transparent 26%),
        radial-gradient(circle at top right, rgba(255, 255, 255, 0.06), transparent 20%),
        linear-gradient(180deg, var(--bg) 0%, var(--bg-alt) 100%);
    }}
    main {{
      max-width: 1100px;
      margin: 0 auto;
      padding: 24px;
    }}
    h1, h2 {{
      margin: 0 0 12px;
      letter-spacing: 0.09em;
      text-transform: uppercase;
      font-family: "Avenir Next Condensed", "Franklin Gothic Medium", "Arial Narrow", sans-serif;
    }}
    h3 {{
      text-transform: uppercase;
      letter-spacing: 0.08em;
      font-family: "Avenir Next Condensed", "Franklin Gothic Medium", "Arial Narrow", sans-serif;
    }}
    .hero {{
      padding: 24px;
      border: 1px solid rgba(253, 202, 15, 0.28);
      background:
        linear-gradient(135deg, rgba(253, 202, 15, 0.16), rgba(42, 46, 48, 0.18) 42%),
        var(--card-strong);
      border-radius: 20px;
      box-shadow: 0 18px 44px rgba(0, 0, 0, 0.28);
    }}
    h1 {{
      color: var(--accent);
      margin-bottom: 1px;
    }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
      gap: 16px;
      margin-top: 18px;
    }}
    .card {{
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 14px 34px rgba(0, 0, 0, 0.22);
      backdrop-filter: blur(4px);
    }}
    .status-card {{
      grid-column: span 2;
    }}
    .map-card {{
      grid-column: span 2;
      padding: 0;
      overflow: hidden;
    }}
    .status-sections {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 14px;
      margin-top: 14px;
    }}
    .status-panel {{
      border: 1px solid var(--line);
      border-radius: 16px;
      padding: 14px;
      background: var(--panel);
    }}
    .status-panel h3 {{
      margin: 0 0 10px;
      font-size: 1rem;
      letter-spacing: 0.02em;
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
    .inline-status {{
      display: inline-flex;
      align-items: center;
      gap: 8px;
    }}
    .spinner {{
      width: 0.9rem;
      height: 0.9rem;
      border-radius: 50%;
      border: 2px solid rgba(253, 202, 15, 0.28);
      border-top-color: var(--accent);
      animation: spin 0.85s linear infinite, pulse-fade 1.2s ease-in-out infinite;
      flex: 0 0 auto;
    }}
    button {{
      border: 0;
      border-radius: 999px;
      padding: 10px 16px;
      font-size: 0.95rem;
      cursor: pointer;
      color: #08100a;
      background: var(--accent);
      font-weight: 700;
      letter-spacing: 0.04em;
      text-transform: uppercase;
    }}
    button:hover {{ background: var(--accent-strong); }}
    button.stop {{ background: var(--danger); }}
    button:disabled {{
      cursor: not-allowed;
      background: #5c5b55;
      color: #d7ddd8;
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
    .banner.warn {{ background: rgba(180, 83, 9, 0.12); color: var(--warn); }}
    .nav {{
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      margin-top: 14px;
    }}
    .nav-link {{
      display: inline-block;
      text-decoration: none;
      color: var(--ink);
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 8px 14px;
      background: rgba(52, 53, 53, 0.72);
      font-size: 0.92rem;
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }}
    .actions {{
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      margin-top: 12px;
    }}
    .live-strip {{
      margin-top: 0;
      display: flex;
      align-items: center;
      gap: 12px;
      flex-wrap: wrap;
    }}
    .live-pill {{
      display: inline-flex;
      align-items: center;
      gap: 8px;
      border-radius: 999px;
      padding: 0;
      background: transparent;
      border: none;
      font-size: 0.95rem;
      color: var(--muted);
    }}
    .live-status-text.connected {{
      color: #22c55e;
    }}
    .live-status-text.disconnected {{
      color: #ef4444;
    }}
    .live-dot {{
      width: 10px;
      height: 10px;
      border-radius: 50%;
      background: #ef4444;
      box-shadow: 0 0 0 0 rgba(239, 68, 68, 0.45);
      animation: pulse 1.2s infinite;
    }}
    .live-dot.connected {{
      background: #22c55e;
      box-shadow: 0 0 0 0 rgba(34, 197, 94, 0.45);
    }}
    .clock-block {{
      display: flex;
      align-items: baseline;
      gap: 10px;
      flex-wrap: wrap;
      color: var(--muted);
    }}
    .clock-value {{
      font-size: 1rem;
      font-weight: 700;
      color: var(--ink);
      letter-spacing: 0.04em;
    }}
    .map-info {{
      padding: 18px 18px 0;
    }}
    .position-map {{
      width: 100%;
      min-height: 360px;
    }}
    @keyframes pulse {{
      0% {{ box-shadow: 0 0 0 0 rgba(255, 224, 107, 0.45); }}
      70% {{ box-shadow: 0 0 0 10px rgba(255, 224, 107, 0); }}
      100% {{ box-shadow: 0 0 0 0 rgba(255, 224, 107, 0); }}
    }}
    @keyframes spin {{
      from {{ transform: rotate(0deg); }}
      to {{ transform: rotate(360deg); }}
    }}
    @keyframes pulse-fade {{
      0%, 100% {{ opacity: 0.45; }}
      50% {{ opacity: 1; }}
    }}
    .log-list {{
      display: grid;
      gap: 10px;
      margin-top: 14px;
    }}
    .log-entry {{
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 12px;
      background: var(--panel);
    }}
    .log-entry.warn {{
      border-color: rgba(180, 83, 9, 0.35);
    }}
    .log-entry.error, .log-entry.fatal {{
      border-color: rgba(185, 28, 28, 0.35);
    }}
    .log-meta {{
      font-size: 0.82rem;
      color: var(--muted);
      margin-bottom: 6px;
    }}
    .safety-card {{
      border-color: rgba(185, 28, 28, 0.2);
    }}
    .safety-card.active {{
      background: rgba(185, 28, 28, 0.08);
      border-color: rgba(185, 28, 28, 0.35);
    }}
    .safety-action {{
      margin-top: 8px;
    }}
    .safety-action button {{
      width: 100%;
      min-height: 112px;
      font-size: 1.15rem;
      letter-spacing: 0.08em;
    }}
    .safety-action button.stop {{
      background: #d22c2c;
      color: #fff8f6;
    }}
    .safety-action button.stop:hover {{
      background: #ec3b3b;
    }}
    @media (max-width: 780px) {{
      .status-card {{
        grid-column: span 1;
      }}
    }}
  </style>
</head>
<body>
  <main>
    <section class="hero">
      <h1>AMR-Sweeper</h1>
      <div id="banner" class="banner"></div>
      <div class="live-strip">
        <div class="live-pill">
          <span id="live-dot" class="live-dot"></span>
          <span id="live-status" class="live-status-text">CONNECTING</span>
        </div>
      </div>
      <div class="nav">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Calendar</a>
        <a class="nav-link" href="/map">Missions</a>
        <a class="nav-link" href="/record-map">Record Map</a>
        <a class="nav-link" href="/developer">Developer</a>
      </div>
    </section>

    <section class="grid">
      <article id="safety-card" class="card safety-card status-card">
        <h2>System Status</h2>
        <div class="status-sections">
          <section class="status-panel">
            <div id="fsm-state" class="stat">Waiting...</div>
            <div id="fsm-profile" class="muted">Profile: -</div>
            <div id="fsm-transition" class="muted">Transition: -</div>
            <div class="actions">
              <button id="reboot-button">Reboot</button>
            </div>
          </section>
          <section id="safety-card" class="status-panel safety-card">
            <div class="safety-action">
              <button class="stop" id="safety-stop-button">SAFETY STOP</button>
            </div>
          </section>
          <section class="status-panel">
            <h3>Active Mission</h3>
            <div id="active-mission" class="stat">No Active Missions</div>
            <div id="active-directory" class="muted">Run folder: -</div>
            <div class="actions">
              <button class="stop" id="stop-button" disabled>Stop Mission</button>
            </div>
          </section>
          <section class="status-panel">
            <h3>Battery</h3>
            <div id="battery-percent" class="stat">--</div>
            <div id="battery-voltage" class="muted">Voltage: --</div>
            <div id="battery-current" class="muted">Current: --</div>
          </section>
        </div>
      </article>
      <article class="card map-card">
        <div class="map-info">
          <h2>Position</h2>
          <div id="position-lat" class="muted">Latitude: --</div>
          <div id="position-lon" class="muted">Longitude: --</div>
          <div id="position-alt" class="muted">Altitude: --</div>
        </div>
        <div id="dashboard-position-map" class="position-map"></div>
      </article>
    </section>

  </main>

  <script
    src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
    integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo="
    crossorigin=""
  ></script>
  <script>
    const banner = document.getElementById('banner');
    let lastStatusEpochMs = 0;
    let lastSafetyLatched = false;
    let lastSafetyCanClear = false;
    let dashboardMap = null;
    let dashboardMarker = null;

    function ensureDashboardMap() {{
      if (dashboardMap) {{
        return dashboardMap;
      }}
      dashboardMap = L.map('dashboard-position-map', {{ zoomControl: true }}).setView([55.6761, 12.5683], 18);
      L.tileLayer(
        'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}',
        {{ maxZoom: 20, attribution: '&copy; Esri' }}
      ).addTo(dashboardMap);
      return dashboardMap;
    }}

    function updateDashboardPositionMap(position) {{
      const map = ensureDashboardMap();
      if (dashboardMarker) {{
        map.removeLayer(dashboardMarker);
        dashboardMarker = null;
      }}
      if (
        !position ||
        position.latitude === undefined ||
        position.longitude === undefined
      ) {{
        return;
      }}
      const lat = Number(position.latitude);
      const lon = Number(position.longitude);
      if (!Number.isFinite(lat) || !Number.isFinite(lon)) {{
        return;
      }}
      dashboardMarker = L.circleMarker(
        [lat, lon],
        {{ radius: 7, color: '#ffffff', weight: 2, fillColor: '#1d4ed8', fillOpacity: 1 }}
      ).addTo(map);
      map.setView([lat, lon], Math.max(map.getZoom(), 18));
    }}

    function deriveStateFromProfile(profile) {{
      const numericProfile = Number(profile);
      if (!Number.isFinite(numericProfile)) {{
        return '';
      }}
      if (numericProfile >= 0 && numericProfile <= 99) {{
        return 'INITIALIZING';
      }}
      if (numericProfile >= 100 && numericProfile <= 199) {{
        return 'IDLING';
      }}
      if (numericProfile >= 200 && numericProfile <= 299) {{
        return 'RUNNING';
      }}
      if (numericProfile >= 300 && numericProfile <= 399) {{
        return 'CHARGING';
      }}
      if (numericProfile >= 400 && numericProfile <= 499) {{
        return 'FAULT';
      }}
      return '';
    }}

    function formatArrowValue(currentValue, nextValue) {{
      const currentText = currentValue ?? '-';
      const nextText = nextValue ?? '-';
      return `${{currentText}} -> ${{nextText}}`;
    }}

    function formatProfileValue(profile) {{
      if (profile === null || profile === undefined || profile === '') {{
        return '-';
      }}
      const numericProfile = Number(profile);
      if (!Number.isFinite(numericProfile)) {{
        return String(profile);
      }}
      return String(Math.trunc(numericProfile)).padStart(3, '0');
    }}

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
      const safety = data.safety_stop || {{}};
      const active = data.active_execution || {{}};
      const hasActiveMission = Boolean(
        active &&
        active.mission_id &&
        active.active !== false
      );
      lastStatusEpochMs = Date.now();

      const liveStatus = document.getElementById('live-status');
      liveStatus.textContent = 'CONNECTED';
      liveStatus.classList.add('connected');
      liveStatus.classList.remove('disconnected');
      document.getElementById('live-dot').classList.add('connected');

      const currentState = fsm.current_state || 'Unknown';
      const currentProfile = formatProfileValue(fsm.current_profile);
      const targetProfile = fsm.transitioning_to_profile ?? null;
      const transitionInProgress = fsm.transition_status === 'TRANSITIONING';
      const targetState = deriveStateFromProfile(targetProfile) || currentState;
      const formattedTargetProfile = formatProfileValue(targetProfile);

      document.getElementById('fsm-state').textContent = transitionInProgress
        ? formatArrowValue(currentState, targetState)
        : currentState;
      document.getElementById('fsm-profile').textContent = transitionInProgress
        ? `Profile: ${{formatArrowValue(currentProfile, formattedTargetProfile)}}`
        : `Profile: ${{currentProfile}}`;

      const transitionElement = document.getElementById('fsm-transition');
      if (transitionInProgress) {{
        transitionElement.innerHTML = '<span class="inline-status"><span class="spinner" aria-hidden="true"></span><span>Transition in progress</span></span>';
      }} else {{
        transitionElement.textContent = `Transition: ${{fsm.transition_status || '-'}}`;
      }}

      document.getElementById('position-lat').textContent =
        position.latitude !== undefined ? `Latitude: ${{Number(position.latitude).toFixed(7)}}` : 'Latitude: --';
      document.getElementById('position-lon').textContent =
        position.longitude !== undefined ? `Longitude: ${{Number(position.longitude).toFixed(7)}}` : 'Longitude: --';
      document.getElementById('position-alt').textContent =
        position.altitude !== undefined ? `Altitude: ${{Number(position.altitude).toFixed(2)}} m` : 'Altitude: --';
      updateDashboardPositionMap(position);

      document.getElementById('battery-percent').textContent =
        battery.percentage !== null && battery.percentage !== undefined
          ? `${{Math.round(Number(battery.percentage) * 100)}}%`
          : '--';
      document.getElementById('battery-voltage').textContent =
        battery.voltage !== undefined ? `Voltage: ${{Number(battery.voltage).toFixed(2)}} V` : 'Voltage: --';
      document.getElementById('battery-current').textContent =
        battery.current !== undefined ? `Current: ${{Number(battery.current).toFixed(2)}} A` : 'Current: --';

      const safetyCard = document.getElementById('safety-card');
      const safetyButton = document.getElementById('safety-stop-button');
      const safetyLatched = Boolean(safety.latched);
      const safetyCanClear = Boolean(safety.can_clear);
      const clearAvailableInSec = Number(safety.clear_available_in_sec ?? 0);
      lastSafetyLatched = safetyLatched;
      lastSafetyCanClear = safetyCanClear;
      safetyCard.classList.toggle('active', safetyLatched);
      if (safetyLatched) {{
        safetyButton.textContent = safetyCanClear
          ? 'CLEAR SAFETY STOP'
          : `CLEAR SAFETY STOP (${{
              Math.ceil(Math.max(0, clearAvailableInSec))
            }}s)`;
        safetyButton.disabled = !safetyCanClear;
      }} else {{
        safetyButton.textContent = 'SAFETY STOP';
        safetyButton.disabled = false;
      }}

      document.getElementById('active-mission').textContent =
        hasActiveMission ? active.mission_id : 'No Active Missions';
      document.getElementById('active-directory').textContent =
        `Run folder: ${{hasActiveMission ? (active.mission_run_directory || '-') : '-'}}`;
      document.getElementById('stop-button').disabled = !hasActiveMission;
    }}

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

    document.getElementById('reboot-button').addEventListener('click', async () => {{
      const response = await fetch('/api/reboot', {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/json' }},
        body: JSON.stringify({{}})
      }});
      const data = await response.json();
      setBanner(data.success ? 'ok' : 'error', data.message || 'Reboot request completed');
      await loadStatus();
    }});

    document.getElementById('safety-stop-button').addEventListener('click', async () => {{
      const path = lastSafetyLatched ? '/api/safety/clear' : '/api/safety/stop';
      const payload = lastSafetyLatched ? {{}} : {{
        sender: 'frontend_http_node',
        reason: 'safety stop requested from dashboard'
      }};
      const response = await fetch(path, {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/json' }},
        body: JSON.stringify(payload)
      }});
      const data = await response.json();
      setBanner(
        data.success ? 'ok' : 'error',
        data.message || (lastSafetyLatched ? 'Safety clear request completed' : 'Safety stop request completed')
      );
      await loadStatus();
    }});

    async function refresh() {{
      try {{
        await loadStatus();
      }} catch (error) {{
        const liveStatus = document.getElementById('live-status');
        liveStatus.textContent = 'DISCONNECTED';
        liveStatus.classList.add('disconnected');
        liveStatus.classList.remove('connected');
        document.getElementById('live-dot').classList.remove('connected');
        setBanner('error', error.message || 'Failed to reach mission web server');
      }}
    }}

    function refreshHeartbeat() {{
      const liveStatus = document.getElementById('live-status');
      const liveDot = document.getElementById('live-dot');
      if (!lastStatusEpochMs) {{
        liveStatus.textContent = 'CONNECTING';
        liveStatus.classList.remove('connected', 'disconnected');
        liveDot.classList.remove('connected');
        return;
      }}
      const ageSec = (Date.now() - lastStatusEpochMs) / 1000;
      if (ageSec < 2.5) {{
        liveStatus.textContent = 'CONNECTED';
        liveStatus.classList.add('connected');
        liveStatus.classList.remove('disconnected');
        liveDot.classList.add('connected');
      }} else {{
        liveStatus.textContent = 'DISCONNECTED';
        liveStatus.classList.add('disconnected');
        liveStatus.classList.remove('connected');
        liveDot.classList.remove('connected');
      }}
    }}

    refresh();
    setInterval(async () => {{
      try {{
        await loadStatus();
      }} catch (_error) {{
        const liveStatus = document.getElementById('live-status');
        liveStatus.textContent = 'DISCONNECTED';
        liveStatus.classList.add('disconnected');
        liveStatus.classList.remove('connected');
        document.getElementById('live-dot').classList.remove('connected');
      }}
    }}, 1000);
    setInterval(refreshHeartbeat, 250);
  </script>
</body>
</html>
"""

    def render_calendar_html(self) -> str:
        title = escape(self._site_title)
        return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{title} - Calendar</title>
  <style>
    :root {{
      --bg: #1b1e20;
      --bg-alt: #2b2f31;
      --card: rgba(54, 58, 60, 0.94);
      --panel: rgba(88, 92, 94, 0.48);
      --ink: #f5f1df;
      --muted: #c4bb98;
      --accent: #fdca0f;
      --accent-strong: #ffe06b;
      --line: rgba(253, 202, 15, 0.22);
      --work: #fdca0f;
      --nowork: #7f8a8f;
      --safety: #ff7b5c;
    }}
    body {{
      margin: 0;
      font-family: "Avenir Next", "Segoe UI", sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(253, 202, 15, 0.18), transparent 26%),
        linear-gradient(180deg, var(--bg) 0%, var(--bg-alt) 100%);
    }}
    main {{
      max-width: 1200px;
      margin: 0 auto;
      padding: 24px;
    }}
    .card {{
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 14px 34px rgba(0, 0, 0, 0.22);
      backdrop-filter: blur(4px);
    }}
    h1, h2 {{
      text-transform: uppercase;
      letter-spacing: 0.09em;
      font-family: "Avenir Next Condensed", "Franklin Gothic Medium", "Arial Narrow", sans-serif;
    }}
    h1 {{
      color: var(--accent);
    }}
    .nav {{
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      margin-top: 14px;
    }}
    .nav-link {{
      display: inline-block;
      text-decoration: none;
      color: var(--ink);
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 8px 14px;
      background: rgba(52, 53, 53, 0.72);
      font-size: 0.92rem;
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }}
    .toolbar {{
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 12px;
      flex-wrap: wrap;
      margin: 18px 0;
    }}
    .toolbar-group {{
      display: flex;
      align-items: center;
      gap: 10px;
      flex-wrap: wrap;
    }}
    .calendar-clock {{
      display: flex;
      align-items: baseline;
      gap: 10px;
      flex-wrap: wrap;
    }}
    .calendar-clock-value {{
      font-size: 1.1rem;
      font-weight: 700;
      color: var(--accent);
      letter-spacing: 0.04em;
    }}
    button {{
      border: 0;
      border-radius: 999px;
      padding: 10px 16px;
      font-size: 0.95rem;
      cursor: pointer;
      color: #08100a;
      background: var(--accent);
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.04em;
    }}
    button:hover {{ background: var(--accent-strong); }}
    .week-shell {{
      overflow-x: auto;
    }}
    .week-grid {{
      display: grid;
      grid-template-columns: 88px repeat(7, minmax(140px, 1fr));
      gap: 8px;
      min-width: 1100px;
      align-items: start;
    }}
    .corner, .day-head, .time-rail, .day-column {{
      border: 1px solid var(--line);
      border-radius: 14px;
      background: var(--panel);
    }}
    .corner, .day-head {{
      min-height: 86px;
      padding: 10px 12px;
    }}
    .day-head {{
      font-weight: 700;
      background: rgba(253, 202, 15, 0.14);
      display: flex;
      flex-direction: column;
      justify-content: space-between;
    }}
    .day-name {{
      font-size: 0.82rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--muted);
    }}
    .day-number {{
      font-weight: 700;
      font-size: 1.1rem;
    }}
    .corner {{
      display: flex;
      align-items: flex-end;
      justify-content: center;
      font-size: 0.82rem;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }}
    .time-rail {{
      min-height: 1536px;
      padding: 0;
      overflow: hidden;
    }}
    .time-slot {{
      height: 64px;
      padding: 6px 10px;
      display: flex;
      align-items: flex-start;
      justify-content: flex-end;
      color: var(--muted);
      font-size: 0.82rem;
      border-bottom: 1px solid rgba(255, 255, 255, 0.06);
    }}
    .day-column {{
      position: relative;
      min-height: 1536px;
      overflow: hidden;
      background:
        linear-gradient(to bottom, rgba(255, 255, 255, 0.06) 1px, transparent 1px);
      background-size: 100% 64px;
    }}
    .event-chip {{
      position: absolute;
      left: 8px;
      right: 8px;
      border-radius: 12px;
      padding: 8px 10px;
      font-size: 0.78rem;
      color: #101214;
      overflow: hidden;
      border: 1px solid rgba(16, 18, 20, 0.18);
      box-shadow: 0 8px 16px rgba(0, 0, 0, 0.18);
      z-index: 1;
    }}
    .event-chip.planned {{
      opacity: 0.48;
      border-style: dashed;
      z-index: 1;
    }}
    .event-chip.actual {{
      opacity: 0.96;
      z-index: 3;
      box-shadow: 0 10px 18px rgba(0, 0, 0, 0.26);
    }}
    .event-chip.planned.WORK {{ background: #8aa4b8; color: #0f1418; }}
    .event-chip.planned.NO_WORK {{ background: #6f7880; color: #f5f1df; }}
    .event-chip.planned.SAFETY {{ background: #b78888; color: #161111; }}
    .event-chip.actual.WORK {{ background: var(--work); color: #101214; }}
    .event-chip.actual.NO_WORK {{ background: var(--nowork); color: var(--ink); }}
    .event-chip.actual.SAFETY {{ background: var(--safety); color: #fff4ec; }}
    .event-chip.SAFETY {{
      background: #ff3b30;
      color: #fff7f5;
      border-color: rgba(90, 0, 0, 0.42);
      z-index: 5;
      box-shadow: 0 12px 22px rgba(120, 0, 0, 0.34);
    }}
    .event-source {{
      font-size: 0.68rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      margin-top: 4px;
    }}
    .event-time {{
      font-weight: 700;
      margin-bottom: 4px;
    }}
    .muted {{ color: var(--muted); }}
    .legend {{
      display: flex;
      gap: 12px;
      flex-wrap: wrap;
      margin-top: 12px;
      font-size: 0.9rem;
    }}
    .editor-layout {{
      display: grid;
      grid-template-columns: 1.1fr 0.9fr;
      gap: 18px;
      margin-top: 18px;
    }}
    .form-grid {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
      margin-top: 14px;
    }}
    .form-grid .span-2 {{
      grid-column: span 2;
    }}
    label {{
      display: grid;
      gap: 6px;
      font-size: 0.9rem;
      color: var(--muted);
    }}
    input, select, textarea {{
      width: 100%;
      border-radius: 12px;
      border: 1px solid var(--line);
      background: rgba(18, 20, 21, 0.82);
      color: var(--ink);
      padding: 12px;
      font: inherit;
    }}
    textarea {{
      resize: vertical;
      min-height: 100px;
    }}
    .entry-list {{
      display: grid;
      gap: 10px;
      margin-top: 14px;
    }}
    .entry-card {{
      border: 1px solid var(--line);
      border-radius: 14px;
      background: var(--panel);
      padding: 12px;
    }}
    .entry-actions {{
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      margin-top: 10px;
    }}
    .secondary {{
      background: rgba(138, 164, 184, 0.8);
      color: #08100a;
    }}
    .danger {{
      background: var(--safety);
      color: #fff4ec;
    }}
    @media (max-width: 900px) {{
      .toolbar {{
        align-items: flex-start;
      }}
      .editor-layout {{
        grid-template-columns: 1fr;
      }}
    }}
  </style>
</head>
<body>
  <main>
    <section class="card">
      <h1>Calendar</h1>
      <div class="muted">View the active schedule as a weekly planner with full 24-hour day lanes.</div>
      <div class="nav">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Calendar</a>
        <a class="nav-link" href="/map">Missions</a>
        <a class="nav-link" href="/record-map">Record Map</a>
        <a class="nav-link" href="/developer">Developer</a>
      </div>
    </section>
    <section class="toolbar">
      <div class="toolbar-group">
        <button id="prev-week">Previous Week</button>
        <button id="next-week">Next Week</button>
      </div>
      <div class="toolbar-group">
        <div id="week-number" style="font-size: 0.95rem; font-weight: 700; color: var(--accent);">Week --</div>
        <div id="week-label" style="font-size: 1.2rem; font-weight: 700;">Loading...</div>
        <div id="calendar-timezone" class="muted">Robot timezone: --</div>
        <div class="calendar-clock">
          <div class="muted">Robot local time:</div>
          <div id="calendar-robot-clock" class="calendar-clock-value">--:--:--</div>
        </div>
      </div>
    </section>
    <section class="editor-layout">
      <section class="card">
        <h2>Planned Entry Editor</h2>
        <div class="muted">Create, edit, and delete planned calendar entries in the robot's local timezone.</div>
        <form id="schedule-form" class="form-grid">
          <input id="entry-uid" type="hidden">
          <label>
            Summary
            <input id="entry-summary" type="text" placeholder="RBT-01 WORK window">
          </label>
          <label>
            Schedule Type
            <select id="entry-schedule-type">
              <option value="WORK">WORK</option>
              <option value="NO_WORK">NO_WORK</option>
              <option value="SAFETY">SAFETY</option>
            </select>
          </label>
          <label>
            Mission ID
            <input id="entry-mission-id" type="text" placeholder="Optional mission id">
          </label>
          <label>
            Robot ID
            <input id="entry-robot-id" type="text" placeholder="Optional robot id">
          </label>
          <label>
            Start
            <input id="entry-start-local" type="datetime-local">
          </label>
          <label>
            End
            <input id="entry-end-local" type="datetime-local">
          </label>
          <label class="span-2">
            Recurrence
            <select id="entry-recurrence-type">
              <option value="none">One-off</option>
              <option value="daily">Daily</option>
              <option value="monthly_nth_weekday">Monthly on this weekday occurrence</option>
            </select>
          </label>
          <label class="span-2">
            Description
            <textarea id="entry-description" placeholder="Optional notes for operators"></textarea>
          </label>
          <div class="span-2 entry-actions">
            <button id="save-entry" type="submit">Save Entry</button>
            <button id="reset-entry" class="secondary" type="button">New Entry</button>
          </div>
        </form>
      </section>
      <section class="card">
        <h2>Planned Entries</h2>
        <div id="planned-entry-list" class="entry-list"></div>
      </section>
    </section>
    <section class="card">
      <div id="schedule-path" class="muted" style="margin-bottom: 12px;">Schedule: -</div>
      <div class="week-shell">
        <div id="calendar-grid" class="week-grid"></div>
      </div>
      <div class="legend">
        <span><strong style="color: var(--work);">WORK</strong> mission windows</span>
        <span><strong style="color: var(--nowork);">NO_WORK</strong> blackout windows</span>
        <span><strong style="color: var(--safety);">SAFETY</strong> logged safety events</span>
        <span>Planned blocks are blue-gray, dashed, and semi-transparent in the background. Actual blocks are solid in the foreground.</span>
      </div>
    </section>
  </main>
  <script>
    let activeWeek = '';
    let activeScheduleData = null;
    const weekdayNames = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'];
    const hourHeight = 64;

    function shiftWeek(week, delta) {{
      const [yearPart, weekPart] = week.split('-W');
      const year = Number(yearPart);
      const weekNumber = Number(weekPart);
      const monday = isoWeekStart(year, weekNumber);
      monday.setDate(monday.getDate() + (delta * 7));
      return toIsoWeekString(monday);
    }}

    function isoWeekStart(year, weekNumber) {{
      const januaryFourth = new Date(Date.UTC(year, 0, 4));
      const weekday = januaryFourth.getUTCDay() || 7;
      const monday = new Date(januaryFourth);
      monday.setUTCDate(januaryFourth.getUTCDate() - weekday + 1 + ((weekNumber - 1) * 7));
      return monday;
    }}

    function toIsoWeekString(date) {{
      const utcDate = new Date(Date.UTC(date.getFullYear(), date.getMonth(), date.getDate()));
      const weekday = utcDate.getUTCDay() || 7;
      utcDate.setUTCDate(utcDate.getUTCDate() + 4 - weekday);
      const yearStart = new Date(Date.UTC(utcDate.getUTCFullYear(), 0, 1));
      const weekNumber = Math.ceil((((utcDate - yearStart) / 86400000) + 1) / 7);
      return `${{utcDate.getUTCFullYear()}}-W${{String(weekNumber).padStart(2, '0')}}`;
    }}

    function parseLocalDate(dateText) {{
      const [year, month, day] = dateText.split('-').map(Number);
      return new Date(year, month - 1, day, 0, 0, 0, 0);
    }}

    function parseLocalDateTime(dateText) {{
      const [datePart, timePart] = dateText.split('T');
      const [year, month, day] = datePart.split('-').map(Number);
      const [hour, minute, second = '0'] = timePart.split(':');
      return new Date(year, month - 1, day, Number(hour), Number(minute), Number(second), 0);
    }}

    function startOfDay(date) {{
      return new Date(date.getFullYear(), date.getMonth(), date.getDate(), 0, 0, 0, 0);
    }}

    function toDateInputValue(date) {{
      const year = date.getFullYear();
      const month = String(date.getMonth() + 1).padStart(2, '0');
      const day = String(date.getDate()).padStart(2, '0');
      const hour = String(date.getHours()).padStart(2, '0');
      const minute = String(date.getMinutes()).padStart(2, '0');
      return `${{year}}-${{month}}-${{day}}T${{hour}}:${{minute}}`;
    }}

    function formatLocalTime(date) {{
      return `${{String(date.getHours()).padStart(2, '0')}}:${{String(date.getMinutes()).padStart(2, '0')}}`;
    }}

    function clamp(value, min, max) {{
      return Math.max(min, Math.min(max, value));
    }}

    function resetEntryForm() {{
      document.getElementById('entry-uid').value = '';
      document.getElementById('entry-summary').value = '';
      document.getElementById('entry-schedule-type').value = 'WORK';
      document.getElementById('entry-mission-id').value = '';
      document.getElementById('entry-robot-id').value = '';
      document.getElementById('entry-description').value = '';
      document.getElementById('entry-recurrence-type').value = 'none';
      const baseDate = activeScheduleData?.week_start ? parseLocalDate(activeScheduleData.week_start) : new Date();
      const start = new Date(baseDate.getFullYear(), baseDate.getMonth(), baseDate.getDate(), 8, 0, 0, 0);
      const end = new Date(baseDate.getFullYear(), baseDate.getMonth(), baseDate.getDate(), 10, 0, 0, 0);
      document.getElementById('entry-start-local').value = toDateInputValue(start);
      document.getElementById('entry-end-local').value = toDateInputValue(end);
    }}

    function populateEntryForm(entry) {{
      document.getElementById('entry-uid').value = entry.uid || '';
      document.getElementById('entry-summary').value = entry.summary || '';
      document.getElementById('entry-schedule-type').value = entry.schedule_type || 'WORK';
      document.getElementById('entry-mission-id').value = entry.mission_id || '';
      document.getElementById('entry-robot-id').value = entry.robot_id || '';
      document.getElementById('entry-start-local').value = entry.start_local || '';
      document.getElementById('entry-end-local').value = entry.end_local || '';
      document.getElementById('entry-recurrence-type').value = entry.recurrence_type || 'none';
      document.getElementById('entry-description').value = entry.description || '';
    }}

    async function postJson(path, body) {{
      const response = await fetch(path, {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/json' }},
        body: JSON.stringify(body || {{}})
      }});
      return response.json();
    }}

    function renderPlannedEntries(entries) {{
      const list = document.getElementById('planned-entry-list');
      list.innerHTML = '';
      if (!entries || entries.length === 0) {{
        list.innerHTML = '<div class="muted">No planned entries yet.</div>';
        return;
      }}

      for (const entry of entries) {{
        const card = document.createElement('div');
        card.className = 'entry-card';
        card.innerHTML = `
          <div><strong>${{entry.summary || entry.schedule_type || 'Planned entry'}}</strong></div>
          <div class="muted">${{entry.start_local || '-'}} to ${{entry.end_local || '-'}} | ${{entry.recurrence_label || 'One-off'}}</div>
          <div class="muted">${{entry.schedule_type || 'WORK'}}${{entry.mission_id ? ` | Mission: ${{entry.mission_id}}` : ''}}${{entry.robot_id ? ` | Robot: ${{entry.robot_id}}` : ''}}</div>
          <div class="entry-actions">
            <button class="secondary" type="button">Edit</button>
            <button class="danger" type="button">Delete</button>
          </div>
        `;
        const [editButton, deleteButton] = card.querySelectorAll('button');
        editButton.addEventListener('click', () => populateEntryForm(entry));
        deleteButton.addEventListener('click', async () => {{
          const result = await postJson('/api/schedule/entry/delete', {{ uid: entry.uid }});
          if (!result.success) {{
            window.alert(result.message || 'Failed to delete schedule entry');
            return;
          }}
          await loadCalendar(activeWeek);
        }});
        list.appendChild(card);
      }}
    }}

    async function loadCalendar(week) {{
      const response = await fetch(`/api/schedule?week=${{encodeURIComponent(week)}}`, {{ cache: 'no-store' }});
      const data = await response.json();
      activeScheduleData = data;
      activeWeek = data.week;
      document.getElementById('week-label').textContent = data.week_label || data.week;
      document.getElementById('week-number').textContent = `CW ${{data.week_number ?? '--'}}`;
      document.getElementById('calendar-timezone').textContent = `Robot timezone: ${{data.robot_timezone || '--'}}`;
      const robotClock = data.robot_clock || {{}};
      document.getElementById('calendar-robot-clock').textContent = robotClock.local_time || '--:--:--';
      document.getElementById('schedule-path').textContent =
        `Planned: ${{data.planned_schedule_path || '-'}} | Actual: ${{data.actual_schedule_path || '-'}}`;
      renderPlannedEntries(data.planned_entries || []);

      const grid = document.getElementById('calendar-grid');
      grid.innerHTML = '';
      const weekStart = parseLocalDate(data.week_start);

      const corner = document.createElement('div');
      corner.className = 'corner';
      corner.textContent = '24H';
      grid.appendChild(corner);

      const timeRail = document.createElement('div');
      timeRail.className = 'time-rail';
      for (let hour = 0; hour < 24; hour += 1) {{
        const slot = document.createElement('div');
        slot.className = 'time-slot';
        slot.textContent = `${{String(hour).padStart(2, '0')}}:00`;
        timeRail.appendChild(slot);
      }}

      const dayColumns = [];
      for (let index = 0; index < 7; index += 1) {{
        const current = new Date(weekStart);
        current.setDate(weekStart.getDate() + index);

        const head = document.createElement('div');
        head.className = 'day-head';
        head.innerHTML = `
          <div class="day-name">${{weekdayNames[index]}}</div>
          <div class="day-number">${{String(current.getDate()).padStart(2, '0')}}.${{String(current.getMonth() + 1).padStart(2, '0')}}</div>
        `;
        grid.appendChild(head);
      }}

      grid.appendChild(timeRail);
      for (let dayIndex = 0; dayIndex < 7; dayIndex += 1) {{
        const dayColumn = document.createElement('div');
        dayColumn.className = 'day-column';
        dayColumn.dataset.dayIndex = String(dayIndex);
        dayColumns.push(dayColumn);
        grid.appendChild(dayColumn);
      }}

      const weekEnd = new Date(weekStart);
      weekEnd.setDate(weekStart.getDate() + 7);
      const weekStartMs = weekStart.getTime();

      function renderEvents(events, sourceLabel) {{
        for (const event of events || []) {{
          const start = parseLocalDateTime(event.start_local);
          const end = parseLocalDateTime(event.end_local);
          const visibleStart = start > weekStart ? start : weekStart;
          const visibleEnd = end < weekEnd ? end : weekEnd;
          if (visibleEnd <= visibleStart) {{
            continue;
          }}

          let segmentDay = startOfDay(visibleStart);
          while (segmentDay < visibleEnd) {{
            const nextDay = new Date(segmentDay);
            nextDay.setDate(segmentDay.getDate() + 1);
            const segmentStart = visibleStart > segmentDay ? visibleStart : segmentDay;
            const segmentEnd = visibleEnd < nextDay ? visibleEnd : nextDay;
            const dayIndex = Math.floor((segmentDay.getTime() - weekStartMs) / 86400000);
            if (dayIndex >= 0 && dayIndex <= 6 && segmentEnd > segmentStart) {{
              const column = dayColumns[dayIndex];
              const startMinutes = clamp((segmentStart.getHours() * 60) + segmentStart.getMinutes(), 0, 1440);
              const endMinutes = clamp((segmentEnd.getHours() * 60) + segmentEnd.getMinutes(), 0, 1440);
              const durationMinutes = Math.max(
                30,
                (segmentEnd >= nextDay && endMinutes === 0 ? 1440 : endMinutes) - startMinutes
              );

              const chip = document.createElement('div');
              chip.className = `event-chip ${{event.schedule_type || 'WORK'}} ${{sourceLabel}}`;
              chip.style.top = `${{(startMinutes / 60) * hourHeight + 6}}px`;
              chip.style.height = `${{Math.max(28, (durationMinutes / 60) * hourHeight - 8)}}px`;
              chip.title = event.description || event.summary || '';
              chip.innerHTML = `
                <div class="event-time">${{formatLocalTime(segmentStart)}} - ${{segmentEnd >= nextDay ? '24:00' : formatLocalTime(segmentEnd)}}</div>
                <div><strong>${{event.summary || event.schedule_type || 'Event'}}</strong></div>
                <div>${{event.mission_id || event.robot_id || ''}}</div>
                <div class="event-source">${{sourceLabel}}</div>
              `;
              if (sourceLabel === 'planned') {{
                chip.style.cursor = 'pointer';
                chip.addEventListener('click', () => {{
                  const entry = (data.planned_entries || []).find((plannedEntry) => plannedEntry.uid === event.uid);
                  if (entry) {{
                    populateEntryForm(entry);
                  }}
                }});
              }}
              column.appendChild(chip);
            }}
            segmentDay = nextDay;
          }}
        }}
      }}

      renderEvents(data.planned_events || [], 'planned');
      renderEvents(data.actual_events || [], 'actual');
    }}

    document.getElementById('prev-week').addEventListener('click', async () => {{
      await loadCalendar(shiftWeek(activeWeek, -1));
    }});

    document.getElementById('next-week').addEventListener('click', async () => {{
      await loadCalendar(shiftWeek(activeWeek, 1));
    }});

    document.getElementById('schedule-form').addEventListener('submit', async (event) => {{
      event.preventDefault();
      const payload = {{
        uid: document.getElementById('entry-uid').value,
        summary: document.getElementById('entry-summary').value,
        schedule_type: document.getElementById('entry-schedule-type').value,
        mission_id: document.getElementById('entry-mission-id').value,
        robot_id: document.getElementById('entry-robot-id').value,
        start_local: document.getElementById('entry-start-local').value,
        end_local: document.getElementById('entry-end-local').value,
        recurrence_type: document.getElementById('entry-recurrence-type').value,
        description: document.getElementById('entry-description').value,
      }};
      const result = await postJson('/api/schedule/entry', payload);
      if (!result.success) {{
        window.alert(result.message || 'Failed to save schedule entry');
        return;
      }}
      resetEntryForm();
      await loadCalendar(activeWeek || toIsoWeekString(new Date()));
    }});

    document.getElementById('reset-entry').addEventListener('click', () => {{
      resetEntryForm();
    }});

    loadCalendar(toIsoWeekString(new Date())).then(() => {{
      resetEntryForm();
    }});
  </script>
</body>
</html>
"""

    def render_record_map_html(self) -> str:
        title = escape(self._site_title)
        return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{title} - Record Map</title>
  <link
    rel="stylesheet"
    href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
    integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY="
    crossorigin=""
  >
  <style>
    :root {{
      --bg: #1b1e20;
      --bg-alt: #2b2f31;
      --card: rgba(54, 58, 60, 0.94);
      --card-strong: rgba(42, 46, 48, 0.98);
      --panel: rgba(88, 92, 94, 0.48);
      --ink: #f5f1df;
      --muted: #c4bb98;
      --accent: #fdca0f;
      --accent-strong: #ffe06b;
      --danger: #ff7b5c;
      --line: rgba(253, 202, 15, 0.22);
      --gold: #fdca0f;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      color: var(--ink);
      font-family: "Avenir Next", "Segoe UI", sans-serif;
      background:
        radial-gradient(circle at top left, rgba(253, 202, 15, 0.18), transparent 26%),
        radial-gradient(circle at top right, rgba(255, 255, 255, 0.06), transparent 20%),
        linear-gradient(180deg, var(--bg) 0%, var(--bg-alt) 100%);
    }}
    main {{
      max-width: 1280px;
      margin: 0 auto;
      padding: 24px;
    }}
    .card {{
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: 20px;
      padding: 18px;
      box-shadow: 0 14px 36px rgba(0, 0, 0, 0.24);
      backdrop-filter: blur(4px);
    }}
    .hero {{
      display: grid;
      gap: 12px;
      background:
        linear-gradient(135deg, rgba(253, 202, 15, 0.16), rgba(42, 46, 48, 0.18) 42%),
        var(--card-strong);
    }}
    h1, h2 {{
      text-transform: uppercase;
      letter-spacing: 0.09em;
      font-family: "Avenir Next Condensed", "Franklin Gothic Medium", "Arial Narrow", sans-serif;
    }}
    h1 {{
      color: var(--accent);
    }}
    .nav {{
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
    }}
    .nav-link {{
      display: inline-block;
      text-decoration: none;
      color: var(--ink);
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 8px 14px;
      background: rgba(52, 53, 53, 0.72);
      font-size: 0.92rem;
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }}
    .layout {{
      display: grid;
      grid-template-columns: 1.6fr 1fr;
      gap: 18px;
      margin-top: 18px;
    }}
    .map-shell {{
      min-height: 620px;
      overflow: hidden;
      padding: 0;
    }}
    #record-map {{
      width: 100%;
      min-height: 620px;
      border-radius: 20px;
    }}
    .stack {{
      display: grid;
      gap: 16px;
    }}
    .toolbar {{
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      align-items: center;
    }}
    .status-chip {{
      display: inline-flex;
      align-items: center;
      border-radius: 999px;
      padding: 8px 12px;
      background: rgba(253, 202, 15, 0.12);
      color: var(--accent);
      font-weight: 600;
      font-size: 0.92rem;
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }}
    .status-chip.idle {{
      background: rgba(180, 83, 9, 0.12);
      color: var(--gold);
    }}
    .muted {{ color: var(--muted); }}
    .meta-grid {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
    }}
    .meta {{
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 12px;
      background: var(--panel);
    }}
    label {{
      display: block;
      font-size: 0.9rem;
      color: var(--muted);
      margin-bottom: 6px;
    }}
    input[type="text"], select {{
      width: 100%;
      padding: 12px;
      border-radius: 12px;
      border: 1px solid var(--line);
      background: rgba(18, 20, 21, 0.82);
      color: var(--ink);
    }}
    .pattern-list {{
      display: grid;
      gap: 10px;
      margin-top: 8px;
    }}
    .pattern-option {{
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 12px;
      background: var(--panel);
    }}
    .pattern-option input {{
      margin-right: 8px;
    }}
    button {{
      border: 0;
      border-radius: 999px;
      padding: 11px 16px;
      font-size: 0.95rem;
      cursor: pointer;
      color: #08100a;
      background: var(--accent);
      font-weight: 700;
      letter-spacing: 0.04em;
      text-transform: uppercase;
    }}
    button:hover {{ background: var(--accent-strong); }}
    button.stop {{ background: var(--danger); }}
    button.secondary {{
      color: var(--ink);
      background: rgba(96, 100, 102, 0.72);
      border: 1px solid var(--line);
    }}
    .banner {{
      display: none;
      border-radius: 14px;
      padding: 12px 14px;
      font-weight: 600;
    }}
    .banner.show {{ display: block; }}
    .banner.ok {{ background: rgba(15, 118, 110, 0.12); color: var(--accent-strong); }}
    .banner.error {{ background: rgba(185, 28, 28, 0.12); color: var(--danger); }}
    .countdown {{
      font-size: 0.88rem;
      color: var(--gold);
      min-height: 1.2rem;
    }}
    @media (max-width: 980px) {{
      .layout {{ grid-template-columns: 1fr; }}
      .meta-grid {{ grid-template-columns: 1fr; }}
      #record-map {{ min-height: 440px; }}
    }}
  </style>
</head>
<body>
  <main>
    <section class="card hero">
      <h1>Record Map And Create Missions</h1>
      <div class="muted">Drive the robot around the working-area perimeter, let RecordMap update the latest recorded map, then create one or more named autonomous missions from that map using a sweep pattern.</div>
      <div id="banner" class="banner"></div>
      <div class="nav">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Calendar</a>
        <a class="nav-link" href="/map">Missions</a>
        <a class="nav-link" href="/record-map">Record Map</a>
        <a class="nav-link" href="/developer">Developer</a>
      </div>
    </section>

    <section class="layout">
      <section class="card map-shell">
        <div id="record-map"></div>
      </section>
      <section class="stack">
        <section class="card">
          <div class="toolbar">
            <div id="recording-chip" class="status-chip idle">RecordMap idle</div>
            <button id="record-button">Record</button>
            <button id="stop-button" class="stop">Stop</button>
          </div>
          <div class="muted" style="margin-top: 12px;">The latest recorded map is overwritten whenever a new RecordMap session is completed.</div>
        </section>

        <section class="card">
          <h2>Latest Recorded Map</h2>
          <div class="meta-grid" style="margin-top: 12px;">
            <div class="meta">
              <strong>Recorded Run</strong>
              <div id="latest-run" class="muted" style="margin-top: 6px;">No recording captured yet.</div>
            </div>
            <div class="meta">
              <strong>Obstacle Count</strong>
              <div id="latest-obstacles" class="muted" style="margin-top: 6px;">-</div>
            </div>
          </div>
          <div id="latest-map-message" class="muted" style="margin-top: 12px;">Complete a RecordMap session to unlock mission creation.</div>
        </section>

        <section class="card">
          <h2>Create Autonomous Mission</h2>
          <div style="margin-top: 12px;">
            <label for="mission-name">Mission name</label>
            <input id="mission-name" type="text" placeholder="yard_east_zigzag">
          </div>
          <div style="margin-top: 14px;">
            <strong>Pattern</strong>
            <div id="pattern-countdown" class="countdown"></div>
            <div class="pattern-list">
              <label class="pattern-option"><input type="radio" name="pattern" value="zigzag"> Zigzag coverage</label>
              <label class="pattern-option"><input type="radio" name="pattern" value="random"> Random roaming coverage</label>
              <label class="pattern-option"><input type="radio" name="pattern" value="spiral"> Spiral inward coverage</label>
            </div>
          </div>
          <div class="toolbar" style="margin-top: 16px;">
            <button id="save-button">Save Mission</button>
            <button id="refresh-button" class="secondary">Refresh</button>
          </div>
        </section>
      </section>
    </section>
  </main>

  <script
    src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
    integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo="
    crossorigin=""
  ></script>
  <script>
    const banner = document.getElementById('banner');
    const chip = document.getElementById('recording-chip');
    const latestRun = document.getElementById('latest-run');
    const latestObstacles = document.getElementById('latest-obstacles');
    const latestMapMessage = document.getElementById('latest-map-message');
    const patternCountdown = document.getElementById('pattern-countdown');
    const patternInputs = [...document.querySelectorAll('input[name="pattern"]')];
    const missionNameInput = document.getElementById('mission-name');
    let patternTouched = false;
    let countdownTimer = null;
    let countdownSeconds = 20;
    let lastLatestRunId = '';
    let activePolyline = null;
    let latestPolyline = null;
    let perimeterPolyline = null;
    let currentMarker = null;

    const map = L.map('record-map', {{ zoomControl: true }}).setView([55.6761, 12.5683], 18);
    L.tileLayer(
      'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}',
      {{ maxZoom: 20, attribution: '&copy; Esri' }}
    ).addTo(map);

    function setBanner(kind, message) {{
      banner.className = `banner show ${{kind}}`;
      banner.textContent = message;
      window.setTimeout(() => {{
        banner.className = 'banner';
        banner.textContent = '';
      }}, 5000);
    }}

    function selectedPattern() {{
      const selected = patternInputs.find((input) => input.checked);
      return selected ? selected.value : '';
    }}

    function ensureDefaultPatternSelection() {{
      if (!selectedPattern()) {{
        const zigzag = patternInputs.find((input) => input.value === 'zigzag');
        if (zigzag) {{
          zigzag.checked = true;
        }}
      }}
    }}

    function startPatternCountdown() {{
      window.clearInterval(countdownTimer);
      countdownSeconds = 20;
      patternTouched = false;
      patternCountdown.textContent = 'Zigzag will be selected automatically in 20 seconds if you do not choose a pattern.';
      countdownTimer = window.setInterval(() => {{
        countdownSeconds -= 1;
        if (patternTouched) {{
          window.clearInterval(countdownTimer);
          patternCountdown.textContent = '';
          return;
        }}
        if (countdownSeconds <= 0) {{
          ensureDefaultPatternSelection();
          patternCountdown.textContent = 'No pattern was chosen in time, so zigzag is selected.';
          window.clearInterval(countdownTimer);
          return;
        }}
        patternCountdown.textContent = `Zigzag will be selected automatically in ${{countdownSeconds}} seconds if you do not choose a pattern.`;
      }}, 1000);
    }}

    for (const input of patternInputs) {{
      input.addEventListener('change', () => {{
        patternTouched = true;
        patternCountdown.textContent = '';
      }});
    }}

    function lineStringLatLngs(geojson) {{
      const latlngs = [];
      for (const feature of geojson?.features || []) {{
        if (feature?.geometry?.type !== 'LineString') {{
          continue;
        }}
        for (const coordinate of feature.geometry.coordinates || []) {{
          if (Array.isArray(coordinate) && coordinate.length >= 2) {{
            latlngs.push([Number(coordinate[1]), Number(coordinate[0])]);
          }}
        }}
      }}
      return latlngs;
    }}

    function updateMap(data) {{
      if (activePolyline) {{
        map.removeLayer(activePolyline);
        activePolyline = null;
      }}
      if (latestPolyline) {{
        map.removeLayer(latestPolyline);
        latestPolyline = null;
      }}
      if (perimeterPolyline) {{
        map.removeLayer(perimeterPolyline);
        perimeterPolyline = null;
      }}
      if (currentMarker) {{
        map.removeLayer(currentMarker);
        currentMarker = null;
      }}

      const bounds = [];
      const activeLatLngs = lineStringLatLngs(data.active_navsat_geojson);
      if (activeLatLngs.length > 1) {{
        activePolyline = L.polyline(activeLatLngs, {{ color: '#dc2626', weight: 4 }}).addTo(map);
        bounds.push(...activeLatLngs);
      }}

      const latestLatLngs = lineStringLatLngs(data.latest_navsat_geojson);
      if (latestLatLngs.length > 1) {{
        latestPolyline = L.polyline(latestLatLngs, {{ color: '#0f766e', weight: 4 }}).addTo(map);
        bounds.push(...latestLatLngs);
      }}

      const perimeterLatLngs = lineStringLatLngs(data.latest_route_geojson);
      if (perimeterLatLngs.length > 1) {{
        perimeterPolyline = L.polyline(perimeterLatLngs, {{ color: '#f59e0b', weight: 3, dashArray: '8 8' }}).addTo(map);
      }}

      const position = data.current_position;
      if (position && position.latitude !== undefined && position.longitude !== undefined) {{
        currentMarker = L.circleMarker(
          [Number(position.latitude), Number(position.longitude)],
          {{ radius: 6, color: '#ffffff', weight: 2, fillColor: '#1d4ed8', fillOpacity: 1 }}
        ).addTo(map);
        bounds.push([Number(position.latitude), Number(position.longitude)]);
      }}

      if (bounds.length > 0) {{
        map.fitBounds(bounds, {{ padding: [30, 30], maxZoom: 19 }});
      }}
    }}

    async function loadRecordMapSnapshot() {{
      const response = await fetch('/api/record-map', {{ cache: 'no-store' }});
      const data = await response.json();
      chip.textContent = data.active_recording ? 'RecordMap recording' : 'RecordMap idle';
      chip.className = data.active_recording ? 'status-chip' : 'status-chip idle';

      const latest = data.latest_recorded_map;
      if (latest && !latest.error) {{
        latestRun.textContent = latest.run_started_at || 'Latest recording available';
        latestObstacles.textContent = String(latest.recorded_obstacle_count ?? '-');
        latestMapMessage.textContent = 'This latest recorded map can be reused to make several missions with different patterns.';
        const latestRunId = String(latest.run_started_at || '');
        if (latestRunId && latestRunId !== lastLatestRunId) {{
          startPatternCountdown();
          lastLatestRunId = latestRunId;
        }}
      }} else {{
        latestRun.textContent = 'No recording captured yet.';
        latestObstacles.textContent = '-';
        latestMapMessage.textContent = 'Complete a RecordMap session to unlock mission creation.';
        window.clearInterval(countdownTimer);
        patternCountdown.textContent = '';
        lastLatestRunId = '';
      }}

      updateMap(data);
      return data;
    }}

    async function postJson(path, body) {{
      const response = await fetch(path, {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/json' }},
        body: JSON.stringify(body || {{}})
      }});
      return response.json();
    }}

    document.getElementById('record-button').addEventListener('click', async () => {{
      const data = await postJson('/api/record-map/start', {{}});
      setBanner(data.success ? 'ok' : 'error', data.message || 'RecordMap request completed');
      await loadRecordMapSnapshot();
    }});

    document.getElementById('stop-button').addEventListener('click', async () => {{
      const data = await postJson('/api/record-map/stop', {{}});
      setBanner(data.success ? 'ok' : 'error', data.message || 'RecordMap stop request completed');
      await loadRecordMapSnapshot();
    }});

    document.getElementById('save-button').addEventListener('click', async () => {{
      ensureDefaultPatternSelection();
      const missionName = missionNameInput.value.trim();
      if (!missionName) {{
        setBanner('error', 'Enter a mission name before saving.');
        return;
      }}
      const data = await postJson('/api/record-map/save-mission', {{
        mission_name: missionName,
        sweep_pattern: selectedPattern(),
        overwrite_existing: false
      }});
      setBanner(data.success ? 'ok' : 'error', data.message || 'Save mission request completed');
      if (data.success) {{
        missionNameInput.value = '';
      }}
    }});

    document.getElementById('refresh-button').addEventListener('click', async () => {{
      await loadRecordMapSnapshot();
    }});

    ensureDefaultPatternSelection();
    loadRecordMapSnapshot().catch((error) => {{
      setBanner('error', error.message || 'Failed to load record map page state');
    }});
    window.setInterval(() => {{
      loadRecordMapSnapshot().catch(() => null);
    }}, 4000);
  </script>
</body>
</html>
"""

    def render_map_html(self) -> str:
        title = escape(self._site_title)
        return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{title} - Missions</title>
  <style>
    :root {{
      --bg: #1b1e20;
      --bg-alt: #2b2f31;
      --card: rgba(54, 58, 60, 0.94);
      --panel: rgba(88, 92, 94, 0.48);
      --ink: #f5f1df;
      --muted: #c4bb98;
      --accent: #fdca0f;
      --accent-strong: #ffe06b;
      --line: rgba(253, 202, 15, 0.22);
      --danger: #ff7b5c;
    }}
    body {{
      margin: 0;
      font-family: "Avenir Next", "Segoe UI", sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(253, 202, 15, 0.18), transparent 26%),
        linear-gradient(180deg, var(--bg) 0%, var(--bg-alt) 100%);
    }}
    main {{
      max-width: 1200px;
      margin: 0 auto;
      padding: 24px;
    }}
    .card {{
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 14px 34px rgba(0, 0, 0, 0.22);
      backdrop-filter: blur(4px);
    }}
    h1, h2 {{
      text-transform: uppercase;
      letter-spacing: 0.09em;
      font-family: "Avenir Next Condensed", "Franklin Gothic Medium", "Arial Narrow", sans-serif;
    }}
    h1 {{
      color: var(--accent);
    }}
    .nav {{
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      margin-top: 14px;
    }}
    .nav-link {{
      display: inline-block;
      text-decoration: none;
      color: var(--ink);
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 8px 14px;
      background: rgba(52, 53, 53, 0.72);
      font-size: 0.92rem;
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }}
    .map-layout {{
      display: grid;
      grid-template-columns: 1.6fr 1fr;
      gap: 16px;
      margin-top: 18px;
    }}
    .map-frame {{
      width: 100%;
      min-height: 520px;
      border: 1px solid var(--line);
      border-radius: 16px;
      background: var(--panel);
      display: flex;
      align-items: center;
      justify-content: center;
      overflow: hidden;
    }}
    .legend-list {{
      display: grid;
      gap: 10px;
    }}
    .legend-item {{
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 12px;
      background: var(--panel);
    }}
    .mission-list {{
      display: grid;
      gap: 12px;
      margin-top: 14px;
    }}
    .mission-group {{
      display: grid;
      gap: 12px;
    }}
    .mission-group-title {{
      margin: 4px 0 0;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--accent);
      font-family: "Avenir Next Condensed", "Franklin Gothic Medium", "Arial Narrow", sans-serif;
    }}
    .mission {{
      border: 1px solid var(--line);
      border-radius: 16px;
      padding: 14px;
      background: var(--panel);
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 14px;
      flex-wrap: wrap;
    }}
    .mission-actions {{
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
    }}
    .legend-actions {{
      margin-top: 10px;
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
    }}
    button {{
      appearance: none;
      border: none;
      border-radius: 12px;
      padding: 12px 16px;
      font-size: 0.95rem;
      font-weight: 600;
      cursor: pointer;
      color: #08100a;
      background: var(--accent);
      transition: transform 0.12s ease, box-shadow 0.12s ease;
      text-transform: uppercase;
      letter-spacing: 0.04em;
    }}
    button:hover {{
      background: var(--accent-strong);
      transform: translateY(-1px);
      box-shadow: 0 12px 18px rgba(15, 118, 110, 0.16);
    }}
    .banner {{
      display: none;
      border-radius: 14px;
      padding: 12px 14px;
      font-weight: 600;
      margin-top: 14px;
    }}
    .banner.show {{ display: block; }}
    .banner.ok {{ background: rgba(15, 118, 110, 0.12); color: var(--accent-strong); }}
    .banner.error {{ background: rgba(185, 28, 28, 0.12); color: var(--danger); }}
    .muted {{ color: var(--muted); }}
    @media (max-width: 900px) {{
      .map-layout {{ grid-template-columns: 1fr; }}
    }}
  </style>
</head>
<body>
  <main>
    <section class="card">
      <h1>Missions</h1>
      <div class="muted">Preview built or decoded mission routes from the synced mission database, and upload VDA5050 missions.</div>
      <div id="banner" class="banner"></div>
      <div class="nav">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Calendar</a>
        <a class="nav-link" href="/map">Missions</a>
        <a class="nav-link" href="/record-map">Record Map</a>
        <a class="nav-link" href="/developer">Developer</a>
      </div>
    </section>
    <section class="map-layout">
      <section class="card">
        <div id="map-frame" class="map-frame">Loading mission geometry...</div>
      </section>
      <section class="card">
        <h2>Legend</h2>
        <div id="legend-list" class="legend-list"></div>
      </section>
    </section>
    <section class="card" style="margin-top: 18px;">
      <h2>Executable Missions</h2>
      <div class="muted">Launch manual or autonomous missions from the synced mission database.</div>
      <div id="mission-list" class="mission-list"></div>
    </section>
    <section class="card" style="margin-top: 18px;">
      <h2>Upload VDA5050 Mission</h2>
      <div class="muted">Paste a VDA5050 mission JSON document. You can optionally provide a mission id; otherwise `orderId` is used.</div>
      <div style="display: grid; gap: 12px; margin-top: 14px;">
        <input id="upload-file" type="file" accept=".json,application/json" style="padding: 12px; border-radius: 12px; border: 1px solid var(--line); background: rgba(18, 20, 21, 0.82); color: var(--ink);">
        <input id="upload-mission-id" type="text" placeholder="Optional mission id" style="padding: 12px; border-radius: 12px; border: 1px solid var(--line); background: rgba(18, 20, 21, 0.82); color: var(--ink);">
        <label class="muted"><input id="upload-overwrite" type="checkbox"> Overwrite existing mission with same id</label>
        <textarea id="upload-json" rows="14" placeholder='{{"orderId":"field_block_12","nodes":[...],"edges":[...]}}' style="width: 100%; padding: 12px; border-radius: 12px; border: 1px solid var(--line); background: rgba(18, 20, 21, 0.82); color: var(--ink); font-family: monospace;"></textarea>
        <div>
          <button id="upload-button">Upload Mission</button>
        </div>
      </div>
    </section>
  </main>
  <script>
    const banner = document.getElementById('banner');
    const missionList = document.getElementById('mission-list');
    const palette = ['#fdca0f', '#f5f1df', '#d2a500', '#ff7b5c', '#9aa0a3', '#ffe06b'];

    function setBanner(kind, message) {{
      banner.className = `banner show ${{kind}}`;
      banner.textContent = message;
      setTimeout(() => {{
        banner.className = 'banner';
        banner.textContent = '';
      }}, 5000);
    }}

    function extractLineCoordinates(geojson) {{
      const lines = [];
      for (const feature of geojson.features || []) {{
        const geometry = feature.geometry || {{}};
        if (geometry.type === 'LineString') {{
          lines.push(geometry.coordinates || []);
        }}
      }}
      return lines;
    }}

    function renderMap(missions, activeRoute) {{
      const frame = document.getElementById('map-frame');
      const legend = document.getElementById('legend-list');
      legend.innerHTML = '';

      const layers = [];
      for (const [index, mission] of missions.entries()) {{
        if (!mission.route_geojson) {{
          continue;
        }}
        layers.push({{
          mission_id: mission.mission_id,
          color: palette[index % palette.length],
          lines: extractLineCoordinates(mission.route_geojson),
        }});
      }}
      if (activeRoute) {{
        layers.push({{
          mission_id: 'Active Mission',
          color: '#dc2626',
          lines: extractLineCoordinates(activeRoute),
        }});
      }}

      if (layers.length === 0 && missions.length === 0) {{
        frame.textContent = 'No mission route geometry is available yet.';
        legend.innerHTML = '<div class="muted">Built route geometry will appear here once mission artifacts are available.</div>';
        return;
      }}

      if (layers.length === 0) {{
        frame.textContent = 'No mission route geometry is available yet.';
      }}

      if (layers.length > 0) {{
        const points = [];
        for (const layer of layers) {{
          for (const line of layer.lines) {{
            for (const point of line) {{
              if (Array.isArray(point) && point.length >= 2) {{
                points.push(point);
              }}
            }}
          }}
        }}
        const xs = points.map((point) => Number(point[0]));
        const ys = points.map((point) => Number(point[1]));
        const minX = Math.min(...xs);
        const maxX = Math.max(...xs);
        const minY = Math.min(...ys);
        const maxY = Math.max(...ys);
        const width = Math.max(1, maxX - minX);
        const height = Math.max(1, maxY - minY);
        const padding = 40;
        const viewWidth = 900;
        const viewHeight = 520;

        function project(point) {{
          const x = padding + ((Number(point[0]) - minX) / width) * (viewWidth - padding * 2);
          const y = viewHeight - padding - ((Number(point[1]) - minY) / height) * (viewHeight - padding * 2);
          return `${{x.toFixed(2)}},${{y.toFixed(2)}}`;
        }}

        const polylines = layers.flatMap((layer) =>
          layer.lines.map((line) =>
            `<polyline fill="none" stroke="${{layer.color}}" stroke-width="${{layer.mission_id === 'Active Mission' ? 5 : 3}}" points="${{line.map(project).join(' ')}}" />`
          )
        ).join('');

        frame.innerHTML = `
          <svg viewBox="0 0 ${{viewWidth}} ${{viewHeight}}" width="100%" height="100%" xmlns="http://www.w3.org/2000/svg">
            <rect x="0" y="0" width="${{viewWidth}}" height="${{viewHeight}}" fill="#2c3032" />
            <g opacity="0.15">
              <line x1="40" y1="40" x2="40" y2="${{viewHeight - 40}}" stroke="#eef3eb" />
              <line x1="40" y1="${{viewHeight - 40}}" x2="${{viewWidth - 40}}" y2="${{viewHeight - 40}}" stroke="#eef3eb" />
            </g>
            ${{polylines}}
          </svg>
        `;
      }}

      for (const mission of missions) {{
        const item = document.createElement('div');
        item.className = 'legend-item';
        const layer = layers.find((entry) => entry.mission_id === mission.mission_id);
        const downloadHref = `/api/missions/${{encodeURIComponent(mission.mission_id)}}/download`;
        item.innerHTML = `
          <div><strong>${{mission.mission_id}}</strong></div>
          <div class="muted">
            ${{
              layer
                ? `Color: <span style="color:${{layer.color}};">${{layer.color}}</span>`
                : 'No route geometry available'
            }}
          </div>
          ${{
            downloadHref
              ? `<div class="legend-actions"><a class="nav-link" href="${{downloadHref}}" download>Download JSON</a></div>`
              : ''
          }}
        `;
        legend.appendChild(item);
      }}

      if (activeRoute) {{
        const item = document.createElement('div');
        item.className = 'legend-item';
        item.innerHTML = `
          <div><strong>Active Mission</strong></div>
          <div class="muted">Color: <span style="color:#dc2626;">#dc2626</span></div>
        `;
        legend.appendChild(item);
      }}
    }}

    async function loadMap() {{
      const response = await fetch('/api/map-data', {{ cache: 'no-store' }});
      const data = await response.json();
      renderMap(data.missions || [], data.active_route_geojson || null);
    }}

    async function loadMissions() {{
      const response = await fetch('/api/missions', {{ cache: 'no-store' }});
      const data = await response.json();
      missionList.innerHTML = '';

      const defaultMissionOrder = ['SpotSweep', '3x3Sweep', 'Teleop'];
      const hiddenMissionIds = new Set(['RecordMap']);
      const missions = (data.missions || []).filter((mission) => !hiddenMissionIds.has(mission.mission_id));
      const defaultMissionMap = new Map(missions.map((mission) => [mission.mission_id, mission]));
      const defaultMissions = defaultMissionOrder
        .map((missionId) => defaultMissionMap.get(missionId))
        .filter((mission) => Boolean(mission));
      const savedMissions = missions
        .filter((mission) => !defaultMissionOrder.includes(mission.mission_id))
        .sort((left, right) => left.mission_id.localeCompare(right.mission_id));

      function appendMissionGroup(title, groupMissions) {{
        if (groupMissions.length === 0) {{
          return;
        }}
        const group = document.createElement('section');
        group.className = 'mission-group';
        group.innerHTML = `<h3 class="mission-group-title">${{title}}</h3>`;

        for (const mission of groupMissions) {{
          const item = document.createElement('div');
          item.className = 'mission';
          item.innerHTML = `
            <div>
              <strong>${{mission.mission_id}}</strong><br>
              <span class="muted">${{mission.is_manual ? 'Manual' : 'Autonomous'}} | Type: ${{mission.mission_type || '-'}} | Mode: ${{mission.execution_mode || '-'}} | RUNNING profile: ${{mission.running_profile_id}} | Artifacts: ${{mission.artifacts_ready ? 'ready' : 'pending build'}}</span>
            </div>
            <div class="mission-actions">
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
          }});
          group.appendChild(item);
        }}

        missionList.appendChild(group);
      }}

      appendMissionGroup('Default Missions', defaultMissions);
      appendMissionGroup('Saved Missions', savedMissions);

      if (missionList.children.length === 0) {{
        missionList.innerHTML = '<div class="muted">No executable missions are available right now.</div>';
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
      await loadMap();
    }});

    Promise.all([loadMap(), loadMissions()]).catch((error) => {{
      setBanner('error', error.message || 'Failed to load mission page state');
    }});
  </script>
</body>
</html>
"""

    def render_developer_html(self) -> str:
        title = escape(self._site_title)
        return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{title} - Developer</title>
  <style>
    :root {{
      --bg: #1b1e20;
      --bg-alt: #2b2f31;
      --card: rgba(54, 58, 60, 0.94);
      --panel: rgba(88, 92, 94, 0.48);
      --ink: #f5f1df;
      --muted: #c4bb98;
      --line: rgba(253, 202, 15, 0.22);
    }}
    body {{
      margin: 0;
      font-family: "Avenir Next", "Segoe UI", sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(253, 202, 15, 0.18), transparent 26%),
        linear-gradient(180deg, var(--bg) 0%, var(--bg-alt) 100%);
    }}
    main {{
      max-width: 1200px;
      margin: 0 auto;
      padding: 24px;
    }}
    .card {{
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 14px 34px rgba(0, 0, 0, 0.22);
      backdrop-filter: blur(4px);
    }}
    h1, h2 {{
      text-transform: uppercase;
      letter-spacing: 0.09em;
      font-family: "Avenir Next Condensed", "Franklin Gothic Medium", "Arial Narrow", sans-serif;
    }}
    h1 {{
      color: var(--accent);
    }}
    .nav {{
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      margin-top: 14px;
    }}
    .nav-link {{
      display: inline-block;
      text-decoration: none;
      color: var(--ink);
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 8px 14px;
      background: rgba(52, 53, 53, 0.72);
      font-size: 0.92rem;
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }}
    .log-list {{
      display: grid;
      gap: 12px;
      margin-top: 14px;
    }}
    .log-entry {{
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 12px;
      background: var(--panel);
    }}
    .log-entry.warn {{
      border-color: rgba(180, 83, 9, 0.35);
    }}
    .log-entry.error, .log-entry.fatal {{
      border-color: rgba(185, 28, 28, 0.35);
    }}
    .log-meta {{
      font-size: 0.82rem;
      color: var(--muted);
      margin-bottom: 6px;
    }}
    pre {{
      margin: 0;
      white-space: pre-wrap;
      word-break: break-word;
      font-family: "SFMono-Regular", Consolas, "Liberation Mono", monospace;
      color: var(--ink);
    }}
    .muted {{ color: var(--muted); }}
  </style>
</head>
<body>
  <main>
    <section class="card">
      <h1>Developer</h1>
      <div class="muted">Inspect recent ROS warning/error logs and the raw web status payload.</div>
      <div class="nav">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Calendar</a>
        <a class="nav-link" href="/map">Missions</a>
        <a class="nav-link" href="/record-map">Record Map</a>
        <a class="nav-link" href="/developer">Developer</a>
      </div>
    </section>

    <section class="card" style="margin-top: 18px;">
      <h2>System Log</h2>
      <div class="muted">Shows recent `WARN`, `ERROR`, and `FATAL` messages from ROS.</div>
      <div id="log-list" class="log-list"></div>
    </section>

    <section class="card" style="margin-top: 18px;">
      <h2>Raw Status</h2>
      <pre id="raw-status">{{}}</pre>
    </section>
  </main>

  <script>
    const logList = document.getElementById('log-list');
    const rawStatus = document.getElementById('raw-status');

    async function loadStatus() {{
      const response = await fetch('/api/status', {{ cache: 'no-store' }});
      const data = await response.json();
      const recentLogs = data.recent_logs || [];

      logList.innerHTML = '';
      if (recentLogs.length === 0) {{
        logList.innerHTML = '<div class="muted">No warning, error, or fatal messages captured yet.</div>';
      }} else {{
        for (const entry of recentLogs) {{
          const item = document.createElement('div');
          item.className = `log-entry ${{String(entry.level || '').toLowerCase()}}`;
          item.innerHTML = `
            <div class="log-meta">${{entry.level || 'WARN'}} | ${{entry.name || '-'}} | line ${{entry.line ?? '-'}} </div>
            <div>${{entry.msg || ''}}</div>
          `;
          logList.appendChild(item);
        }}
      }}

      rawStatus.textContent = JSON.stringify(data, null, 2);
    }}

    loadStatus();
    setInterval(loadStatus, 2000);
  </script>
</body>
</html>
"""


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = MissionFrontendHttpNode()
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
        node.get_logger().error(f"Mission frontend HTTP startup failed: {exc}")
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
