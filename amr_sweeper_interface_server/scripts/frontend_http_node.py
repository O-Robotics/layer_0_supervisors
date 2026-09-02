#!/usr/bin/env python3

from __future__ import annotations

import errno
import json
import socket
import threading
import time
import urllib.parse
from html import escape
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


DEFAULT_BACKEND_SOCKET_PATH = "/tmp/amr_sweeper_interface_backend.sock"


class MissionThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class MissionFrontendRenderer:
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
        <a class="nav-link" href="/teleop">Teleop</a>
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
      const response = await fetch('/api/v1/status', {{ cache: 'no-store' }});
      const data = await response.json();
      const fsm = data.fsm_status || data.fsm_state || {{}};
      const fsmDisplay = data.fsm_display || {{}};
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

      const rawCurrentState = fsm.current_state || 'Unknown';
      const rawCurrentProfile = formatProfileValue(fsm.current_profile);
      const currentState = fsmDisplay.current_state || rawCurrentState;
      const currentProfile = formatProfileValue(
        fsmDisplay.current_profile !== undefined && fsmDisplay.current_profile !== null
          ? fsmDisplay.current_profile
          : fsm.current_profile
      );
      const targetProfile = fsm.transitioning_to_profile ?? null;
      const transitionInProgress = Boolean(fsmDisplay.transition_active);
      const targetState = deriveStateFromProfile(targetProfile) || rawCurrentState;
      const formattedTargetProfile = formatProfileValue(targetProfile);

      document.getElementById('fsm-state').textContent = transitionInProgress
        ? formatArrowValue(currentState, targetState)
        : currentState;
      document.getElementById('fsm-profile').textContent = transitionInProgress
        ? `Profile: ${{formatArrowValue(currentProfile, formattedTargetProfile)}}`
        : `Profile: ${{currentProfile}}`;

      const transitionElement = document.getElementById('fsm-transition');
      if (transitionInProgress) {{
        const progressLabel = fsmDisplay.transition_progress || 'Transition in progress';
        transitionElement.innerHTML = `<span class="inline-status"><span class="spinner" aria-hidden="true"></span><span>${{progressLabel}}</span></span>`;
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
      const response = await fetch('/api/v1/mission/stop', {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/json' }},
        body: JSON.stringify({{}})
      }});
      const data = await response.json();
      setBanner(data.success ? 'ok' : 'error', data.message || 'Stop request completed');
      await loadStatus();
    }});

    document.getElementById('reboot-button').addEventListener('click', async () => {{
      const response = await fetch('/api/v1/system/reinitialize', {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/json' }},
        body: JSON.stringify({{}})
      }});
      const data = await response.json();
      setBanner(data.success ? 'ok' : 'error', data.message || 'Reboot request completed');
      await loadStatus();
    }});

    document.getElementById('safety-stop-button').addEventListener('click', async () => {{
      const path = lastSafetyLatched ? '/api/v1/safety/clear' : '/api/v1/safety/stop';
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
        <a class="nav-link" href="/teleop">Teleop</a>
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
              <option value="minutely">Continuously every N minutes</option>
              <option value="daily">Daily</option>
              <option value="monthly_nth_weekday">Monthly on this weekday occurrence</option>
            </select>
          </label>
          <label>
            Repeat Interval Minutes
            <input id="entry-recurrence-interval-minutes" type="number" min="1" step="1" value="10">
          </label>
          <label>
            Continuous Duration Minutes
            <input id="entry-continuous-duration-minutes" type="number" min="1" step="1" value="180">
          </label>
          <label class="span-2">
            <input id="entry-record-rosbag" type="checkbox">
            Record rosbag
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
      document.getElementById('entry-recurrence-interval-minutes').value = '10';
      document.getElementById('entry-continuous-duration-minutes').value = '180';
      document.getElementById('entry-record-rosbag').checked = false;
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
      document.getElementById('entry-recurrence-interval-minutes').value = entry.recurrence_interval_minutes || 10;
      if (entry.recurrence_until_local && entry.start_local) {{
        const start = parseLocalDateTime(entry.start_local);
        const until = parseLocalDateTime(entry.recurrence_until_local);
        const durationMinutes = Math.max(1, Math.round((until - start) / 60000));
        document.getElementById('entry-continuous-duration-minutes').value = String(durationMinutes);
      }} else {{
        document.getElementById('entry-continuous-duration-minutes').value = '180';
      }}
      document.getElementById('entry-record-rosbag').checked = Boolean(entry.record_rosbag);
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
          const result = await postJson('/api/v1/schedule/entry/delete', {{ uid: entry.uid }});
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
      const response = await fetch(`/api/v1/schedule?week=${{encodeURIComponent(week)}}`, {{ cache: 'no-store' }});
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
        recurrence_interval_minutes: document.getElementById('entry-recurrence-interval-minutes').value,
        continuous_duration_minutes: document.getElementById('entry-continuous-duration-minutes').value,
        record_rosbag: document.getElementById('entry-record-rosbag').checked,
        description: document.getElementById('entry-description').value,
      }};
      const result = await postJson('/api/v1/schedule/entry', payload);
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
        <a class="nav-link" href="/teleop">Teleop</a>
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
      const response = await fetch('/api/v1/record-map', {{ cache: 'no-store' }});
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
      const data = await postJson('/api/v1/record-map/start', {{}});
      setBanner(data.success ? 'ok' : 'error', data.message || 'RecordMap request completed');
      await loadRecordMapSnapshot();
    }});

    document.getElementById('stop-button').addEventListener('click', async () => {{
      const data = await postJson('/api/v1/record-map/stop', {{}});
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
      const data = await postJson('/api/v1/record-map/save-mission', {{
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

    def render_teleop_html(self) -> str:
        title = escape(self._site_title)
        return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{title} - Teleop</title>
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
      --danger: #ff7b5c;
      --line: rgba(253, 202, 15, 0.22);
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      min-height: 100vh;
      font-family: "Avenir Next", "Segoe UI", sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(253, 202, 15, 0.18), transparent 26%),
        linear-gradient(180deg, var(--bg) 0%, var(--bg-alt) 100%);
      touch-action: manipulation;
    }}
    main {{
      max-width: 1180px;
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
      margin: 0 0 12px;
      text-transform: uppercase;
      letter-spacing: 0.09em;
      font-family: "Avenir Next Condensed", "Franklin Gothic Medium", "Arial Narrow", sans-serif;
    }}
    h1 {{ color: var(--accent); }}
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
    .teleop-layout {{
      display: grid;
      grid-template-columns: minmax(220px, 1fr) 180px minmax(220px, 1fr);
      gap: 22px;
      align-items: center;
    }}
    .teleop-layout.one-stick {{
      grid-template-columns: minmax(320px, 1fr) 180px;
    }}
    .teleop-stage {{
      position: relative;
      overflow: hidden;
      margin-top: 18px;
    }}
    .teleop-stage::before {{
      content: "";
      position: absolute;
      inset: 0;
      z-index: 1;
      pointer-events: none;
      background: rgba(5, 8, 10, 0.34);
      opacity: 0;
      transition: opacity 160ms ease;
    }}
    .teleop-stage.camera-active::before {{
      opacity: 1;
    }}
    .teleop-stage.camera-waiting::before {{
      opacity: 0.52;
    }}
    .camera-feed {{
      position: absolute;
      inset: 0;
      z-index: 0;
      display: none;
      width: 100%;
      height: 100%;
      object-fit: cover;
      background: #101214;
    }}
    .teleop-stage.camera-active .camera-feed {{
      display: block;
    }}
    .teleop-stage > .teleop-layout {{
      position: relative;
      z-index: 2;
    }}
    .camera-message {{
      position: absolute;
      inset: 0;
      z-index: 3;
      display: none;
      place-items: center;
      pointer-events: none;
      color: var(--ink);
      font-size: 1.05rem;
      font-weight: 700;
      letter-spacing: 0.04em;
      text-transform: uppercase;
      text-align: center;
      padding: 24px;
      text-shadow: 0 2px 12px rgba(0, 0, 0, 0.74);
    }}
    .teleop-stage.camera-waiting .camera-message {{
      display: grid;
    }}
    .stick-panel {{
      display: grid;
      justify-items: center;
      gap: 12px;
    }}
    .teleop-layout.one-stick .tools-panel {{
      display: none;
    }}
    .stick-cluster {{
      --stick-size: min(32vw, 310px);
      --bar-width: clamp(42px, 6vw, 58px);
      --cluster-gap: clamp(8px, 1.6vw, 12px);
      display: grid;
      grid-template-columns: var(--bar-width) var(--stick-size) var(--bar-width);
      gap: var(--cluster-gap);
      align-items: center;
      justify-content: center;
      width: 100%;
    }}
    .stick-shell {{
      width: var(--stick-size);
      min-width: 0;
      aspect-ratio: 1;
      border-radius: 50%;
      border: 3px solid rgba(253, 202, 15, 0.55);
      background: rgba(18, 20, 21, 0.44);
      position: relative;
      touch-action: none;
      user-select: none;
    }}
    .stick-knob {{
      width: 31%;
      aspect-ratio: 1;
      border-radius: 50%;
      background: var(--accent);
      box-shadow: 0 12px 24px rgba(0, 0, 0, 0.32);
      position: absolute;
      left: 50%;
      top: 50%;
      transform: translate(-50%, -50%);
      pointer-events: none;
    }}
    .scale-slot {{
      width: var(--bar-width);
      min-height: calc(var(--stick-size) * 0.72);
      display: flex;
      align-items: center;
      justify-content: center;
    }}
    .speed-scale {{
      width: var(--bar-width);
      height: calc(var(--stick-size) * 0.86);
      min-height: 0;
      max-height: 270px;
      display: flex;
      flex-direction: column-reverse;
      align-items: center;
      justify-content: space-between;
      padding: 7px 0;
      touch-action: none;
      user-select: none;
      cursor: pointer;
    }}
    .scale-segment {{
      height: 8px;
      border-radius: 2px;
      background: rgba(8, 9, 10, 0.88);
      border: 1px solid rgba(245, 241, 223, 0.08);
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.04);
      transition: background 90ms ease, border-color 90ms ease;
    }}
    .scale-segment.active {{
      background: linear-gradient(90deg, #f59e0b 0%, var(--accent) 100%);
      border-color: rgba(253, 202, 15, 0.68);
      box-shadow: 0 0 10px rgba(253, 202, 15, 0.2);
    }}
    .center-controls {{
      display: grid;
      gap: 14px;
      justify-items: stretch;
      align-content: center;
    }}
    button {{
      border: 0;
      border-radius: 999px;
      padding: 13px 18px;
      font-size: 0.95rem;
      cursor: pointer;
      color: #08100a;
      background: var(--accent);
      font-weight: 700;
      letter-spacing: 0.04em;
      text-transform: uppercase;
      min-height: 48px;
    }}
    button:hover {{ background: var(--accent-strong); }}
    button.stop {{ background: var(--danger); color: #fff8f6; }}
    button:disabled {{
      cursor: not-allowed;
      background: #5c5b55;
      color: #d7ddd8;
    }}
    .gear {{
      display: inline-block;
      font-size: 1.15rem;
      animation: spin 0.95s linear infinite, pulse-fade 1.2s ease-in-out infinite;
    }}
    .toggle-button.enabled {{
      background: #f8fafc;
      color: #111827;
      box-shadow: 0 0 0 3px rgba(239, 68, 68, 0.32);
    }}
    .mode-button.enabled {{
      background: #fff4bd;
      color: #111827;
      box-shadow: 0 0 0 3px rgba(253, 202, 15, 0.24);
    }}
    .camera-button.enabled {{
      background: #d7f3ff;
      color: #062233;
      box-shadow: 0 0 0 3px rgba(14, 165, 233, 0.28);
    }}
    .status-row {{
      display: flex;
      gap: 12px;
      flex-wrap: wrap;
      margin-top: 14px;
      color: var(--muted);
    }}
    .status-value {{
      color: var(--ink);
      font-weight: 700;
    }}
    .banner {{
      margin-top: 14px;
      padding: 12px 14px;
      border-radius: 12px;
      display: none;
    }}
    .banner.show {{ display: block; }}
    .banner.ok {{ background: rgba(15, 118, 110, 0.12); color: var(--accent-strong); }}
    .banner.error {{ background: rgba(185, 28, 28, 0.12); color: var(--danger); }}
    .muted {{ color: var(--muted); }}
    @keyframes spin {{
      from {{ transform: rotate(0deg); }}
      to {{ transform: rotate(360deg); }}
    }}
    @keyframes pulse-fade {{
      0%, 100% {{ opacity: 0.45; }}
      50% {{ opacity: 1; }}
    }}
    @media (max-width: 820px) {{
      .teleop-layout {{ grid-template-columns: 1fr; }}
      .teleop-layout.one-stick {{ grid-template-columns: 1fr; }}
      .center-controls {{ grid-row: 1; }}
      .stick-cluster {{
        --cluster-gap: clamp(6px, 2vw, 10px);
        --bar-width: clamp(34px, 10vw, 48px);
        --stick-size: min(
          310px,
          calc((100vw - 48px - (2 * var(--bar-width)) - (2 * var(--cluster-gap))) * 0.98)
        );
      }}
    }}
  </style>
</head>
<body>
  <main>
    <section class="card">
      <h1>Teleop</h1>
      <div id="banner" class="banner"></div>
      <div class="nav">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Calendar</a>
        <a class="nav-link" href="/map">Missions</a>
        <a class="nav-link" href="/teleop">Teleop</a>
        <a class="nav-link" href="/record-map">Record Map</a>
        <a class="nav-link" href="/developer">Developer</a>
      </div>
      <div class="status-row">
        <div>FSM: <span id="fsm-state" class="status-value">--</span></div>
        <div>Profile: <span id="fsm-profile" class="status-value">--</span></div>
        <div>Mission: <span id="active-mission" class="status-value">--</span></div>
      </div>
    </section>

    <section id="teleop-stage" class="card teleop-stage">
      <img id="camera-feed" class="camera-feed" alt="">
      <div id="camera-message" class="camera-message">Waiting for Teleop to start</div>
      <div id="teleop-layout" class="teleop-layout one-stick">
        <section class="stick-panel drive-panel">
          <h2>Drive</h2>
          <div class="stick-cluster">
            <div class="scale-slot">
              <div id="wheel-scale" class="speed-scale" role="slider" aria-label="Wheel speed" aria-valuemin="0" aria-valuemax="100" aria-valuenow="50"></div>
            </div>
            <div id="left-stick" class="stick-shell" aria-label="Drive joystick">
              <div id="left-knob" class="stick-knob"></div>
            </div>
            <div id="drive-tool-scale-slot" class="scale-slot"></div>
          </div>
        </section>
        <section class="center-controls">
          <button id="teleop-toggle-button" type="button">Start</button>
          <button id="two-stick-button" class="mode-button" type="button">Two Stick</button>
          <button id="lights-button" class="toggle-button" type="button">Lights</button>
          <button id="camera-button" class="camera-button" type="button">Camera</button>
        </section>
        <section id="tools-panel" class="stick-panel tools-panel">
          <h2>Tools</h2>
          <div class="stick-cluster">
            <div class="scale-slot"></div>
            <div id="right-stick" class="stick-shell" aria-label="Tool joystick">
              <div id="right-knob" class="stick-knob"></div>
            </div>
            <div id="tool-scale-slot" class="scale-slot"></div>
          </div>
        </section>
      </div>
      <div id="tool-scale" class="speed-scale" role="slider" aria-label="Tool speed" aria-valuemin="0" aria-valuemax="100" aria-valuenow="50"></div>
    </section>
  </main>

  <script>
    const banner = document.getElementById('banner');
    const teleopToggleButton = document.getElementById('teleop-toggle-button');
    const twoStickButton = document.getElementById('two-stick-button');
    const lightsButton = document.getElementById('lights-button');
    const cameraButton = document.getElementById('camera-button');
    const cameraFeed = document.getElementById('camera-feed');
    const teleopStage = document.getElementById('teleop-stage');
    const teleopLayout = document.getElementById('teleop-layout');
    const driveToolScaleSlot = document.getElementById('drive-tool-scale-slot');
    const toolScaleSlot = document.getElementById('tool-scale-slot');
    const speedScales = {{
      wheel: {{ value: 0.5, shell: document.getElementById('wheel-scale'), pointerId: null }},
      tool: {{ value: 0.5, shell: document.getElementById('tool-scale'), pointerId: null }},
    }};
    let teleopReady = false;
    let transitionBusy = false;
    let twoStickEnabled = false;
    let lightsEnabled = false;
    let cameraEnabled = false;
    let cameraStreamActive = false;
    let commandInFlight = false;
    const sticks = {{
      left: {{ x: 0, y: 0, shell: document.getElementById('left-stick'), knob: document.getElementById('left-knob'), pointerId: null }},
      right: {{ x: 0, y: 0, shell: document.getElementById('right-stick'), knob: document.getElementById('right-knob'), pointerId: null }},
    }};

    function setBanner(kind, message) {{
      banner.className = `banner show ${{kind}}`;
      banner.textContent = message;
      setTimeout(() => {{
        banner.className = 'banner';
        banner.textContent = '';
      }}, 5000);
    }}
    function formatProfileValue(profile) {{
      const numericProfile = Number(profile);
      return Number.isFinite(numericProfile) ? String(Math.trunc(numericProfile)).padStart(3, '0') : '--';
    }}
    function clamp(value, min, max) {{ return Math.max(min, Math.min(max, value)); }}
    function normalizeToCircle(x, y) {{
      const length = Math.hypot(x, y);
      return length <= 1 ? [x, y] : [x / length, y / length];
    }}
    function updateKnob(stick) {{
      const shellRect = stick.shell.getBoundingClientRect();
      const travel = (shellRect.width * 0.5) - (shellRect.width * 0.155);
      stick.knob.style.transform = `translate(calc(-50% + ${{stick.x * travel}}px), calc(-50% + ${{-stick.y * travel}}px))`;
    }}
    function renderScale(scale) {{
      const activeCount = Math.round(scale.value * scale.shell.children.length);
      [...scale.shell.children].forEach((segment, index) => {{
        segment.classList.toggle('active', index < activeCount);
      }});
      scale.shell.setAttribute('aria-valuenow', String(Math.round(scale.value * 100)));
    }}
    function handleScalePointer(scale, event) {{
      const rect = scale.shell.getBoundingClientRect();
      const y = clamp(event.clientY - rect.top, 0, rect.height);
      scale.value = clamp(1 - (y / rect.height), 0, 1);
      renderScale(scale);
    }}
    function createScaleSegments(scale) {{
      const segmentCount = 20;
      for (let index = 0; index < segmentCount; index += 1) {{
        const segment = document.createElement('div');
        segment.className = 'scale-segment';
        segment.style.width = `${{24 + index * 1.15}}px`;
        scale.shell.appendChild(segment);
      }}
      renderScale(scale);
    }}
    function renderControlMode(resetHiddenToolStick = true) {{
      teleopLayout.classList.toggle('one-stick', !twoStickEnabled);
      twoStickButton.classList.toggle('enabled', twoStickEnabled);
      if (twoStickEnabled) {{
        toolScaleSlot.appendChild(speedScales.tool.shell);
      }} else {{
        driveToolScaleSlot.appendChild(speedScales.tool.shell);
        if (resetHiddenToolStick) {{
          resetStick(sticks.right);
        }}
      }}
      twoStickButton.textContent = twoStickEnabled ? 'One Stick' : 'Two Stick';
    }}
    function handlePointer(stick, event) {{
      const rect = stick.shell.getBoundingClientRect();
      const radius = rect.width * 0.5;
      const rawX = (event.clientX - (rect.left + radius)) / radius;
      const rawY = -((event.clientY - (rect.top + radius)) / radius);
      const [x, y] = normalizeToCircle(rawX, rawY);
      stick.x = clamp(x, -1, 1);
      stick.y = clamp(y, -1, 1);
      updateKnob(stick);
    }}
    function resetStick(stick) {{
      stick.x = 0;
      stick.y = 0;
      stick.pointerId = null;
      updateKnob(stick);
      sendZeroCommand();
    }}
    for (const stick of Object.values(sticks)) {{
      stick.shell.addEventListener('pointerdown', (event) => {{
        stick.pointerId = event.pointerId;
        stick.shell.setPointerCapture(event.pointerId);
        handlePointer(stick, event);
      }});
      stick.shell.addEventListener('pointermove', (event) => {{
        if (stick.pointerId === event.pointerId) {{
          handlePointer(stick, event);
        }}
      }});
      stick.shell.addEventListener('pointerup', () => resetStick(stick));
      stick.shell.addEventListener('pointercancel', () => resetStick(stick));
      updateKnob(stick);
    }}
    for (const scale of Object.values(speedScales)) {{
      createScaleSegments(scale);
      scale.shell.addEventListener('pointerdown', (event) => {{
        scale.pointerId = event.pointerId;
        scale.shell.setPointerCapture(event.pointerId);
        handleScalePointer(scale, event);
      }});
      scale.shell.addEventListener('pointermove', (event) => {{
        if (scale.pointerId === event.pointerId) {{
          handleScalePointer(scale, event);
        }}
      }});
      scale.shell.addEventListener('pointerup', () => {{
        scale.pointerId = null;
      }});
      scale.shell.addEventListener('pointercancel', () => {{
        scale.pointerId = null;
      }});
    }}
    renderControlMode(false);

    async function postJson(path, body, keepalive = false) {{
      const response = await fetch(path, {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/json' }},
        body: JSON.stringify(body || {{}}),
        keepalive,
      }});
      return response.json();
    }}
    function commandPayload() {{
      return {{
        left_x: sticks.left.x,
        left_y: sticks.left.y,
        right_x: twoStickEnabled ? sticks.right.x : 0,
        right_y: twoStickEnabled ? sticks.right.y : 0,
        control_mode: twoStickEnabled ? 'two_stick' : 'one_stick',
        wheel_scale: speedScales.wheel.value,
        tool_scale: speedScales.tool.value,
      }};
    }}
    function closeCameraStream() {{
      if (cameraStreamActive) {{
        cameraFeed.removeAttribute('src');
        cameraStreamActive = false;
      }}
    }}
    function disableCamera() {{
      cameraEnabled = false;
      closeCameraStream();
      cameraButton.classList.remove('enabled');
      teleopStage.classList.remove('camera-active', 'camera-waiting');
    }}
    function updateCameraState() {{
      cameraButton.classList.toggle('enabled', cameraEnabled);
      if (!cameraEnabled) {{
        closeCameraStream();
        teleopStage.classList.remove('camera-active', 'camera-waiting');
        return;
      }}
      if (!teleopReady) {{
        closeCameraStream();
        teleopStage.classList.add('camera-waiting');
        teleopStage.classList.remove('camera-active');
        return;
      }}
      teleopStage.classList.add('camera-active');
      teleopStage.classList.remove('camera-waiting');
      if (!cameraStreamActive) {{
        cameraFeed.src = `/api/v1/teleop/camera/stream?ts=${{Date.now()}}`;
        cameraStreamActive = true;
      }}
    }}
    async function sendZeroCommand(keepalive = false) {{
      try {{
        await postJson('/api/v1/teleop/command', {{
          left_x: 0,
          left_y: 0,
          right_x: 0,
          right_y: 0,
          control_mode: twoStickEnabled ? 'two_stick' : 'one_stick',
          wheel_scale: speedScales.wheel.value,
          tool_scale: speedScales.tool.value,
        }}, keepalive);
      }} catch (_error) {{}}
    }}
    async function streamCommand() {{
      if (!teleopReady || document.visibilityState !== 'visible' || commandInFlight) {{
        return;
      }}
      commandInFlight = true;
      try {{
        await postJson('/api/v1/teleop/command', commandPayload());
      }} catch (_error) {{
      }} finally {{
        commandInFlight = false;
      }}
    }}
    function setBusyButton(label) {{
      transitionBusy = true;
      teleopToggleButton.disabled = true;
      teleopToggleButton.classList.remove('stop');
      teleopToggleButton.innerHTML = `<span class="gear" aria-hidden="true">&#9881;</span><span style="position:absolute;left:-9999px;">${{label}}</span>`;
    }}
    function renderButton() {{
      if (transitionBusy) {{
        return;
      }}
      teleopToggleButton.disabled = false;
      teleopToggleButton.classList.toggle('stop', teleopReady);
      teleopToggleButton.textContent = teleopReady ? 'Stop' : 'Start';
    }}
    async function loadStatus() {{
      const response = await fetch('/api/v1/status', {{ cache: 'no-store' }});
      const data = await response.json();
      const fsm = data.fsm_status || data.fsm_state || {{}};
      const display = data.fsm_display || {{}};
      const active = data.active_execution || {{}};
      const transitionActive = Boolean(display.transition_active) ||
        String(fsm.transition_status || '').toUpperCase() === 'TRANSITIONING';
      teleopReady = String(fsm.current_state || '').toUpperCase() === 'RUNNING' &&
        Number(fsm.current_profile) === 220 &&
        !transitionActive &&
        active &&
        active.mission_id === 'Teleop' &&
        active.active !== false;
      lightsEnabled = Boolean(data.teleop_lights_enabled);
      lightsButton.classList.toggle('enabled', lightsEnabled);
      transitionBusy = transitionActive && (
        Number(fsm.transitioning_to_profile) === 220 ||
        Number(fsm.current_profile) === 220 ||
        active.mission_id === 'Teleop'
      );
      document.getElementById('fsm-state').textContent = display.current_state || fsm.current_state || 'Unknown';
      document.getElementById('fsm-profile').textContent = formatProfileValue(
        display.current_profile !== undefined && display.current_profile !== null ? display.current_profile : fsm.current_profile
      );
      document.getElementById('active-mission').textContent = active?.mission_id || '--';
      if (transitionBusy) {{
        setBusyButton('Transitioning');
      }} else {{
        renderButton();
      }}
      updateCameraState();
    }}

    teleopToggleButton.addEventListener('click', async () => {{
      try {{
        if (teleopReady) {{
          setBusyButton('Stopping');
          await sendZeroCommand();
          const result = await postJson('/api/v1/teleop/stop', {{}});
          setBanner(result.success ? 'ok' : 'error', result.message || 'Teleop stop request completed');
        }} else {{
          setBusyButton('Starting');
          const result = await postJson('/api/v1/teleop/start', {{}});
          setBanner(result.success ? 'ok' : 'error', result.message || 'Teleop start request completed');
        }}
      }} catch (error) {{
        setBanner('error', error.message || 'Teleop request failed');
      }} finally {{
        await loadStatus();
      }}
    }});
    lightsButton.addEventListener('click', async () => {{
      const nextEnabled = !lightsEnabled;
      lightsButton.disabled = true;
      try {{
        const result = await postJson('/api/v1/teleop/lights', {{ enabled: nextEnabled }});
        if (result.success) {{
          lightsEnabled = nextEnabled;
          lightsButton.classList.toggle('enabled', lightsEnabled);
        }}
        setBanner(result.success ? 'ok' : 'error', result.message || 'Lights request completed');
      }} catch (error) {{
        setBanner('error', error.message || 'Lights request failed');
      }} finally {{
        lightsButton.disabled = false;
      }}
    }});
    twoStickButton.addEventListener('click', () => {{
      twoStickEnabled = !twoStickEnabled;
      renderControlMode();
    }});
    cameraButton.addEventListener('click', () => {{
      cameraEnabled = !cameraEnabled;
      updateCameraState();
    }});
    document.addEventListener('visibilitychange', () => {{
      if (document.visibilityState !== 'visible') {{
        disableCamera();
        sendZeroCommand(true);
      }}
    }});
    window.addEventListener('pagehide', () => {{
      disableCamera();
      sendZeroCommand(true);
    }});
    window.addEventListener('beforeunload', () => {{
      disableCamera();
      const body = JSON.stringify({{
        left_x: 0,
        left_y: 0,
        right_x: 0,
        right_y: 0,
        control_mode: twoStickEnabled ? 'two_stick' : 'one_stick',
        wheel_scale: speedScales.wheel.value,
        tool_scale: speedScales.tool.value,
      }});
      navigator.sendBeacon('/api/v1/teleop/command', new Blob([body], {{ type: 'application/json' }}));
    }});
    setInterval(streamCommand, 50);
    loadStatus().catch((error) => setBanner('error', error.message || 'Failed to load teleop status'));
    setInterval(() => loadStatus().catch(() => null), 500);
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
    .hero-actions {{
      display: flex;
      gap: 12px;
      align-items: center;
      flex-wrap: wrap;
      margin-top: 12px;
    }}
    .mission-option-toggle {{
      display: inline-flex;
      align-items: center;
      gap: 10px;
      padding: 10px 14px;
      border-radius: 999px;
      border: 1px solid var(--line);
      background: rgba(52, 53, 53, 0.72);
      color: var(--ink);
      font-size: 0.92rem;
      text-transform: uppercase;
      letter-spacing: 0.05em;
      cursor: pointer;
      user-select: none;
    }}
    .mission-option-toggle input {{
      accent-color: var(--accent);
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
      overflow: hidden;
      position: relative;
    }}
    #mission-preview-map {{
      width: 100%;
      min-height: 520px;
      display: none;
    }}
    #mission-preview-local {{
      width: 100%;
      min-height: 520px;
      display: flex;
      align-items: center;
      justify-content: center;
      padding: 0;
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
    button.secondary {{
      background: rgba(96, 100, 102, 0.72);
      color: var(--ink);
      border: 1px solid var(--line);
    }}
    button.selected {{
      background: #fff3bb;
      color: #08100a;
      box-shadow: 0 0 0 2px rgba(253, 202, 15, 0.35);
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
      <div id="banner" class="banner"></div>
      <div class="nav">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Calendar</a>
        <a class="nav-link" href="/map">Missions</a>
        <a class="nav-link" href="/teleop">Teleop</a>
        <a class="nav-link" href="/record-map">Record Map</a>
        <a class="nav-link" href="/developer">Developer</a>
      </div>
      <div class="hero-actions">
        <button id="start-mission-button" disabled>Start Mission</button>
        <div id="selected-mission-label" class="muted">Selected mission: none</div>
        <label class="mission-option-toggle">
          <input id="record-rosbag-toggle" type="checkbox">
          <span>Record rosbag</span>
        </label>
      </div>
    </section>
    <section class="map-layout">
      <section class="card">
        <div class="map-frame">
          <div id="mission-preview-map"></div>
          <div id="mission-preview-local"></div>
        </div>
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
      <div class="muted">Paste a VDA5050 package JSON object with `order`, optional `zoneSet`, and `map_georeference`. You can optionally provide a mission id; otherwise `order.orderId` is used.</div>
      <div style="display: grid; gap: 12px; margin-top: 14px;">
        <input id="upload-file" type="file" accept=".json,application/json" style="padding: 12px; border-radius: 12px; border: 1px solid var(--line); background: rgba(18, 20, 21, 0.82); color: var(--ink);">
        <input id="upload-mission-id" type="text" placeholder="Optional mission id" style="padding: 12px; border-radius: 12px; border: 1px solid var(--line); background: rgba(18, 20, 21, 0.82); color: var(--ink);">
        <label class="muted"><input id="upload-overwrite" type="checkbox"> Overwrite existing mission with same id</label>
        <textarea id="upload-json" rows="14" placeholder='{{"order":{{"orderId":"field_block_12","version":"3.0.0","nodes":[...],"edges":[...]}},"map_georeference":{{"mapId":"field_block_12_map","originLatitude":55.0,"originLongitude":10.0,"bounds":{{"min_x":0,"min_y":0,"max_x":10,"max_y":10}}}}}}' style="width: 100%; padding: 12px; border-radius: 12px; border: 1px solid var(--line); background: rgba(18, 20, 21, 0.82); color: var(--ink); font-family: monospace;"></textarea>
        <div>
          <button id="upload-button">Upload Mission</button>
        </div>
      </div>
    </section>
  </main>
  <script
    src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
    integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo="
    crossorigin=""
  ></script>
  <script>
    const banner = document.getElementById('banner');
    const missionList = document.getElementById('mission-list');
    const startMissionButton = document.getElementById('start-mission-button');
    const selectedMissionLabel = document.getElementById('selected-mission-label');
    const recordRosbagToggle = document.getElementById('record-rosbag-toggle');
    const previewMapElement = document.getElementById('mission-preview-map');
    const previewLocalElement = document.getElementById('mission-preview-local');
    let mapDataCache = null;
    let missionsCache = [];
    let selectedMissionId = '';
    let previewMap = null;
    let previewTileLayer = null;
    let previewRouteLayer = null;
    let previewMarker = null;
    let previewCrsMode = '';
    const layerToggleDefinitions = [
      ['use_amr_sweeper_ros2_control', 'ROS2 Control'],
      ['use_amr_sweeper_battery', 'Battery'],
      ['use_amr_sweeper_system_info', 'System Info'],
      ['use_amr_sweeper_usb_cameras', 'USB Cameras'],
      ['use_amr_sweeper_depth_camera', 'Depth Camera'],
      ['use_amr_sweeper_imu', 'IMU'],
      ['use_amr_sweeper_gnss', 'GNSS'],
      ['use_ntrip_client', 'NTRIP Client'],
      ['use_amr_sweeper_drive_controller', 'Drive Controller'],
      ['use_amr_sweeper_tool_controller', 'Tool Controller'],
      ['use_amr_sweeper_teleop', 'Teleop'],
      ['use_amr_sweeper_sweeping_controller', 'Sweeping Controller'],
      ['use_amr_sweeper_attitude_controller', 'Attitude Controller'],
      ['use_amr_sweeper_collision_detector', 'Collision Detector'],
      ['use_amr_sweeper_safety_controller', 'Safety Controller'],
      ['use_joy_node', 'Joy Node'],
      ['use_amr_sweeper_visual_odometry', 'Visual Odometry'],
      ['use_amr_sweeper_localization', 'Localization'],
      ['use_amr_sweeper_mapping', 'Mapping'],
      ['use_amr_sweeper_navigation', 'Navigation'],
      ['auto_start_mission', 'Auto Start Mission'],
    ];
    const fallbackLayerOverrides = {{
      use_amr_sweeper_ros2_control: true,
      use_amr_sweeper_battery: true,
      use_amr_sweeper_system_info: true,
      use_amr_sweeper_usb_cameras: true,
      use_amr_sweeper_depth_camera: true,
      use_amr_sweeper_imu: true,
      use_amr_sweeper_gnss: true,
      use_ntrip_client: true,
      use_amr_sweeper_drive_controller: true,
      use_amr_sweeper_tool_controller: true,
      use_amr_sweeper_teleop: true,
      use_amr_sweeper_sweeping_controller: true,
      use_amr_sweeper_attitude_controller: true,
      use_amr_sweeper_collision_detector: true,
      use_amr_sweeper_safety_controller: true,
      use_joy_node: false,
      use_amr_sweeper_visual_odometry: false,
      use_amr_sweeper_localization: true,
      use_amr_sweeper_mapping: false,
      use_amr_sweeper_navigation: true,
      auto_start_mission: true,
    }};

    function setBanner(kind, message) {{
      banner.className = `banner show ${{kind}}`;
      banner.textContent = message;
      setTimeout(() => {{
        banner.className = 'banner';
        banner.textContent = '';
      }}, 5000);
    }}

    function routeFrameFromGeojson(geojson) {{
      for (const feature of geojson?.features || []) {{
        const properties = feature?.properties || {{}};
        const coordinateFrame = String(properties.coordinate_frame || '').trim();
        if (coordinateFrame) {{
          return coordinateFrame;
        }}
      }}
      return '';
    }}

    function missionSelectionStorageKey() {{
      return 'amr_sweeper_selected_mission_id';
    }}

    function missionLayerOverridesStorageKey(missionId) {{
      return `amr_sweeper_layer_overrides_${{missionId}}`;
    }}

    function missionRecordRosbagStorageKey(missionId) {{
      return `amr_sweeper_record_rosbag_${{missionId}}`;
    }}

    function defaultLayerOverridesForMission(mission) {{
      return {{
        ...fallbackLayerOverrides,
        ...(mission?.profile_default_overrides || {{}}),
      }};
    }}

    function layerOverridesForMission(mission) {{
      if (!mission?.mission_id) {{
        return defaultLayerOverridesForMission(mission);
      }}
      const stored = window.localStorage.getItem(missionLayerOverridesStorageKey(mission.mission_id));
      if (!stored) {{
        return defaultLayerOverridesForMission(mission);
      }}
      try {{
        const parsed = JSON.parse(stored);
        return {{
          ...defaultLayerOverridesForMission(mission),
          ...parsed,
        }};
      }} catch (_error) {{
        return defaultLayerOverridesForMission(mission);
      }}
    }}

    function recordRosbagForMission(mission) {{
      if (!mission?.mission_id) {{
        return false;
      }}
      window.localStorage.removeItem(missionRecordRosbagStorageKey(mission.mission_id));
      return false;
    }}

    function saveRecordRosbagPreference(missionId, enabled) {{
      window.localStorage.removeItem(missionRecordRosbagStorageKey(missionId));
    }}

    function isGeoReferencedRoute(geojson) {{
      const routeFrame = routeFrameFromGeojson(geojson).toLowerCase();
      if (routeFrame === 'base_footprint' || routeFrame === 'odom') {{
        return false;
      }}
      if (routeFrame.includes('wgs84') || routeFrame.includes('gps') || routeFrame.includes('utm')) {{
        return true;
      }}

      const points = [];
      for (const feature of geojson?.features || []) {{
        const geometry = feature?.geometry || {{}};
        if (geometry.type !== 'LineString') {{
          continue;
        }}
        for (const point of geometry.coordinates || []) {{
          if (Array.isArray(point) && point.length >= 2) {{
            points.push(point);
          }}
        }}
      }}
      if (points.length === 0) {{
        return false;
      }}
      return points.every((point) => {{
        const x = Number(point[0]);
        const y = Number(point[1]);
        return Number.isFinite(x) && Number.isFinite(y) && Math.abs(x) <= 180 && Math.abs(y) <= 90;
      }});
    }}

    function routeLines(geojson) {{
      const lines = [];
      for (const feature of geojson?.features || []) {{
        const geometry = feature?.geometry || {{}};
        if (geometry.type !== 'LineString') {{
          continue;
        }}
        const line = [];
        for (const point of geometry.coordinates || []) {{
          if (Array.isArray(point) && point.length >= 2) {{
            const x = Number(point[0]);
            const y = Number(point[1]);
            if (Number.isFinite(x) && Number.isFinite(y)) {{
              line.push([x, y]);
            }}
          }}
        }}
        if (line.length > 0) {{
          lines.push(line);
        }}
      }}
      return lines;
    }}

    function ensurePreviewMap(crsMode) {{
      if (previewMap && previewCrsMode === crsMode) {{
        return previewMap;
      }}
      if (previewMap) {{
        previewMap.remove();
        previewMap = null;
        previewTileLayer = null;
        previewRouteLayer = null;
        previewMarker = null;
      }}
      previewCrsMode = crsMode;
      previewMap = L.map(
        'mission-preview-map',
        {{
          zoomControl: true,
          crs: crsMode === 'local' ? L.CRS.Simple : L.CRS.EPSG3857,
        }}
      ).setView(crsMode === 'local' ? [0, 0] : [55.6761, 12.5683], crsMode === 'local' ? 18 : 16);
      if (crsMode === 'georef') {{
        previewTileLayer = L.tileLayer(
          'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}',
          {{ maxZoom: 20, attribution: '&copy; Esri', opacity: 0.2 }}
        ).addTo(previewMap);
      }}
      return previewMap;
    }}

    function showGeoreferencedPreview() {{
      previewMapElement.style.display = 'block';
      previewLocalElement.style.display = 'none';
    }}

    function showLocalPreview() {{
      previewMapElement.style.display = 'none';
      previewLocalElement.style.display = 'flex';
    }}

    function clearPreviewLayers() {{
      if (previewMap && previewRouteLayer) {{
        previewMap.removeLayer(previewRouteLayer);
        previewRouteLayer = null;
      }}
      if (previewMap && previewMarker) {{
        previewMap.removeLayer(previewMarker);
        previewMarker = null;
      }}
    }}

    function renderLocalMissionPreview(lines) {{
      showLocalPreview();
      const projectedPoints = [];
      for (const line of lines) {{
        for (const point of line) {{
          const x = Number(point[0]);
          const y = Number(point[1]);
          if (!Number.isFinite(x) || !Number.isFinite(y)) {{
            continue;
          }}
          projectedPoints.push([-y, x]);
        }}
      }}
      if (projectedPoints.length === 0) {{
        previewLocalElement.textContent = 'Mission geometry is present but contains no plottable route points.';
        return;
      }}

      const xs = projectedPoints.map((point) => point[0]);
      const ys = projectedPoints.map((point) => point[1]);
      const minX = Math.min(...xs);
      const maxX = Math.max(...xs);
      const minY = Math.min(...ys);
      const maxY = Math.max(...ys);
      const width = Math.max(1, maxX - minX);
      const height = Math.max(1, maxY - minY);
      const padding = 40;
      const viewWidth = 900;
      const viewHeight = 520;
      const usableWidth = viewWidth - padding * 2;
      const usableHeight = viewHeight - padding * 2;
      const scale = Math.min(usableWidth / width, usableHeight / height);
      const centerX = (minX + maxX) / 2;
      const centerY = (minY + maxY) / 2;

      function projectPoint(point) {{
        const rawX = Number(point[0]);
        const rawY = Number(point[1]);
        const orientedX = -rawY;
        const orientedY = rawX;
        const x = (viewWidth / 2) + ((orientedX - centerX) * scale);
        const y = (viewHeight / 2) - ((orientedY - centerY) * scale);
        return [x, y];
      }}

      const polylines = lines.map((line) => {{
        const points = line.map((point) => projectPoint(point));
        return `<polyline fill="none" stroke="#ffe06b" stroke-width="4" points="${{points.map((point) => `${{point[0].toFixed(2)}},${{point[1].toFixed(2)}}`).join(' ')}}" />`;
      }}).join('');

      let startMarker = '';
      const firstPoint = lines[0] && lines[0][0];
      if (firstPoint) {{
        const [markerX, markerY] = projectPoint(firstPoint);
        startMarker = `<circle cx="${{markerX.toFixed(2)}}" cy="${{markerY.toFixed(2)}}" r="5" fill="#1d4ed8" stroke="#ffffff" stroke-width="2" />`;
      }}

      previewLocalElement.innerHTML = `
        <svg viewBox="0 0 ${{viewWidth}} ${{viewHeight}}" width="100%" height="${{viewHeight}}" xmlns="http://www.w3.org/2000/svg">
          <rect x="0" y="0" width="${{viewWidth}}" height="${{viewHeight}}" fill="#2c3032" />
          <g opacity="0.15">
            <line x1="40" y1="40" x2="40" y2="${{viewHeight - 40}}" stroke="#eef3eb" />
            <line x1="40" y1="${{viewHeight - 40}}" x2="${{viewWidth - 40}}" y2="${{viewHeight - 40}}" stroke="#eef3eb" />
          </g>
          ${{polylines}}
          ${{startMarker}}
        </svg>
      `;
    }}

    function updateSelectedMissionLabel() {{
      selectedMissionLabel.textContent = selectedMissionId
        ? `Selected mission: ${{selectedMissionId}}`
        : 'Selected mission: none';
      startMissionButton.disabled = !selectedMissionId;
      recordRosbagToggle.disabled = !selectedMissionId;
      recordRosbagToggle.checked = selectedMissionId
        ? recordRosbagForMission(missionsCache.find((mission) => mission.mission_id === selectedMissionId) || {{ mission_id: selectedMissionId }})
        : false;
    }}

    function setSelectedMission(missionId) {{
      selectedMissionId = missionId || '';
      if (selectedMissionId) {{
        window.localStorage.setItem(missionSelectionStorageKey(), selectedMissionId);
      }}
      updateSelectedMissionLabel();
      renderMissionList();
      renderSelectedMissionPreview();
    }}

    function selectedMissionPreviewData() {{
      const previewMissions = mapDataCache?.missions || [];
      return previewMissions.find((mission) => mission.mission_id === selectedMissionId) || null;
    }}

    function renderSelectedMissionPreview() {{
      const legend = document.getElementById('legend-list');
      legend.innerHTML = '';
      const selectedMission = selectedMissionPreviewData();
      if (!selectedMission) {{
        showLocalPreview();
        clearPreviewLayers();
        previewLocalElement.textContent = 'Select a mission to preview its route geometry before starting it.';
        legend.innerHTML = '<div class="muted">Select a mission to preview its route geometry before starting it.</div>';
        return;
      }}

      if (!selectedMission.route_geojson) {{
        showLocalPreview();
        clearPreviewLayers();
        previewLocalElement.textContent = 'This mission currently has no route geometry to preview.';
        legend.innerHTML = '<div class="muted">This mission currently has no route geometry to preview.</div>';
        return;
      }}

      const georeferenced = isGeoReferencedRoute(selectedMission.route_geojson);
      const lines = routeLines(selectedMission.route_geojson);
      if (lines.length === 0) {{
        showLocalPreview();
        clearPreviewLayers();
        previewLocalElement.textContent = 'Built route geometry will appear here once mission artifacts are available.';
        legend.innerHTML = '<div class="muted">Built route geometry will appear here once mission artifacts are available.</div>';
        return;
      }}

      let markerLabel = '';
      if (georeferenced) {{
        showGeoreferencedPreview();
        previewLocalElement.innerHTML = '';
        const map = ensurePreviewMap('georef');
        clearPreviewLayers();
        const latLngLines = lines.map((line) =>
          line.map((point) => [point[1], point[0]])
        );
        previewRouteLayer = L.polyline(latLngLines, {{
          color: '#fdca0f',
          weight: 4,
        }}).addTo(map);

        let markerPosition = null;
        const livePosition = mapDataCache?.current_position || null;
        if (
          livePosition &&
          livePosition.latitude !== undefined &&
          livePosition.longitude !== undefined
        ) {{
          markerPosition = [Number(livePosition.latitude), Number(livePosition.longitude)];
          markerLabel = 'Robot live position';
        }}
        if (markerPosition) {{
          previewMarker = L.circleMarker(markerPosition, {{
            radius: 5,
            color: '#ffffff',
            weight: 2,
            fillColor: '#1d4ed8',
            fillOpacity: 1,
          }}).addTo(map);
        }}
        const bounds = previewRouteLayer.getBounds();
        if (markerPosition) {{
          bounds.extend(markerPosition);
        }}
        if (bounds.isValid()) {{
          map.fitBounds(bounds, {{ padding: [24, 24], maxZoom: 19 }});
        }}
      }} else {{
        clearPreviewLayers();
        renderLocalMissionPreview(lines);
        markerLabel = 'Waypoint 0';
      }}

      const downloadHref = `/api/v1/missions/${{encodeURIComponent(selectedMission.mission_id)}}/download`;
      const frameLabel = georeferenced ? 'North-up georeferenced preview' : 'Robot-frame preview (+X up, +Y left)';
      const item = document.createElement('div');
      item.className = 'legend-item';
      item.innerHTML = `
        <div><strong>${{selectedMission.mission_id}}</strong></div>
        <div class="muted">${{frameLabel}}</div>
        <div class="muted">${{markerLabel || 'No marker available'}}</div>
        <div class="legend-actions">
          <a class="nav-link" href="${{downloadHref}}" download>Download JSON</a>
        </div>
      `;
      legend.appendChild(item);

      const activeExecution = mapDataCache?.active_execution || null;
      if (activeExecution && activeExecution.mission_id) {{
        const activeItem = document.createElement('div');
        activeItem.className = 'legend-item';
        activeItem.innerHTML = `
          <div><strong>Active Mission</strong></div>
          <div class="muted">${{activeExecution.mission_id}}</div>
        `;
        legend.appendChild(activeItem);
      }}
    }}

    async function loadMap() {{
      const response = await fetch('/api/v1/map-data', {{ cache: 'no-store' }});
      const data = await response.json();
      mapDataCache = data;
      if (['RecordMap', 'Teleop'].includes(selectedMissionId)) {{
        selectedMissionId = '';
      }}
      if (!selectedMissionId) {{
        const firstPreviewable = (data.missions || []).find(
          (mission) => !['RecordMap', 'Teleop'].includes(mission.mission_id)
        );
        if (firstPreviewable) {{
          selectedMissionId = firstPreviewable.mission_id;
          updateSelectedMissionLabel();
        }}
      }}
      renderSelectedMissionPreview();
    }}

    function renderMissionList() {{
      missionList.innerHTML = '';

      const defaultMissionOrder = ['SpotSweep', '3x3Sweep'];
      const hiddenMissionIds = new Set(['RecordMap', 'Teleop']);
      const missions = (missionsCache || []).filter((mission) => !hiddenMissionIds.has(mission.mission_id));
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
              <button class="${{mission.mission_id === selectedMissionId ? 'selected' : 'secondary'}}" data-mission-id="${{mission.mission_id}}">Select</button>
            </div>
          `;
          item.querySelector('button').addEventListener('click', async () => {{
            setSelectedMission(mission.mission_id);
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

    async function loadMissions() {{
      const response = await fetch('/api/v1/missions', {{ cache: 'no-store' }});
      const data = await response.json();
      missionsCache = data.missions || [];
      if (!selectedMissionId) {{
        selectedMissionId = window.localStorage.getItem(missionSelectionStorageKey()) || '';
      }}
      if (
        ['RecordMap', 'Teleop'].includes(selectedMissionId) ||
        !missionsCache.some((mission) => mission.mission_id === selectedMissionId)
      ) {{
        selectedMissionId = '';
      }}
      if (!selectedMissionId) {{
        const firstPreviewable = missionsCache.find(
          (mission) => !['RecordMap', 'Teleop'].includes(mission.mission_id)
        ) || null;
        if (firstPreviewable) {{
          selectedMissionId = firstPreviewable.mission_id;
        }}
      }}
      renderMissionList();
      updateSelectedMissionLabel();
      renderSelectedMissionPreview();
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
      const response = await fetch('/api/v1/missions/upload-vda5050', {{
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

    recordRosbagToggle.addEventListener('change', () => {{
      if (!selectedMissionId) {{
        recordRosbagToggle.checked = false;
        return;
      }}
      saveRecordRosbagPreference(selectedMissionId, recordRosbagToggle.checked);
    }});

    startMissionButton.addEventListener('click', async () => {{
      if (!selectedMissionId) {{
        setBanner('error', 'Select a mission before starting it.');
        return;
      }}
      const executeResponse = await fetch(`/api/v1/missions/${{encodeURIComponent(selectedMissionId)}}/execute`, {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/json' }},
        body: JSON.stringify({{
          record_rosbag: recordRosbagToggle.checked,
          layer_overrides: layerOverridesForMission(
            missionsCache.find((mission) => mission.mission_id === selectedMissionId) || {{}}
          )
        }})
      }});
      const executeData = await executeResponse.json();
      setBanner(executeData.success ? 'ok' : 'error', executeData.message || 'Mission request completed');
      await Promise.all([loadMap(), loadMissions()]);
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
    .toggle-list {{
      display: grid;
      gap: 10px;
      margin-top: 14px;
    }}
    .toggle-item {{
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 12px;
      background: var(--panel);
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 12px;
      flex-wrap: wrap;
    }}
    .toggle-button {{
      appearance: none;
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 10px 14px;
      background: rgba(96, 100, 102, 0.72);
      color: var(--ink);
      cursor: pointer;
      font-weight: 700;
      letter-spacing: 0.04em;
      text-transform: uppercase;
    }}
    .toggle-button.enabled {{
      background: #fdca0f;
      color: #08100a;
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
        <a class="nav-link" href="/teleop">Teleop</a>
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
      <h2>Layer Toggles</h2>
      <div id="developer-selected-mission" class="muted">Selected mission: none</div>
      <div id="layer-toggle-list" class="toggle-list"></div>
    </section>

    <section class="card" style="margin-top: 18px;">
      <h2>Raw Status</h2>
      <pre id="raw-status">{{}}</pre>
    </section>
  </main>

  <script>
    const logList = document.getElementById('log-list');
    const rawStatus = document.getElementById('raw-status');
    const selectedMissionElement = document.getElementById('developer-selected-mission');
    const layerToggleList = document.getElementById('layer-toggle-list');
    const layerToggleDefinitions = [
      ['use_amr_sweeper_ros2_control', 'ROS2 Control'],
      ['use_amr_sweeper_battery', 'Battery'],
      ['use_amr_sweeper_system_info', 'System Info'],
      ['use_amr_sweeper_usb_cameras', 'USB Cameras'],
      ['use_amr_sweeper_depth_camera', 'Depth Camera'],
      ['use_amr_sweeper_imu', 'IMU'],
      ['use_amr_sweeper_gnss', 'GNSS'],
      ['use_ntrip_client', 'NTRIP Client'],
      ['use_amr_sweeper_drive_controller', 'Drive Controller'],
      ['use_amr_sweeper_tool_controller', 'Tool Controller'],
      ['use_amr_sweeper_teleop', 'Teleop'],
      ['use_amr_sweeper_sweeping_controller', 'Sweeping Controller'],
      ['use_amr_sweeper_attitude_controller', 'Attitude Controller'],
      ['use_amr_sweeper_collision_detector', 'Collision Detector'],
      ['use_amr_sweeper_safety_controller', 'Safety Controller'],
      ['use_joy_node', 'Joy Node'],
      ['use_amr_sweeper_visual_odometry', 'Visual Odometry'],
      ['use_amr_sweeper_localization', 'Localization'],
      ['use_amr_sweeper_mapping', 'Mapping'],
      ['use_amr_sweeper_navigation', 'Navigation'],
      ['auto_start_mission', 'Auto Start Mission'],
    ];
    const fallbackLayerOverrides = {{
      use_amr_sweeper_ros2_control: true,
      use_amr_sweeper_battery: true,
      use_amr_sweeper_system_info: true,
      use_amr_sweeper_usb_cameras: true,
      use_amr_sweeper_depth_camera: true,
      use_amr_sweeper_imu: true,
      use_amr_sweeper_gnss: true,
      use_ntrip_client: true,
      use_amr_sweeper_drive_controller: true,
      use_amr_sweeper_tool_controller: true,
      use_amr_sweeper_teleop: true,
      use_amr_sweeper_sweeping_controller: true,
      use_amr_sweeper_attitude_controller: true,
      use_amr_sweeper_collision_detector: true,
      use_amr_sweeper_safety_controller: true,
      use_joy_node: false,
      use_amr_sweeper_visual_odometry: false,
      use_amr_sweeper_localization: true,
      use_amr_sweeper_mapping: false,
      use_amr_sweeper_navigation: true,
      auto_start_mission: true,
    }};
    let executableMissions = [];

    function missionSelectionStorageKey() {{
      return 'amr_sweeper_selected_mission_id';
    }}

    function missionLayerOverridesStorageKey(missionId) {{
      return `amr_sweeper_layer_overrides_${{missionId}}`;
    }}

    function defaultLayerOverridesForMission(mission) {{
      return {{
        ...fallbackLayerOverrides,
        ...(mission?.profile_default_overrides || {{}}),
      }};
    }}

    function selectedMission() {{
      const selectedMissionId = window.localStorage.getItem(missionSelectionStorageKey()) || '';
      return executableMissions.find((mission) => mission.mission_id === selectedMissionId) || null;
    }}

    function layerOverridesForMission(mission) {{
      if (!mission?.mission_id) {{
        return defaultLayerOverridesForMission(mission);
      }}
      const stored = window.localStorage.getItem(missionLayerOverridesStorageKey(mission.mission_id));
      if (!stored) {{
        return defaultLayerOverridesForMission(mission);
      }}
      try {{
        const parsed = JSON.parse(stored);
        return {{
          ...defaultLayerOverridesForMission(mission),
          ...parsed,
        }};
      }} catch (_error) {{
        return defaultLayerOverridesForMission(mission);
      }}
    }}

    function saveLayerOverrides(missionId, overrides) {{
      window.localStorage.setItem(
        missionLayerOverridesStorageKey(missionId),
        JSON.stringify(overrides)
      );
    }}

    function renderLayerToggles() {{
      const mission = selectedMission();
      layerToggleList.innerHTML = '';
      if (!mission) {{
        selectedMissionElement.textContent = 'Selected mission: none';
        layerToggleList.innerHTML = '<div class="muted">Select a mission on the Missions page to edit its layer defaults here.</div>';
        return;
      }}

      selectedMissionElement.textContent = `Selected mission: ${{mission.mission_id}}`;
      const overrides = layerOverridesForMission(mission);
      for (const [key, label] of layerToggleDefinitions) {{
        const item = document.createElement('div');
        item.className = 'toggle-item';
        const enabled = Boolean(overrides[key]);
        item.innerHTML = `
          <div>
            <strong>${{label}}</strong><br>
            <span class="muted">${{enabled ? 'Enabled' : 'Disabled'}}</span>
          </div>
          <button class="toggle-button ${{enabled ? 'enabled' : ''}}" type="button">
            ${{enabled ? 'On' : 'Off'}}
          </button>
        `;
        item.querySelector('button').addEventListener('click', () => {{
          const nextOverrides = {{
            ...overrides,
            [key]: !enabled,
          }};
          saveLayerOverrides(mission.mission_id, nextOverrides);
          renderLayerToggles();
        }});
        layerToggleList.appendChild(item);
      }}
    }}

    async function loadMissions() {{
      const response = await fetch('/api/v1/missions', {{ cache: 'no-store' }});
      const data = await response.json();
      executableMissions = data.missions || [];
      renderLayerToggles();
    }}

    async function loadStatus() {{
      const response = await fetch('/api/v1/status', {{ cache: 'no-store' }});
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

    Promise.all([loadStatus(), loadMissions()]).catch(() => null);
    setInterval(loadStatus, 2000);
    window.addEventListener('storage', () => {{
      renderLayerToggles();
    }});
  </script>
</body>
</html>
"""


class MissionFrontendHttpNode(Node, MissionFrontendRenderer):
    def __init__(self) -> None:
        super().__init__("frontend_http_node")
        self._http_host = self.declare_parameter("http_host", "0.0.0.0").value
        self._http_port = int(self.declare_parameter("http_port", 8080).value)
        self._backend_socket_path = str(
            self.declare_parameter("backend_socket_path", DEFAULT_BACKEND_SOCKET_PATH).value
        )
        self._site_title = self.declare_parameter("site_title", "AMR-Sweeper").value
        self._public_base_url = self.declare_parameter(
            "public_base_url",
            "http://192.168.2.1:8080",
        ).value
        self._teleop_camera_rgb_topic = str(
            self.declare_parameter(
                "teleop_camera_rgb_topic",
                "/amr_sweeper/depth_camera/color/image_raw",
            ).value
        )
        self._http_server: ThreadingHTTPServer | None = None
        self._camera_condition = threading.Condition()
        self._camera_subscription = None
        self._camera_client_count = 0
        self._camera_latest_jpeg: bytes | None = None
        self._camera_latest_stamp = 0.0

    def start_http_server(self) -> None:
        handler = self._build_handler()
        try:
            self._http_server = MissionThreadingHTTPServer((self._http_host, self._http_port), handler)
        except OSError as exc:
            if exc.errno == errno.EADDRINUSE:
                raise RuntimeError(
                    f"HTTP listen address {self._http_host}:{self._http_port} is already in use. "
                    "Another web frontend instance may still be running."
                ) from exc
            raise
        self.get_logger().info(
            f"Mission web frontend listening on http://{self._http_host}:{self._http_port}"
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

    def add_camera_stream_client(self) -> None:
        with self._camera_condition:
            self._camera_client_count += 1
            if self._camera_subscription is None:
                self._camera_subscription = self.create_subscription(
                    Image,
                    self._teleop_camera_rgb_topic,
                    self._handle_teleop_camera_image,
                    qos_profile_sensor_data,
                )
                self.get_logger().info(
                    f"Teleop camera stream subscribed to {self._teleop_camera_rgb_topic}"
                )

    def remove_camera_stream_client(self) -> None:
        with self._camera_condition:
            self._camera_client_count = max(0, self._camera_client_count - 1)
            if self._camera_client_count == 0 and self._camera_subscription is not None:
                self.destroy_subscription(self._camera_subscription)
                self._camera_subscription = None
                self._camera_latest_jpeg = None
                self._camera_latest_stamp = 0.0
                self._camera_condition.notify_all()
                self.get_logger().info("Teleop camera stream unsubscribed; no connected clients")

    def wait_for_camera_jpeg(self, last_stamp: float, timeout_sec: float = 2.0) -> tuple[bytes | None, float]:
        deadline = time.monotonic() + timeout_sec
        with self._camera_condition:
            while self._camera_latest_stamp <= last_stamp:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None, last_stamp
                self._camera_condition.wait(timeout=remaining)
            return self._camera_latest_jpeg, self._camera_latest_stamp

    def _handle_teleop_camera_image(self, msg: Image) -> None:
        try:
            jpeg = self._encode_camera_image_to_jpeg(msg)
        except ValueError as exc:
            self.get_logger().warn(f"Skipping teleop camera frame: {exc}", throttle_duration_sec=5.0)
            return
        with self._camera_condition:
            self._camera_latest_jpeg = jpeg
            self._camera_latest_stamp = time.monotonic()
            self._camera_condition.notify_all()

    @staticmethod
    def _encode_camera_image_to_jpeg(msg: Image) -> bytes:
        if msg.width <= 0 or msg.height <= 0:
            raise ValueError("empty image dimensions")
        if msg.encoding not in {"rgb8", "bgr8"}:
            raise ValueError(f"unsupported encoding {msg.encoding!r}; expected rgb8 or bgr8")
        channels = 3
        expected_row_bytes = int(msg.width) * channels
        if msg.step < expected_row_bytes:
            raise ValueError(
                f"invalid image step {msg.step}; expected at least {expected_row_bytes}"
            )
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        required_bytes = int(msg.step) * int(msg.height)
        if raw.size < required_bytes:
            raise ValueError(
                f"image data too short ({raw.size} bytes); expected {required_bytes}"
            )
        rows = raw[:required_bytes].reshape((int(msg.height), int(msg.step)))
        image = rows[:, :expected_row_bytes].reshape((int(msg.height), int(msg.width), channels))
        if msg.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        success, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
        if not success:
            raise ValueError("JPEG encoding failed")
        return encoded.tobytes()

    def _build_handler(self):
        node = self

        class MissionFrontendRequestHandler(BaseHTTPRequestHandler):
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
                if parsed.path == "/teleop":
                    self._send_html(node.render_teleop_html())
                    return
                if parsed.path == "/api/v1/teleop/camera/stream":
                    self._send_teleop_camera_stream()
                    return
                if parsed.path.startswith("/api/v1/"):
                    self._proxy_to_backend()
                    return
                self._send_json(HTTPStatus.NOT_FOUND, {"success": False, "message": "Not found"})

            def do_POST(self) -> None:  # noqa: N802
                parsed = urllib.parse.urlparse(self.path)
                if parsed.path.startswith("/api/v1/"):
                    self._proxy_to_backend()
                    return
                self._send_json(HTTPStatus.NOT_FOUND, {"success": False, "message": "Not found"})

            def log_message(self, format: str, *args: Any) -> None:
                node.get_logger().debug(f"HTTP {self.address_string()} - {format % args}")

            def _proxy_to_backend(self) -> None:
                try:
                    backend_request = self._build_backend_request()
                    backend_response = self._exchange_backend_jsonl(backend_request)
                    status = self._backend_status(backend_response)
                    if backend_request["action"] == "DOWNLOAD_MISSION" and status.value < 400:
                        self._send_backend_download(status, backend_response)
                        return
                    public_response = dict(backend_response)
                    public_response.pop("status_code", None)
                    public_response.pop("error", None)
                    self._send_json(status, public_response)
                except ValueError as exc:
                    status = HTTPStatus.NOT_FOUND if str(exc) == "Not found" else HTTPStatus.BAD_REQUEST
                    self._send_json(
                        status,
                        {"success": False, "message": str(exc)},
                    )
                except RuntimeError as exc:
                    self._send_json(
                        HTTPStatus.BAD_GATEWAY,
                        {"success": False, "message": f"Backend IPC request failed: {exc}"},
                    )
                except OSError as exc:
                    self._send_json(
                        HTTPStatus.BAD_GATEWAY,
                        {
                            "success": False,
                            "message": f"Backend IPC request failed: {exc}",
                        },
                    )

            def _build_backend_request(self) -> dict[str, Any]:
                parsed = urllib.parse.urlparse(self.path)
                payload: dict[str, Any] = {}
                if self.command == "POST":
                    payload = self._read_json_body()

                if self.command == "GET" and parsed.path == "/api/v1/status":
                    return {"action": "GET_STATUS", "payload": {}}
                if self.command == "GET" and parsed.path == "/api/v1/missions":
                    return {"action": "LIST_MISSIONS", "payload": {}}
                if (
                    self.command == "GET"
                    and parsed.path.startswith("/api/v1/missions/")
                    and parsed.path.endswith("/download")
                ):
                    mission_segment = parsed.path[len("/api/v1/missions/"):-len("/download")]
                    mission_id = urllib.parse.unquote(mission_segment.rstrip("/"))
                    if not mission_id:
                        raise ValueError("mission_id is required")
                    return {"action": "DOWNLOAD_MISSION", "payload": {"mission_id": mission_id}}
                if self.command == "GET" and parsed.path == "/api/v1/schedule":
                    query = urllib.parse.parse_qs(parsed.query)
                    return {"action": "GET_SCHEDULE", "payload": {"week": query.get("week", [""])[0]}}
                if self.command == "GET" and parsed.path == "/api/v1/map-data":
                    return {"action": "GET_MAP_DATA", "payload": {}}
                if self.command == "GET" and parsed.path == "/api/v1/record-map":
                    return {"action": "GET_RECORD_MAP", "payload": {}}

                if (
                    self.command == "POST"
                    and parsed.path.startswith("/api/v1/missions/")
                    and parsed.path.endswith("/execute")
                ):
                    mission_segment = parsed.path[len("/api/v1/missions/"):-len("/execute")]
                    mission_id = urllib.parse.unquote(mission_segment.rstrip("/"))
                    if not mission_id:
                        raise ValueError("mission_id is required")
                    request_payload = dict(payload)
                    request_payload["mission_id"] = mission_id
                    return {"action": "EXECUTE_MISSION", "payload": request_payload}

                post_routes = {
                    "/api/v1/missions/upload-vda5050": "UPLOAD_VDA5050_MISSION",
                    "/api/v1/mission/stop": "STOP_MISSION",
                    "/api/v1/system/reinitialize": "REINITIALIZE_SYSTEM",
                    "/api/v1/safety/clear": "CLEAR_SAFETY_STOP",
                    "/api/v1/safety/stop": "TRIGGER_SAFETY_STOP",
                    "/api/v1/teleop/start": "START_TELEOP",
                    "/api/v1/teleop/stop": "STOP_TELEOP",
                    "/api/v1/teleop/command": "SEND_TELEOP_COMMAND",
                    "/api/v1/teleop/lights": "SET_TELEOP_LIGHTS",
                    "/api/v1/record-map/start": "START_RECORD_MAP",
                    "/api/v1/record-map/stop": "STOP_RECORD_MAP",
                    "/api/v1/record-map/save-mission": "SAVE_RECORDED_MISSION",
                    "/api/v1/schedule/entry": "SAVE_SCHEDULE_ENTRY",
                    "/api/v1/schedule/entry/delete": "DELETE_SCHEDULE_ENTRY",
                }
                action = post_routes.get(parsed.path) if self.command == "POST" else None
                if action is None:
                    raise ValueError("Not found")
                return {"action": action, "payload": payload}

            def _exchange_backend_jsonl(self, request: dict[str, Any]) -> dict[str, Any]:
                encoded = json.dumps(request, separators=(",", ":")).encode("utf-8") + b"\n"
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
                    connection.settimeout(20.0)
                    connection.connect(node._backend_socket_path)
                    connection.sendall(encoded)
                    connection.shutdown(socket.SHUT_WR)
                    raw_response = self._read_backend_line(connection)
                try:
                    decoded = json.loads(raw_response.decode("utf-8").strip())
                except UnicodeDecodeError as exc:
                    raise RuntimeError(f"Backend IPC response was not valid UTF-8: {exc}") from exc
                except json.JSONDecodeError as exc:
                    raise RuntimeError(f"Invalid backend IPC JSON response: {exc}") from exc
                if not isinstance(decoded, dict):
                    raise RuntimeError("Backend IPC response must be a JSON object")
                return decoded

            def _read_backend_line(self, connection: socket.socket) -> bytes:
                chunks: list[bytes] = []
                total_size = 0
                while total_size <= 1024 * 1024:
                    chunk = connection.recv(4096)
                    if not chunk:
                        break
                    newline_index = chunk.find(b"\n")
                    if newline_index >= 0:
                        chunks.append(chunk[:newline_index])
                        return b"".join(chunks)
                    chunks.append(chunk)
                    total_size += len(chunk)
                if total_size > 1024 * 1024:
                    raise RuntimeError("Backend IPC response exceeded maximum size")
                response = b"".join(chunks)
                if not response:
                    raise RuntimeError("Backend IPC response was empty")
                return response

            def _read_json_body(self) -> dict[str, Any]:
                body = self._read_body()
                if not body:
                    return {}
                try:
                    decoded = json.loads(body.decode("utf-8"))
                except UnicodeDecodeError as exc:
                    raise ValueError(f"Request body was not valid UTF-8: {exc}") from exc
                except json.JSONDecodeError as exc:
                    raise ValueError(f"Invalid JSON body: {exc}") from exc
                if not isinstance(decoded, dict):
                    raise ValueError("JSON body must be an object")
                return decoded

            @staticmethod
            def _backend_status(response: dict[str, Any]) -> HTTPStatus:
                try:
                    default_status = 200 if response.get("success", True) else 502
                    return HTTPStatus(int(response.get("status_code", default_status)))
                except (TypeError, ValueError):
                    return HTTPStatus.BAD_GATEWAY

            def _send_backend_download(self, status: HTTPStatus, response: dict[str, Any]) -> None:
                body = str(response.get("body", "")).encode("utf-8")
                filename = str(response.get("filename", "mission.json")).replace('"', "")
                content_type = str(response.get("content_type", "application/json; charset=utf-8"))
                self._send_bytes(
                    status,
                    body,
                    {
                        "Content-Type": content_type,
                        "Cache-Control": "no-store",
                        "Content-Length": str(len(body)),
                        "Content-Disposition": f'attachment; filename="{filename}"',
                    },
                )

            def _read_body(self) -> bytes:
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                except ValueError:
                    return b""
                if length <= 0:
                    return b""
                return self.rfile.read(length)

            def _send_html(self, body: str) -> None:
                encoded = body.encode("utf-8")
                self._send_bytes(
                    HTTPStatus.OK,
                    encoded,
                    {
                        "Content-Type": "text/html; charset=utf-8",
                        "Content-Length": str(len(encoded)),
                    },
                )

            def _send_json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
                encoded = json.dumps(payload).encode("utf-8")
                self._send_bytes(
                    status,
                    encoded,
                    {
                        "Content-Type": "application/json; charset=utf-8",
                        "Cache-Control": "no-store",
                        "Content-Length": str(len(encoded)),
                    },
                )

            def _send_teleop_camera_stream(self) -> None:
                boundary = "teleop-camera-frame"
                node.add_camera_stream_client()
                last_stamp = 0.0
                try:
                    self.send_response(HTTPStatus.OK)
                    self.send_header(
                        "Content-Type",
                        f"multipart/x-mixed-replace; boundary={boundary}",
                    )
                    self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                    self.send_header("Pragma", "no-cache")
                    self.send_header("Connection", "close")
                    self.end_headers()
                    while True:
                        jpeg, last_stamp = node.wait_for_camera_jpeg(last_stamp)
                        if jpeg is None:
                            continue
                        part_headers = (
                            f"--{boundary}\r\n"
                            "Content-Type: image/jpeg\r\n"
                            f"Content-Length: {len(jpeg)}\r\n\r\n"
                        ).encode("ascii")
                        self.wfile.write(part_headers)
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                except OSError as exc:
                    if not self._is_client_disconnect(exc):
                        raise
                finally:
                    node.remove_camera_stream_client()

            def _send_bytes(
                self,
                status: HTTPStatus,
                body: bytes,
                headers: dict[str, str],
            ) -> None:
                self.send_response(status)
                for key, value in headers.items():
                    self.send_header(key, value)
                try:
                    self.end_headers()
                    self.wfile.write(body)
                except OSError as exc:
                    if self._is_client_disconnect(exc):
                        node.get_logger().debug(
                            f"HTTP client disconnected before response completed: {exc}"
                        )
                        return
                    raise

            @staticmethod
            def _is_client_disconnect(exc: OSError) -> bool:
                return isinstance(exc, (BrokenPipeError, ConnectionResetError, socket.timeout)) or (
                    exc.errno in {errno.EPIPE, errno.ECONNRESET, errno.ECONNABORTED}
                )

        return MissionFrontendRequestHandler


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = MissionFrontendHttpNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    server_thread: threading.Thread | None = None

    try:
        node.start_http_server()
        server_thread = threading.Thread(target=node.serve_forever, name="mission_http_frontend", daemon=True)
        server_thread.start()
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"Mission frontend HTTP startup failed: {exc}")
        return 1
    finally:
        executor.shutdown()
        try:
            executor.remove_node(node)
        except (KeyboardInterrupt, RuntimeError, AttributeError):
            pass
        try:
            node.stop_http_server()
        except RuntimeError:
            pass
        if server_thread is not None:
            server_thread.join(timeout=2.0)
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError, AttributeError):
            pass
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
