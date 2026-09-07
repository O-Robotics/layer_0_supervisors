#!/usr/bin/env python3
#
# Copyright 2026 O-Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import errno
from html import escape
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import socket
import threading
import time
from typing import Any
import urllib.parse

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


DEFAULT_BACKEND_SOCKET_PATH = "/tmp/amr_sweeper_interface_backend.sock"


def gaussian_splat_artifact_allowed_roots(cwd: Path | None = None) -> list[Path]:
    root = Path.cwd() if cwd is None else cwd
    return [
        (root / "missions" / "logs").resolve(),
        (root / "missions" / "maps").resolve(),
        (root / "missions" / "simulations").resolve(),
        Path("/tmp").resolve(),
    ]


class MissionThreadingHTTPServer(ThreadingHTTPServer):

    allow_reuse_address = True
    daemon_threads = True


class MissionFrontendRenderer:

    def _shared_topbar_css(self, absolute: bool = False) -> str:
        position = "absolute" if absolute else "fixed"
        return f"""
    .app-topbar {{
      position: {position};
      top: calc(10px + env(safe-area-inset-top));
      left: calc(10px + env(safe-area-inset-left));
      right: calc(10px + env(safe-area-inset-right));
      z-index: 700;
      display: grid;
      grid-template-columns: 46px minmax(0, 1fr) minmax(64px, auto) minmax(54px, auto) minmax(0, auto);
      gap: 8px;
      align-items: center;
      min-height: 52px;
      padding: 5px;
      border: 1px solid var(--line);
      border-radius: 18px;
      background: rgba(24, 27, 29, 0.84);
      box-shadow: 0 16px 36px rgba(0, 0, 0, 0.32);
      backdrop-filter: blur(8px);
    }}
    .topbar-title {{
      min-width: 0;
      overflow: hidden;
    }}
    .topbar-title h1 {{
      overflow: hidden;
      margin: 0;
      color: var(--accent);
      text-overflow: ellipsis;
      white-space: nowrap;
      font-size: 1.08rem;
    }}
    .topbar-substatus {{
      display: block;
      overflow: hidden;
      color: var(--muted);
      text-overflow: ellipsis;
      white-space: nowrap;
      font-size: 0.72rem;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.04em;
    }}
    .topbar-status, .battery-button {{
      display: inline-flex;
      align-items: center;
      justify-content: center;
      min-width: 0;
      min-height: 34px;
      max-width: 118px;
      border-radius: 999px;
      padding: 6px 10px;
      overflow: hidden;
      background: rgba(253, 202, 15, 0.12);
      color: var(--accent);
      font-weight: 700;
      font-size: 0.74rem;
      text-overflow: ellipsis;
      text-transform: uppercase;
      white-space: nowrap;
    }}
    .topbar-status.connected {{
      background: rgba(34, 197, 94, 0.13);
      color: #86efac;
    }}
    .topbar-status.disconnected {{
      background: rgba(239, 68, 68, 0.13);
      color: #fca5a5;
    }}
    .battery-button {{
      border: 0;
      cursor: pointer;
      font: inherit;
      letter-spacing: 0.02em;
    }}
    .topbar-action {{
      display: flex;
      gap: 6px;
      justify-content: flex-end;
      min-width: 0;
    }}
    .topbar-action:empty {{
      display: none;
    }}
    .battery-popover {{
      position: fixed;
      top: calc(70px + env(safe-area-inset-top));
      right: calc(10px + env(safe-area-inset-right));
      z-index: 920;
      display: none;
      width: min(92vw, 320px);
      padding: 12px;
      border: 1px solid var(--line);
      border-radius: 14px;
      background: rgba(42, 46, 48, 0.98);
      box-shadow: 0 18px 42px rgba(0, 0, 0, 0.36);
      backdrop-filter: blur(8px);
    }}
    .battery-popover.open {{
      display: grid;
      gap: 8px;
    }}
    .battery-detail-row {{
      display: flex;
      justify-content: space-between;
      gap: 12px;
      border-bottom: 1px solid rgba(253, 202, 15, 0.12);
      padding-bottom: 7px;
      color: var(--muted);
      font-size: 0.9rem;
    }}
    .battery-detail-row strong {{
      color: var(--ink);
      text-align: right;
    }}
    .icon-button {{
      width: 46px;
      padding: 0;
      color: var(--ink);
      background: rgba(18, 20, 21, 0.76);
      border: 1px solid var(--line);
      font-size: 1.24rem;
      letter-spacing: 0;
      text-transform: none;
    }}
    @media (max-width: 560px) {{
      .app-topbar {{
        grid-template-columns: 46px minmax(0, 1fr) minmax(54px, auto) minmax(48px, auto) minmax(0, auto);
        gap: 5px;
      }}
      .topbar-status, .battery-button {{
        max-width: 76px;
        padding: 6px 8px;
        font-size: 0.68rem;
      }}
      .topbar-action button {{
        min-width: 0;
        padding-left: 10px;
        padding-right: 10px;
        font-size: 0.72rem;
      }}
    }}
"""

    def _render_topbar(self, page_title: str, action_html: str = "") -> str:
        escaped_title = escape(page_title)
        return f"""<header class="app-topbar" aria-label="{escaped_title} controls" data-shared-topbar="true">
    <button id="open-nav-button" class="icon-button" type="button" aria-label="Open navigation">&#9776;</button>
    <div class="topbar-title">
      <h1>{escaped_title}</h1>
      <span id="topbar-page-state" class="topbar-substatus">Loading</span>
    </div>
    <span id="topbar-connection" class="topbar-status">Connecting</span>
    <button id="topbar-battery-button" class="battery-button" type="button" aria-expanded="false" aria-controls="battery-popover">--</button>
    <div class="topbar-action">{action_html}</div>
  </header>
  <section id="battery-popover" class="battery-popover" aria-label="Battery details">
    <div class="battery-detail-row"><span>Battery</span><strong id="battery-detail-percent">--</strong></div>
    <div class="battery-detail-row"><span>Voltage</span><strong id="battery-detail-voltage">--</strong></div>
    <div class="battery-detail-row"><span>Current</span><strong id="battery-detail-current">--</strong></div>
    <div class="battery-detail-row"><span>Charge</span><strong id="battery-detail-charge">--</strong></div>
    <div class="battery-detail-row"><span>Temperature</span><strong id="battery-detail-temperature">--</strong></div>
    <div class="battery-detail-row"><span>Runtime</span><strong id="battery-detail-runtime">--</strong></div>
    <div class="battery-detail-row"><span>Status</span><strong id="battery-detail-status">--</strong></div>
  </section>"""

    def _shared_topbar_js(self) -> str:
        return """
    const topbarConnection = document.getElementById('topbar-connection');
    const topbarPageState = document.getElementById('topbar-page-state');
    const topbarBatteryButton = document.getElementById('topbar-battery-button');
    const batteryPopover = document.getElementById('battery-popover');

    function finiteNumber(value) {
      const number = Number(value);
      return Number.isFinite(number) ? number : null;
    }

    function formatPercent(value) {
      const number = finiteNumber(value);
      return number === null ? '--' : `${Math.round(number * 100)}%`;
    }

    function formatFixed(value, digits, unit) {
      const number = finiteNumber(value);
      return number === null ? '--' : `${number.toFixed(digits)} ${unit}`;
    }

    function formatCharge(battery) {
      const charge = finiteNumber(battery?.charge);
      const capacity = finiteNumber(battery?.capacity);
      if (charge === null && capacity === null) {
        return '--';
      }
      if (charge !== null && capacity !== null && capacity > 0) {
        return `${charge.toFixed(1)} / ${capacity.toFixed(1)} Ah`;
      }
      return charge !== null ? `${charge.toFixed(1)} Ah` : `${capacity.toFixed(1)} Ah capacity`;
    }

    function formatRuntime(battery) {
      const charge = finiteNumber(battery?.charge);
      const current = finiteNumber(battery?.current);
      if (charge === null || charge <= 0 || current === null || current >= 0) {
        return '--';
      }
      const hours = charge / Math.abs(current);
      if (!Number.isFinite(hours) || hours <= 0) {
        return '--';
      }
      const minutes = Math.round(hours * 60);
      return minutes >= 60 ? `~${Math.floor(minutes / 60)}h ${minutes % 60}m` : `~${minutes}m`;
    }

    function batteryStatusLabel(battery) {
      if (!battery) {
        return '--';
      }
      const present = battery.present === false ? 'Not present' : 'Present';
      const status = finiteNumber(battery.power_supply_status);
      const labels = {
        1: 'Unknown',
        2: 'Charging',
        3: 'Discharging',
        4: 'Not charging',
        5: 'Full',
      };
      return `${present}${status === null ? '' : ` | ${labels[status] || `Status ${status}`}`}`;
    }

    function updateTopbarFromStatus(data, pageState) {
      const fsm = data?.fsm_status || data?.fsm_state || {};
      const display = data?.fsm_display || {};
      const battery = data?.battery || {};
      if (topbarConnection) {
        topbarConnection.textContent = 'Connected';
        topbarConnection.classList.add('connected');
        topbarConnection.classList.remove('disconnected');
      }
      if (topbarPageState) {
        topbarPageState.textContent = pageState || display.current_state || fsm.current_state || 'Ready';
      }
      if (topbarBatteryButton) {
        topbarBatteryButton.textContent = formatPercent(battery.percentage);
      }
      const percent = document.getElementById('battery-detail-percent');
      if (percent) {
        percent.textContent = formatPercent(battery.percentage);
        document.getElementById('battery-detail-voltage').textContent = formatFixed(battery.voltage, 2, 'V');
        document.getElementById('battery-detail-current').textContent = formatFixed(battery.current, 2, 'A');
        document.getElementById('battery-detail-charge').textContent = formatCharge(battery);
        document.getElementById('battery-detail-temperature').textContent = formatFixed(battery.temperature, 1, 'C');
        document.getElementById('battery-detail-runtime').textContent = formatRuntime(battery);
        document.getElementById('battery-detail-status').textContent = batteryStatusLabel(battery);
      }
    }

    function markTopbarDisconnected() {
      if (topbarConnection) {
        topbarConnection.textContent = 'Disconnected';
        topbarConnection.classList.add('disconnected');
        topbarConnection.classList.remove('connected');
      }
    }

    function closeBatteryPopover() {
      if (batteryPopover) {
        batteryPopover.classList.remove('open');
      }
      if (topbarBatteryButton) {
        topbarBatteryButton.setAttribute('aria-expanded', 'false');
      }
    }

    if (topbarBatteryButton && batteryPopover) {
      topbarBatteryButton.addEventListener('click', (event) => {
        event.stopPropagation();
        const open = !batteryPopover.classList.contains('open');
        batteryPopover.classList.toggle('open', open);
        topbarBatteryButton.setAttribute('aria-expanded', String(open));
      });
      document.addEventListener('click', (event) => {
        if (!batteryPopover.contains(event.target) && event.target !== topbarBatteryButton) {
          closeBatteryPopover();
        }
      });
      document.addEventListener('keydown', (event) => {
        if (event.key === 'Escape') {
          closeBatteryPopover();
        }
      });
    }
"""

    def render_index_html(self) -> str:
        title = escape(self._site_title)
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
      padding: calc(82px + env(safe-area-inset-top)) 24px 24px;
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
    .app-topbar {{
      position: fixed;
      top: calc(10px + env(safe-area-inset-top));
      left: calc(10px + env(safe-area-inset-left));
      right: calc(10px + env(safe-area-inset-right));
      z-index: 700;
      display: grid;
      grid-template-columns: 46px minmax(0, 1fr) auto auto;
      gap: 10px;
      align-items: center;
      min-height: 52px;
      padding: 5px;
      border: 1px solid var(--line);
      border-radius: 18px;
      background: rgba(24, 27, 29, 0.84);
      box-shadow: 0 16px 36px rgba(0, 0, 0, 0.32);
      backdrop-filter: blur(8px);
    }}
    .app-topbar h1 {{
      overflow: hidden;
      margin: 0;
      text-overflow: ellipsis;
      white-space: nowrap;
      font-size: 1.08rem;
    }}
    .topbar-status {{
      display: inline-flex;
      align-items: center;
      min-height: 34px;
      border-radius: 999px;
      padding: 6px 10px;
      background: rgba(253, 202, 15, 0.12);
      color: var(--accent);
      font-weight: 700;
      font-size: 0.78rem;
      text-transform: uppercase;
    }}
    .icon-button {{
      width: 46px;
      padding: 0;
      color: var(--ink);
      background: rgba(18, 20, 21, 0.76);
      border: 1px solid var(--line);
      font-size: 1.24rem;
      letter-spacing: 0;
      text-transform: none;
    }}
    .drawer-backdrop {{
      position: fixed;
      inset: 0;
      z-index: 850;
      display: none;
      background: rgba(0, 0, 0, 0.42);
    }}
    .drawer-backdrop.show {{ display: block; }}
    .nav {{
      position: fixed;
      top: 0;
      bottom: 0;
      left: 0;
      z-index: 900;
      display: grid;
      align-content: start;
      gap: 9px;
      width: min(90vw, 390px);
      padding: calc(18px + env(safe-area-inset-top)) 18px calc(18px + env(safe-area-inset-bottom));
      overflow-y: auto;
      background: rgba(42, 46, 48, 0.96);
      border-right: 1px solid var(--line);
      box-shadow: 0 18px 42px rgba(0, 0, 0, 0.36);
      transform: translateX(-105%);
      transition: transform 180ms ease;
      backdrop-filter: blur(8px);
    }}
    .nav.open {{
      transform: translateX(0);
    }}
    .nav-link {{
      display: block;
      text-decoration: none;
      color: var(--ink);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      background: rgba(18, 20, 21, 0.58);
      font-size: 0.92rem;
      text-transform: none;
      letter-spacing: 0;
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
      main {{ padding: calc(124px + env(safe-area-inset-top)) 14px 18px; }}
      .status-card {{
        grid-column: span 1;
      }}
    }}
    {self._shared_topbar_css()}
  </style>
</head>
<body>
  {self._render_topbar("Dashboard")}
  <div id="drawer-backdrop" class="drawer-backdrop"></div>
  <nav id="nav-drawer" class="nav" aria-label="Application navigation">
    <h2>O-ROBOTICS</h2>
    <div class="live-strip">
      <div class="live-pill">
        <span id="live-dot" class="live-dot"></span>
        <span id="live-status" class="live-status-text">CONNECTING</span>
      </div>
    </div>
    <a class="nav-link" href="/">Dashboard</a>
    <a class="nav-link" href="/calendar">Calendar</a>
    <a class="nav-link" href="/missions">Missions</a>
    <a class="nav-link" href="/teleop">Teleop</a>
    <a class="nav-link" href="/developer">Developer</a>
    <button id="drawer-safety-stop-button" class="stop" type="button">SAFETY STOP</button>
  </nav>
  <main>
    <section class="hero">
      <h1>AMR-Sweeper</h1>
      <div id="banner" class="banner"></div>
    </section>

    <section class="grid">
      <article id="system-status-card" class="card safety-card status-card">
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
          <section id="safety-control-panel" class="status-panel safety-card">
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
    {self._shared_topbar_js()}
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
      updateTopbarFromStatus(data, transitionInProgress ? 'Running' : currentState);

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

      const safetyCard = document.getElementById('safety-control-panel');
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
    document.getElementById('drawer-safety-stop-button').addEventListener('click', () => {{
      document.getElementById('safety-stop-button').click();
    }});
    document.getElementById('open-nav-button').addEventListener('click', () => {{
      closeBatteryPopover();
      document.getElementById('nav-drawer').classList.add('open');
      document.getElementById('drawer-backdrop').classList.add('show');
    }});
    document.getElementById('drawer-backdrop').addEventListener('click', () => {{
      document.getElementById('nav-drawer').classList.remove('open');
      document.getElementById('drawer-backdrop').classList.remove('show');
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
        markTopbarDisconnected();
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
        markTopbarDisconnected();
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
      padding: calc(82px + env(safe-area-inset-top)) 24px 24px;
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
    .app-topbar {{
      position: fixed;
      top: calc(10px + env(safe-area-inset-top));
      left: calc(10px + env(safe-area-inset-left));
      right: calc(10px + env(safe-area-inset-right));
      z-index: 700;
      display: grid;
      grid-template-columns: 46px minmax(0, 1fr) auto;
      gap: 10px;
      align-items: center;
      min-height: 52px;
      padding: 5px;
      border: 1px solid var(--line);
      border-radius: 18px;
      background: rgba(24, 27, 29, 0.84);
      box-shadow: 0 16px 36px rgba(0, 0, 0, 0.32);
      backdrop-filter: blur(8px);
    }}
    .app-topbar h1 {{
      overflow: hidden;
      margin: 0;
      color: var(--accent);
      text-overflow: ellipsis;
      white-space: nowrap;
      font-size: 1.08rem;
    }}
    .topbar-status {{
      display: inline-flex;
      align-items: center;
      min-height: 34px;
      border-radius: 999px;
      padding: 6px 10px;
      background: rgba(253, 202, 15, 0.12);
      color: var(--accent);
      font-weight: 700;
      font-size: 0.78rem;
      text-transform: uppercase;
    }}
    .icon-button {{
      width: 46px;
      padding: 0;
      color: var(--ink);
      background: rgba(18, 20, 21, 0.76);
      border: 1px solid var(--line);
      font-size: 1.24rem;
      letter-spacing: 0;
      text-transform: none;
    }}
    .drawer-backdrop {{
      position: fixed;
      inset: 0;
      z-index: 850;
      display: none;
      background: rgba(0, 0, 0, 0.42);
    }}
    .drawer-backdrop.show {{ display: block; }}
    .nav {{
      position: fixed;
      top: 0;
      bottom: 0;
      left: 0;
      z-index: 900;
      display: grid;
      align-content: start;
      gap: 9px;
      width: min(90vw, 390px);
      padding: calc(18px + env(safe-area-inset-top)) 18px calc(18px + env(safe-area-inset-bottom));
      overflow-y: auto;
      background: rgba(42, 46, 48, 0.96);
      border-right: 1px solid var(--line);
      box-shadow: 0 18px 42px rgba(0, 0, 0, 0.36);
      transform: translateX(-105%);
      transition: transform 180ms ease;
      backdrop-filter: blur(8px);
    }}
    .nav.open {{
      transform: translateX(0);
    }}
    .nav-link {{
      display: block;
      text-decoration: none;
      color: var(--ink);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      background: rgba(18, 20, 21, 0.58);
      font-size: 0.92rem;
      text-transform: none;
      letter-spacing: 0;
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
    .banner {{
      margin-top: 12px;
      padding: 12px 14px;
      border-radius: 12px;
      display: none;
      font-weight: 700;
    }}
    .banner.show {{ display: block; }}
    .banner.ok {{ background: rgba(15, 118, 110, 0.18); color: var(--accent-strong); }}
    .banner.error {{ background: rgba(185, 28, 28, 0.18); color: var(--safety); }}
    .banner.warn {{ background: rgba(180, 83, 9, 0.18); color: var(--accent); }}
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
      main {{ padding: calc(82px + env(safe-area-inset-top)) 14px 18px; }}
      .toolbar {{
        align-items: flex-start;
      }}
      .editor-layout {{
        grid-template-columns: 1fr;
      }}
      .form-grid {{
        grid-template-columns: 1fr;
      }}
      .form-grid .span-2 {{
        grid-column: span 1;
      }}
    }}
    .calendar-card-header {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      cursor: pointer;
    }}
    .calendar-card-header button {{
      min-height: 36px;
      padding: 8px 12px;
    }}
    .compact-week-grid {{
      display: grid;
      grid-template-columns: repeat(7, minmax(82px, 1fr));
      gap: 8px;
      margin-top: 12px;
    }}
    .compact-day {{
      min-height: 118px;
      border: 1px solid var(--line);
      border-radius: 10px;
      padding: 9px;
      background: var(--panel);
      overflow: hidden;
    }}
    .compact-day strong {{
      display: block;
      color: var(--accent);
      margin-bottom: 7px;
      font-size: 0.82rem;
      text-transform: uppercase;
    }}
    .compact-event {{
      display: block;
      margin-top: 5px;
      border-radius: 7px;
      padding: 5px 6px;
      overflow: hidden;
      background: rgba(253, 202, 15, 0.18);
      color: var(--ink);
      text-overflow: ellipsis;
      white-space: nowrap;
      font-size: 0.76rem;
    }}
    .calendar-detail {{
      display: none;
      margin-top: 14px;
    }}
    .calendar-card.expanded .calendar-detail {{
      display: block;
    }}
    @media (max-width: 700px) {{
      .compact-week-grid {{
        grid-template-columns: repeat(7, minmax(64px, 1fr));
        overflow-x: auto;
      }}
      .compact-day {{
        min-height: 94px;
        padding: 7px;
      }}
      .compact-event {{
        font-size: 0.68rem;
      }}
    }}
    {self._shared_topbar_css()}
  </style>
</head>
<body>
  {self._render_topbar("Calendar")}
  <div id="drawer-backdrop" class="drawer-backdrop"></div>
  <nav id="nav-drawer" class="nav" aria-label="Application navigation">
    <h2>O-ROBOTICS</h2>
    <div class="muted">Schedule planning</div>
    <a class="nav-link" href="/">Dashboard</a>
    <a class="nav-link" href="/calendar">Calendar</a>
    <a class="nav-link" href="/missions">Missions</a>
    <a class="nav-link" href="/teleop">Teleop</a>
    <a class="nav-link" href="/developer">Developer</a>
  </nav>
  <main>
    <section class="card">
      <h1>Calendar</h1>
      <div class="muted">View the active schedule as a weekly planner with full 24-hour day lanes.</div>
      <div id="banner" class="banner" role="status" aria-live="polite"></div>
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
    <section id="calendar-card" class="card calendar-card" aria-expanded="false">
      <div id="calendar-card-header" class="calendar-card-header">
        <div>
          <h2>Week Overview</h2>
          <div class="muted">Tap to expand the 24-hour planner.</div>
        </div>
        <button id="calendar-expand-button" type="button" aria-label="Expand calendar">Expand</button>
      </div>
      <div id="compact-calendar-grid" class="compact-week-grid"></div>
      <div class="calendar-detail">
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
            <input id="entry-gaussian-capture" type="checkbox">
            Gaussian capture
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
  </main>
  <script>
    {self._shared_topbar_js()}
    let activeWeek = '';
    let activeScheduleData = null;
    const weekdayNames = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'];
    const hourHeight = 64;
    const banner = document.getElementById('banner');
    const navDrawer = document.getElementById('nav-drawer');
    const drawerBackdrop = document.getElementById('drawer-backdrop');
    const calendarCard = document.getElementById('calendar-card');
    const calendarExpandButton = document.getElementById('calendar-expand-button');

    function setBanner(kind, message) {{
      banner.className = `banner show ${{kind}}`;
      banner.textContent = message;
      window.setTimeout(() => {{
        banner.className = 'banner';
        banner.textContent = '';
      }}, 5000);
    }}

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
      document.getElementById('entry-gaussian-capture').checked = false;
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
      document.getElementById('entry-gaussian-capture').checked = Boolean(entry.gaussian_capture);
      document.getElementById('entry-description').value = entry.description || '';
    }}

    async function postJson(path, body) {{
      const response = await fetch(path, {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/json' }},
        body: JSON.stringify(body || {{}})
      }});
      const data = await response.json();
      if (!response.ok) {{
        throw new Error(data.message || `${{path}} failed with HTTP ${{response.status}}`);
      }}
      return data;
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
          <div class="muted">${{entry.schedule_type || 'WORK'}}${{entry.mission_id ? ` | Mission: ${{entry.mission_id}}` : ''}}${{entry.robot_id ? ` | Robot: ${{entry.robot_id}}` : ''}}${{entry.gaussian_capture ? ' | Gaussian capture' : ''}}</div>
          <div class="entry-actions">
            <button class="secondary" type="button">Edit</button>
            <button class="danger" type="button">Delete</button>
          </div>
        `;
        const [editButton, deleteButton] = card.querySelectorAll('button');
        editButton.addEventListener('click', () => populateEntryForm(entry));
        deleteButton.addEventListener('click', async () => {{
          if (!window.confirm('Delete this planned schedule entry?')) {{
            return;
          }}
          const result = await postJson('/api/v1/schedule/entry/delete', {{ uid: entry.uid }});
          if (!result.success) {{
            setBanner('error', result.message || 'Failed to delete schedule entry');
            return;
          }}
          setBanner('ok', result.message || 'Schedule entry deleted');
          await loadCalendar(activeWeek);
        }});
        list.appendChild(card);
      }}
    }}

    function renderCompactCalendar(data) {{
      const grid = document.getElementById('compact-calendar-grid');
      grid.innerHTML = '';
      const weekStart = parseLocalDate(data.week_start);
      const eventsByDay = Array.from({{ length: 7 }}, () => []);
      for (const event of [...(data.planned_events || []), ...(data.actual_events || [])]) {{
        const start = parseLocalDateTime(event.start_local);
        const dayIndex = Math.floor((startOfDay(start).getTime() - weekStart.getTime()) / 86400000);
        if (dayIndex >= 0 && dayIndex <= 6) {{
          eventsByDay[dayIndex].push(event);
        }}
      }}
      for (let index = 0; index < 7; index += 1) {{
        const current = new Date(weekStart);
        current.setDate(weekStart.getDate() + index);
        const day = document.createElement('div');
        day.className = 'compact-day';
        day.innerHTML = `<strong>${{weekdayNames[index]}} ${{String(current.getDate()).padStart(2, '0')}}.${{String(current.getMonth() + 1).padStart(2, '0')}}</strong>`;
        const dayEvents = eventsByDay[index].slice(0, 3);
        if (dayEvents.length === 0) {{
          day.insertAdjacentHTML('beforeend', '<span class="muted">Clear</span>');
        }}
        for (const event of dayEvents) {{
          const start = parseLocalDateTime(event.start_local);
          day.insertAdjacentHTML(
            'beforeend',
            `<span class="compact-event">${{formatLocalTime(start)}} ${{event.summary || event.schedule_type || 'Event'}}</span>`
          );
        }}
        if (eventsByDay[index].length > dayEvents.length) {{
          day.insertAdjacentHTML('beforeend', `<span class="compact-event">+${{eventsByDay[index].length - dayEvents.length}} more</span>`);
        }}
        grid.appendChild(day);
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
      renderCompactCalendar(data);

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
    document.getElementById('open-nav-button').addEventListener('click', () => {{
      closeBatteryPopover();
      navDrawer.classList.add('open');
      drawerBackdrop.classList.add('show');
    }});
    drawerBackdrop.addEventListener('click', () => {{
      navDrawer.classList.remove('open');
      drawerBackdrop.classList.remove('show');
    }});

    function setCalendarExpanded(expanded) {{
      calendarCard.classList.toggle('expanded', expanded);
      calendarCard.setAttribute('aria-expanded', String(expanded));
      calendarExpandButton.textContent = expanded ? 'Collapse' : 'Expand';
      calendarExpandButton.setAttribute('aria-label', expanded ? 'Collapse calendar' : 'Expand calendar');
    }}
    document.getElementById('calendar-card-header').addEventListener('click', () => {{
      setCalendarExpanded(!calendarCard.classList.contains('expanded'));
    }});
    calendarExpandButton.addEventListener('click', (event) => {{
      event.stopPropagation();
      setCalendarExpanded(!calendarCard.classList.contains('expanded'));
    }});

    async function refreshTopbarStatus() {{
      try {{
        const response = await fetch('/api/v1/status', {{ cache: 'no-store' }});
        updateTopbarFromStatus(await response.json(), 'Schedule');
      }} catch (_error) {{
        markTopbarDisconnected();
      }}
    }}

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
        gaussian_capture: document.getElementById('entry-gaussian-capture').checked,
        description: document.getElementById('entry-description').value,
      }};
      const result = await postJson('/api/v1/schedule/entry', payload);
      if (!result.success) {{
        setBanner('error', result.message || 'Failed to save schedule entry');
        return;
      }}
      resetEntryForm();
      setBanner('ok', result.message || 'Schedule entry saved');
      await loadCalendar(activeWeek || toIsoWeekString(new Date()));
    }});

    document.getElementById('reset-entry').addEventListener('click', () => {{
      resetEntryForm();
    }});

    loadCalendar(toIsoWeekString(new Date())).then(() => {{
      resetEntryForm();
    }});
    refreshTopbarStatus();
    setInterval(refreshTopbarStatus, 2000);
  </script>
</body>
</html>
"""

    def render_record_map_html(self) -> str:
        return self.render_map_html()

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
      --card-strong: rgba(42, 46, 48, 0.98);
      --panel: rgba(88, 92, 94, 0.48);
      --ink: #f5f1df;
      --muted: #c4bb98;
      --accent: #fdca0f;
      --accent-strong: #ffe06b;
      --danger: #ff7b5c;
      --line: rgba(253, 202, 15, 0.22);
      --gold: #fdca0f;
      --surface: rgba(30, 34, 36, 0.86);
      --sheet: rgba(42, 46, 48, 0.94);
      --bottom-sheet-height: 96px;
    }}
    * {{ box-sizing: border-box; }}
    html, body {{
      width: 100vw;
      min-height: 100dvh;
      overflow-x: hidden;
    }}
    body {{
      margin: 0;
      color: var(--ink);
      font-family: "Avenir Next", "Segoe UI", sans-serif;
      background: linear-gradient(180deg, var(--bg) 0%, var(--bg-alt) 100%);
      touch-action: manipulation;
    }}
    button, input, textarea {{
      font: inherit;
    }}
    button {{
      border: 0;
      border-radius: 999px;
      min-height: 44px;
      padding: 11px 16px;
      cursor: pointer;
      color: #08100a;
      background: var(--accent);
      font-weight: 700;
      letter-spacing: 0.04em;
      text-transform: uppercase;
    }}
    button:hover {{ background: var(--accent-strong); }}
    button:focus-visible, input:focus-visible, textarea:focus-visible {{
      outline: 2px solid var(--accent-strong);
      outline-offset: 2px;
    }}
    button.stop {{ background: var(--danger); }}
    button.secondary {{
      color: var(--ink);
      background: rgba(96, 100, 102, 0.72);
      border: 1px solid var(--line);
    }}
    button.icon-button {{
      width: 46px;
      padding: 0;
      color: var(--ink);
      background: rgba(18, 20, 21, 0.76);
      border: 1px solid var(--line);
      font-size: 1.24rem;
      letter-spacing: 0;
      text-transform: none;
    }}
    button:disabled {{
      cursor: not-allowed;
      color: #d7ddd8;
      background: #5c5b55;
    }}
    label {{
      display: block;
      font-size: 0.9rem;
      color: var(--muted);
      margin-bottom: 6px;
    }}
    input[type="text"], textarea {{
      width: 100%;
      padding: 12px;
      border-radius: 10px;
      border: 1px solid var(--line);
      background: rgba(18, 20, 21, 0.82);
      color: var(--ink);
    }}
    h1, h2, h3 {{
      margin: 0;
      text-transform: uppercase;
      letter-spacing: 0.09em;
      font-family: "Avenir Next Condensed", "Franklin Gothic Medium", "Arial Narrow", sans-serif;
    }}
    h2 {{ color: var(--accent); font-size: 1.08rem; }}
    h3 {{ color: var(--accent); font-size: 0.82rem; }}
    .mission-app {{
      position: relative;
      width: 100vw;
      height: 100dvh;
      overflow: hidden;
      background: #07090a;
    }}
    .map-stage {{
      position: absolute;
      inset: 0;
      z-index: 1;
    }}
    #record-map, #splat-view {{
      position: absolute;
      inset: 0;
      width: 100%;
      height: 100%;
      min-height: 0;
      border-radius: 0;
      background: #07090a;
    }}
    #splat-view {{ display: none; }}
    #splat-canvas {{
      display: block;
      width: 100%;
      height: 100%;
    }}
    .app-topbar {{
      position: absolute;
      top: calc(10px + env(safe-area-inset-top));
      left: calc(10px + env(safe-area-inset-left));
      right: calc(10px + env(safe-area-inset-right));
      z-index: 700;
      display: grid;
      grid-template-columns: 46px minmax(0, 1fr) 46px;
      gap: 10px;
      align-items: center;
      min-height: 52px;
      padding: 5px;
      border: 1px solid var(--line);
      border-radius: 18px;
      background: rgba(24, 27, 29, 0.78);
      box-shadow: 0 16px 36px rgba(0, 0, 0, 0.32);
      backdrop-filter: blur(8px);
    }}
    .mission-title-button {{
      min-width: 0;
      justify-self: start;
      max-width: 100%;
      color: var(--ink);
      background: transparent;
      border: 0;
      padding: 8px 10px;
      text-align: left;
      letter-spacing: 0;
      text-transform: none;
    }}
    .mission-title-button strong {{
      display: block;
      overflow: hidden;
      color: var(--accent);
      text-overflow: ellipsis;
      white-space: nowrap;
      font-size: 1.08rem;
    }}
    .mission-title-button span {{
      display: block;
      overflow: hidden;
      color: var(--muted);
      text-overflow: ellipsis;
      white-space: nowrap;
      font-size: 0.78rem;
    }}
    .floating-surface, .drawer, .sheet, .popover, .modal-card {{
      border: 1px solid var(--line);
      background: var(--sheet);
      box-shadow: 0 18px 42px rgba(0, 0, 0, 0.36);
      backdrop-filter: blur(8px);
    }}
    .nav-drawer, .mission-drawer {{
      position: absolute;
      top: 0;
      bottom: 0;
      left: 0;
      z-index: 900;
      width: min(90vw, 390px);
      padding: calc(18px + env(safe-area-inset-top)) 18px calc(18px + env(safe-area-inset-bottom));
      transform: translateX(-105%);
      transition: transform 180ms ease;
      overflow-y: auto;
    }}
    .nav-drawer.open, .mission-drawer.open {{
      transform: translateX(0);
    }}
    .drawer-backdrop, .modal-backdrop {{
      position: absolute;
      inset: 0;
      z-index: 850;
      display: none;
      background: rgba(0, 0, 0, 0.42);
    }}
    .drawer-backdrop.show, .modal-backdrop.show {{ display: block; }}
    .drawer-header, .sheet-header {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 14px;
    }}
    .nav-list, .mission-list, .action-list, .run-list {{
      display: grid;
      gap: 9px;
    }}
    .nav-link, .mission-row, .action-list button, .run-row {{
      display: block;
      width: 100%;
      min-height: 44px;
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      color: var(--ink);
      background: rgba(18, 20, 21, 0.58);
      text-align: left;
      text-decoration: none;
      letter-spacing: 0;
      text-transform: none;
    }}
    .mission-row.active {{
      border-color: rgba(253, 202, 15, 0.72);
      background: rgba(253, 202, 15, 0.15);
    }}
    .mission-row strong, .run-row strong {{ display: block; }}
    .mission-row span, .run-row span {{
      display: block;
      margin-top: 4px;
      color: var(--muted);
      font-size: 0.86rem;
    }}
    .filter-row {{
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      margin: 12px 0;
    }}
    .filter-chip {{
      min-height: 34px;
      padding: 7px 11px;
      color: var(--ink);
      background: rgba(18, 20, 21, 0.62);
      border: 1px solid var(--line);
      font-size: 0.78rem;
    }}
    .filter-chip.active {{
      color: #08100a;
      background: var(--accent);
    }}
    .map-view-control {{
      position: absolute;
      left: calc(14px + env(safe-area-inset-left));
      bottom: calc(var(--bottom-sheet-height) + 18px + env(safe-area-inset-bottom));
      z-index: 650;
      display: flex;
      gap: 6px;
      padding: 5px;
      border-radius: 12px;
      transition: bottom 160ms ease;
    }}
    .map-view-control button {{
      min-width: 48px;
      border-radius: 8px;
      padding: 9px 11px;
    }}
    .map-tools {{
      position: absolute;
      right: calc(14px + env(safe-area-inset-right));
      bottom: calc(var(--bottom-sheet-height) + 18px + env(safe-area-inset-bottom));
      z-index: 650;
      display: grid;
      gap: 10px;
      transition: bottom 160ms ease;
    }}
    .map-layer-control {{
      position: relative;
      z-index: 651;
      font: 13px/1.45 "Avenir Next", "Segoe UI", sans-serif;
    }}
    .map-layer-button {{
      min-width: 104px;
      color: var(--ink);
      background: rgba(18, 20, 21, 0.78);
      border: 1px solid var(--line);
      border-radius: 999px;
      box-shadow: 0 12px 28px rgba(0, 0, 0, 0.28);
    }}
    .map-layer-panel {{
      position: absolute;
      right: 0;
      bottom: calc(100% + 8px);
      display: none;
      min-width: 210px;
      padding: 12px;
      border-radius: 12px;
      color: var(--ink);
    }}
    .map-layer-control.open .map-layer-panel {{
      display: grid;
      gap: 8px;
    }}
    .map-layer-panel label {{
      display: flex;
      align-items: center;
      gap: 8px;
      margin: 0;
      color: var(--ink);
      white-space: nowrap;
    }}
    .bottom-sheet {{
      position: absolute;
      left: max(10px, env(safe-area-inset-left));
      right: max(10px, env(safe-area-inset-right));
      bottom: max(10px, env(safe-area-inset-bottom));
      z-index: 700;
      max-width: 860px;
      margin: 0 auto;
      height: var(--bottom-sheet-height);
      border-radius: 20px;
      padding: 8px 16px 16px;
      overflow: hidden;
      transition: height 180ms ease;
    }}
    .bottom-sheet.medium {{ --bottom-sheet-height: min(52dvh, 440px); }}
    .bottom-sheet.expanded {{ --bottom-sheet-height: min(88dvh, 760px); }}
    .sheet-handle {{
      width: 64px;
      height: 5px;
      margin: 0 auto 8px;
      border-radius: 999px;
      background: rgba(245, 241, 223, 0.42);
    }}
    .sheet-summary {{
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      gap: 12px;
      align-items: center;
    }}
    .sheet-summary-toggle {{
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      gap: 12px;
      align-items: center;
      min-width: 0;
      padding: 6px 0;
      color: var(--ink);
      background: transparent;
      border: 0;
      text-align: left;
      letter-spacing: 0;
      text-transform: none;
    }}
    .sheet-chevron {{
      color: var(--accent);
      font-size: 1.1rem;
      line-height: 1;
    }}
    .sheet-summary strong {{
      display: block;
      overflow: hidden;
      color: var(--ink);
      text-overflow: ellipsis;
      white-space: nowrap;
      font-size: 1.04rem;
    }}
    .sheet-summary span {{ color: var(--muted); font-size: 0.88rem; }}
    .status-chip {{
      display: inline-flex;
      align-items: center;
      border-radius: 999px;
      padding: 6px 10px;
      background: rgba(253, 202, 15, 0.12);
      color: var(--accent);
      font-weight: 700;
      font-size: 0.78rem;
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }}
    .status-chip.idle {{
      background: rgba(180, 83, 9, 0.12);
      color: var(--gold);
    }}
    .sheet-tabs {{
      display: none;
      gap: 8px;
      margin-top: 16px;
    }}
    .bottom-sheet.medium .sheet-tabs, .bottom-sheet.expanded .sheet-tabs,
    .bottom-sheet.medium .sheet-content, .bottom-sheet.expanded .sheet-content {{
      display: grid;
    }}
    .sheet-tab {{
      color: var(--ink);
      background: rgba(18, 20, 21, 0.52);
      border: 1px solid var(--line);
      border-radius: 8px;
    }}
    .sheet-tab.active {{
      color: #08100a;
      background: var(--accent);
    }}
    .sheet-content {{
      display: none;
      max-height: calc(var(--bottom-sheet-height) - 154px);
      overflow-y: auto;
      margin-top: 14px;
      padding-right: 4px;
    }}
    .detail-grid {{
      display: grid;
      gap: 9px;
    }}
    .detail-row {{
      display: flex;
      justify-content: space-between;
      gap: 14px;
      border-bottom: 1px solid rgba(253, 202, 15, 0.12);
      padding-bottom: 8px;
    }}
    .detail-row span:first-child {{ color: var(--muted); }}
    .toast {{
      position: absolute;
      top: calc(76px + env(safe-area-inset-top));
      left: 50%;
      z-index: 980;
      display: none;
      width: min(92vw, 520px);
      transform: translateX(-50%);
      border-radius: 12px;
      padding: 12px 14px;
      font-weight: 700;
    }}
    .toast.show {{ display: block; }}
    .toast.ok {{ background: rgba(15, 118, 110, 0.88); color: var(--ink); }}
    .toast.error {{ background: rgba(185, 28, 28, 0.9); color: #fff8f6; }}
    .toast.warn {{ background: rgba(180, 83, 9, 0.9); color: var(--ink); }}
    .side-sheet {{
      position: absolute;
      top: calc(76px + env(safe-area-inset-top));
      right: calc(10px + env(safe-area-inset-right));
      bottom: calc(var(--bottom-sheet-height) + 20px + env(safe-area-inset-bottom));
      z-index: 760;
      display: none;
      width: min(92vw, 420px);
      overflow-y: auto;
      border-radius: 18px;
      padding: 16px;
    }}
    .side-sheet.open {{ display: block; }}
    .toolbar, .meta-grid, .pattern-list {{
      display: grid;
      gap: 10px;
    }}
    .meta-grid {{ grid-template-columns: repeat(2, minmax(0, 1fr)); }}
    .meta, .pattern-option {{
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      background: var(--panel);
    }}
    .pattern-option input {{ margin-right: 8px; }}
    .path-row {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      margin-top: 14px;
    }}
    .area-edit-toolbar {{
      position: absolute;
      left: 50%;
      bottom: calc(var(--bottom-sheet-height) + 18px + env(safe-area-inset-bottom));
      z-index: 660;
      display: none;
      grid-template-columns: repeat(5, auto);
      gap: 8px;
      max-width: calc(100vw - 28px);
      padding: 8px;
      overflow-x: auto;
      transform: translateX(-50%);
      border-radius: 14px;
      transition: bottom 160ms ease;
    }}
    .area-edit-toolbar.open {{ display: grid; }}
    .area-edit-toolbar button {{
      min-width: max-content;
      border-radius: 8px;
      color: var(--ink);
      background: rgba(18, 20, 21, 0.72);
      border: 1px solid var(--line);
      text-transform: none;
      letter-spacing: 0;
    }}
    .area-edit-toolbar button.active {{
      color: #08100a;
      background: var(--accent);
    }}
    .zone-dot {{
      width: 16px;
      height: 16px;
      border-radius: 50%;
      border: 2px solid #101214;
      background: var(--accent);
      box-shadow: 0 2px 8px rgba(0, 0, 0, 0.35);
    }}
    .zone-dot.station {{ background: #38bdf8; }}
    .countdown {{
      font-size: 0.88rem;
      color: var(--gold);
      min-height: 1.2rem;
    }}
    .action-menu {{
      position: absolute;
      top: calc(70px + env(safe-area-inset-top));
      right: calc(12px + env(safe-area-inset-right));
      z-index: 820;
      display: none;
      width: min(86vw, 280px);
      border-radius: 14px;
      padding: 10px;
    }}
    .action-menu.open {{ display: block; }}
    .run-review-banner {{
      position: absolute;
      top: calc(76px + env(safe-area-inset-top));
      left: 50%;
      z-index: 760;
      display: none;
      width: min(92vw, 560px);
      grid-template-columns: minmax(0, 1fr) auto;
      gap: 12px;
      align-items: center;
      transform: translateX(-50%);
      border-radius: 14px;
      padding: 10px 12px;
    }}
    .run-review-banner.show {{ display: grid; }}
    .run-review-banner span {{
      overflow: hidden;
      color: var(--muted);
      text-overflow: ellipsis;
      white-space: nowrap;
    }}
    .modal-backdrop {{ z-index: 950; place-items: center; }}
    .modal-backdrop.show {{ display: grid; }}
    .modal-card {{
      width: min(92vw, 440px);
      max-height: 88dvh;
      overflow-y: auto;
      border-radius: 18px;
      padding: 18px;
    }}
    .gaussian-overlay-summary {{
      position: absolute;
      left: calc(14px + env(safe-area-inset-left));
      top: calc(76px + env(safe-area-inset-top));
      z-index: 500;
      max-width: min(460px, calc(100% - 28px));
      padding: 12px 14px;
      border: 1px solid rgba(253, 202, 15, 0.35);
      border-radius: 14px;
      background: rgba(7, 9, 10, 0.84);
      color: var(--ink);
      box-shadow: 0 12px 28px rgba(0, 0, 0, 0.28);
      pointer-events: none;
    }}
    .gaussian-overlay-summary strong {{
      display: block;
      color: var(--accent);
      text-transform: uppercase;
      letter-spacing: 0.05em;
      margin-bottom: 4px;
    }}
    .gaussian-overlay-summary.failed strong {{
      color: var(--danger);
    }}
    .muted {{ color: var(--muted); }}
    .map-combo-list {{
      display: grid;
      gap: 8px;
      max-height: 52dvh;
      overflow-y: auto;
    }}
    .map-combo-option {{
      width: 100%;
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 11px 12px;
      color: var(--ink);
      background: rgba(18, 20, 21, 0.58);
      text-align: left;
      letter-spacing: 0;
      text-transform: none;
    }}
    .map-combo-option:hover, .map-combo-option.active {{
      background: rgba(253, 202, 15, 0.16);
    }}
    .map-combo-option strong, .map-combo-option span {{
      display: block;
    }}
    .map-combo-option span {{
      margin-top: 4px;
      color: var(--muted);
      font-size: 0.86rem;
    }}
    .hidden {{ display: none !important; }}
    @media (max-width: 520px) {{
      .meta-grid {{ grid-template-columns: 1fr; }}
      .bottom-sheet {{ left: 0; right: 0; bottom: 0; border-radius: 20px 20px 0 0; }}
      .side-sheet {{ left: 8px; right: 8px; width: auto; }}
    }}
    {self._shared_topbar_css(absolute=True)}
  </style>
</head>
<body>
  <main id="mission-app" class="mission-app">
    <section class="map-stage" aria-label="Mission map workspace">
        <div id="record-map"></div>
        <div id="splat-view"><canvas id="splat-canvas"></canvas></div>
        <div id="gaussian-overlay-summary" class="gaussian-overlay-summary">
          <strong>3D environment</strong>
          <span>No 3D environment build started.</span>
        </div>
    </section>

    <div id="banner" class="toast" role="status" aria-live="polite"></div>

    {self._render_topbar("Missions", '<button id="mission-title-button" class="icon-button" type="button" aria-label="Select mission">+</button><button id="mission-menu-button" class="icon-button" type="button" aria-label="Mission actions">&#8942;</button>')}

    <div id="drawer-backdrop" class="drawer-backdrop"></div>
    <nav id="nav-drawer" class="drawer nav-drawer" aria-label="Application navigation">
      <div class="drawer-header">
        <h2>Navigate</h2>
        <button id="close-nav-button" class="icon-button" type="button" aria-label="Close navigation">&times;</button>
      </div>
      <div class="nav-list">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Calendar</a>
        <a class="nav-link" href="/missions">Missions</a>
        <a class="nav-link" href="/teleop">Teleop</a>
        <a class="nav-link" href="/developer">Developer</a>
      </div>
    </nav>

    <aside id="mission-drawer" class="drawer mission-drawer" aria-label="Mission Selection">
      <div class="drawer-header">
        <h2>Mission Selection</h2>
        <button id="close-mission-drawer-button" class="icon-button" type="button" aria-label="Close mission selection">&times;</button>
      </div>
      <input id="map-name" type="text" autocomplete="off" placeholder="Search missions">
      <div class="filter-row">
        <button class="filter-chip active" type="button" data-filter="all">All</button>
        <button class="filter-chip" type="button" data-filter="ready">Ready</button>
        <button class="filter-chip" type="button" data-filter="processing">Processing</button>
        <button class="filter-chip" type="button" data-filter="attention">Attention</button>
      </div>
      <div id="map-combo" class="hidden">
        <button id="map-combo-button" type="button" aria-label="Select mission"></button>
      </div>
      <div id="map-combo-list" class="map-combo-list"></div>
      <button id="new-mission-button" class="secondary" type="button" style="width: 100%; margin-top: 14px;">+ New Mission</button>
    </aside>

    <div class="map-view-control floating-surface" aria-label="Map view mode">
      <button id="view-2d-button" type="button" aria-label="Switch to 2D view">2D</button>
      <button id="view-3d-button" class="secondary" type="button" aria-label="Switch to 3D view">3D</button>
    </div>

    <div class="map-tools">
      <div id="map-layer-control" class="map-layer-control">
        <button id="map-layer-button" class="map-layer-button" type="button" aria-label="Map layers" title="Map layers">Layers</button>
        <div class="map-layer-panel floating-surface">
          <h3>Layers</h3>
          <label data-layer-mode="2d"><input class="layer-toggle" data-layer="background" type="checkbox" checked> Satellite</label>
          <label data-layer-mode="both"><input class="layer-toggle" data-layer="boundary" type="checkbox" checked> Mission boundary</label>
          <label data-layer-mode="both"><input class="layer-toggle" data-layer="work_areas" type="checkbox" checked> Work areas</label>
          <label data-layer-mode="both"><input class="layer-toggle" data-layer="no_go" type="checkbox" checked> No-go areas</label>
          <label data-layer-mode="both"><input class="layer-toggle" data-layer="transit" type="checkbox" checked> Transit paths</label>
          <label data-layer-mode="both"><input class="layer-toggle" data-layer="planned" type="checkbox" checked> Mission path</label>
          <label data-layer-mode="both"><input class="layer-toggle" data-layer="recorded" type="checkbox" checked> Actual path</label>
          <label data-layer-mode="both"><input class="layer-toggle" data-layer="start_finish" type="checkbox" checked> Start / finish</label>
          <label data-layer-mode="both"><input class="layer-toggle" data-layer="zones" type="checkbox" checked> Work zones</label>
          <label data-layer-mode="both"><input class="layer-toggle" data-layer="station" type="checkbox" checked> Station</label>
          <label data-layer-mode="both"><input class="layer-toggle" data-layer="robot" type="checkbox" checked> Robot</label>
          <label data-layer-mode="both"><input class="layer-toggle" data-layer="obstacles" type="checkbox" checked> Obstacles</label>
          <label data-layer-mode="3d"><input class="layer-toggle" data-layer="gaussian" type="checkbox" checked> 3D environment</label>
        </div>
      </div>
      <button id="fit-mission-button" class="icon-button" type="button" aria-label="Fit mission" title="Fit mission">&#9678;</button>
    </div>

    <div id="area-edit-toolbar" class="floating-surface area-edit-toolbar" aria-label="Area editing">
      <button type="button" data-area-tool="WORK_AREA">+ Work area</button>
      <button type="button" data-area-tool="NO_GO">+ No-go area</button>
      <button type="button" data-area-tool="TRANSIT">+ Transit path</button>
      <button type="button" data-area-tool="STATION">+ Station</button>
      <button id="cancel-area-edit-button" type="button">Done</button>
    </div>

    <section id="mission-bottom-sheet" class="sheet bottom-sheet" aria-label="Mission details">
      <button id="sheet-handle-button" type="button" class="mission-title-button" aria-label="Open mission details" style="width: 100%; padding: 0;">
        <div class="sheet-handle"></div>
      </button>
      <div class="sheet-summary">
        <button id="sheet-summary-toggle" class="sheet-summary-toggle" type="button" aria-expanded="false" aria-label="Toggle mission details">
          <span>
            <strong id="sheet-mission-title">No mission selected</strong>
            <span id="sheet-mission-meta">Select a mission to review it.</span>
          </span>
          <span id="sheet-chevron" class="sheet-chevron" aria-hidden="true">&#8963;</span>
        </button>
        <button id="start-mission-button" type="button">Start</button>
      </div>
      <div class="sheet-tabs">
        <button id="mission-tab-button" class="sheet-tab active" type="button">Mission</button>
        <button id="runs-tab-button" class="sheet-tab" type="button">Runs</button>
      </div>
      <div id="mission-tab-panel" class="sheet-content">
        <div id="mission-detail-grid" class="detail-grid"></div>
      </div>
      <div id="runs-tab-panel" class="sheet-content hidden">
        <div id="run-list" class="run-list"></div>
      </div>
    </section>

    <section id="mission-action-menu" class="popover action-menu" aria-label="Mission actions">
      <div class="action-list">
        <button id="open-area-edit-button" type="button">Edit areas</button>
        <button id="open-map-edit-button" type="button">Map edit</button>
        <button id="open-path-edit-button" type="button">Edit path</button>
        <button id="open-mission-settings-button" type="button">Mission settings</button>
        <button id="open-advanced-button" type="button">Advanced information</button>
        <button id="download-mission-button" type="button">Download Mission JSON</button>
        <button id="save-map-button" type="button">Save As Mission</button>
        <button id="rename-mission-button" type="button">Rename</button>
        <button id="delete-map-button" class="stop" type="button">Delete</button>
      </div>
    </section>

    <section id="run-review-banner" class="floating-surface run-review-banner" aria-label="Run review">
      <span id="run-review-label">Reviewing run</span>
      <button id="exit-run-review-button" class="secondary" type="button">Exit review</button>
    </section>

    <aside id="map-edit-sheet" class="sheet side-sheet" aria-label="Map Edit">
      <div class="sheet-header">
        <h2>Map Edit</h2>
        <button class="icon-button close-side-sheet-button" type="button" aria-label="Close Map Edit">&times;</button>
      </div>
      <div class="toolbar">
        <div id="recording-chip" class="status-chip idle">RecordMap idle</div>
        <button id="build-gaussian-button" class="secondary" type="button">Build 3D Environment</button>
      </div>
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
      <div id="gaussian-build-status" class="muted" style="margin-top: 12px;">3D environment idle.</div>
      <div id="latest-map-message" class="muted" style="margin-top: 12px;">Select a mission to review map artifacts.</div>
    </aside>

    <aside id="path-edit-sheet" class="sheet side-sheet" aria-label="Path Edit">
      <div class="sheet-header">
        <h2>Path Edit</h2>
        <button class="icon-button close-side-sheet-button" type="button" aria-label="Close Path Edit">&times;</button>
      </div>
      <strong>Pattern</strong>
      <div id="pattern-countdown" class="countdown"></div>
      <div class="pattern-list" style="margin-top: 8px;">
        <label class="pattern-option"><input type="radio" name="pattern" value="zigzag"> Zigzag coverage</label>
        <label class="pattern-option"><input type="radio" name="pattern" value="random"> Random roaming coverage</label>
        <label class="pattern-option"><input type="radio" name="pattern" value="spiral"> Spiral inward coverage</label>
      </div>
      <div class="path-row">
        <label for="planned-path-toggle">Mission path</label>
        <input id="planned-path-toggle" class="layer-toggle" data-layer="planned" type="checkbox" checked>
      </div>
      <div style="margin-top: 14px;">
        <label for="start-position">Start position</label>
        <input id="start-position" type="range" min="0" max="100" value="0">
      </div>
      <div style="margin-top: 14px;">
        <label class="pattern-option"><input id="use-end-position" type="checkbox"> Use end position</label>
        <input id="end-position" type="range" min="0" max="100" value="100">
      </div>
      <div class="toolbar" style="margin-top: 16px;">
        <button id="save-button" type="button">Save Path</button>
        <button id="refresh-button" class="secondary" type="button">Refresh</button>
      </div>
    </aside>

    <aside id="area-edit-sheet" class="sheet side-sheet" aria-label="Edit Areas">
      <div class="sheet-header">
        <h2>Edit Areas</h2>
        <button class="icon-button close-side-sheet-button" type="button" aria-label="Close Edit Areas">&times;</button>
      </div>
      <div id="area-edit-message" class="muted">Choose an area tool, then tap the map.</div>
      <div style="margin-top: 12px;">
        <label for="area-name-input">Name</label>
        <input id="area-name-input" type="text" placeholder="Area name">
      </div>
      <div id="area-point-list" class="muted" style="margin-top: 10px;">No points selected.</div>
      <div class="toolbar" style="margin-top: 14px;">
        <button id="save-area-button" type="button">Save area</button>
        <button id="cancel-area-button" class="secondary" type="button">Cancel</button>
        <button id="delete-area-button" class="stop" type="button">Delete</button>
      </div>
    </aside>

    <aside id="mission-settings-sheet" class="sheet side-sheet" aria-label="Mission Settings">
      <div class="sheet-header">
        <h2>Mission Settings</h2>
        <button class="icon-button close-side-sheet-button" type="button" aria-label="Close Mission Settings">&times;</button>
      </div>
      <div class="detail-grid" id="mission-settings-grid"></div>
    </aside>

    <aside id="advanced-sheet" class="sheet side-sheet" aria-label="Advanced Information">
      <div class="sheet-header">
        <h2>Advanced Information</h2>
        <button class="icon-button close-side-sheet-button" type="button" aria-label="Close Advanced Information">&times;</button>
      </div>
      <div id="advanced-detail-grid" class="detail-grid"></div>
    </aside>

    <div id="modal-backdrop" class="modal-backdrop">
      <section id="start-confirmation-sheet" class="modal-card" aria-label="Start Mission">
        <div class="sheet-header">
          <h2>Start Mission</h2>
          <button id="cancel-start-x-button" class="icon-button" type="button" aria-label="Cancel start">&times;</button>
        </div>
        <div id="start-confirmation-summary" class="detail-grid"></div>
        <label class="pattern-option" style="margin-top: 14px;">
          <input id="record-rosbag-toggle" type="checkbox"> Record rosbag
        </label>
        <div class="toolbar" style="margin-top: 16px; grid-template-columns: 1fr 1fr;">
          <button id="cancel-start-button" class="secondary" type="button">Cancel</button>
          <button id="confirm-start-mission-button" type="button">Start</button>
        </div>
      </section>
      <section id="new-mission-sheet" class="modal-card hidden" aria-label="New Mission">
        <div class="sheet-header">
          <h2>New Mission</h2>
          <button id="cancel-new-mission-button" class="icon-button" type="button" aria-label="Close New Mission">&times;</button>
        </div>
        <input id="upload-file" type="file" accept=".json,application/json" style="margin-top: 10px;">
        <input id="upload-mission-id" type="text" placeholder="Mission ID" style="margin-top: 10px;">
        <label class="pattern-option" style="margin-top: 10px;"><input id="upload-overwrite" type="checkbox"> Overwrite existing mission with same id</label>
        <textarea id="upload-json" rows="12" placeholder="Paste VDA5050 mission JSON" style="margin-top: 10px; font-family: monospace;"></textarea>
        <button id="upload-button" type="button" style="width: 100%; margin-top: 12px;">Upload Mission</button>
      </section>
    </div>
  </main>

  <script
    src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
    integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo="
    crossorigin=""
  ></script>
  <script>
    {self._shared_topbar_js()}
    const banner = document.getElementById('banner');
    const chip = document.getElementById('recording-chip');
    const latestRun = document.getElementById('latest-run');
    const latestObstacles = document.getElementById('latest-obstacles');
    const latestMapMessage = document.getElementById('latest-map-message');
    const gaussianBuildStatus = document.getElementById('gaussian-build-status');
    const patternCountdown = document.getElementById('pattern-countdown');
    const patternInputs = [...document.querySelectorAll('input[name="pattern"]')];
    const mapCombo = document.getElementById('map-combo');
    const mapComboButton = document.getElementById('map-combo-button');
    const mapComboList = document.getElementById('map-combo-list');
    const mapNameInput = document.getElementById('map-name');
    const missionApp = document.getElementById('mission-app');
    const selectedMissionTitle = document.querySelector('.topbar-title h1');
    const selectedMissionState = document.getElementById('topbar-page-state');
    const sheetMissionTitle = document.getElementById('sheet-mission-title');
    const sheetMissionMeta = document.getElementById('sheet-mission-meta');
    const missionBottomSheet = document.getElementById('mission-bottom-sheet');
    const missionDetailGrid = document.getElementById('mission-detail-grid');
    const advancedDetailGrid = document.getElementById('advanced-detail-grid');
    const runList = document.getElementById('run-list');
    const drawerBackdrop = document.getElementById('drawer-backdrop');
    const navDrawer = document.getElementById('nav-drawer');
    const missionDrawer = document.getElementById('mission-drawer');
    const missionActionMenu = document.getElementById('mission-action-menu');
    const mapEditSheet = document.getElementById('map-edit-sheet');
    const pathEditSheet = document.getElementById('path-edit-sheet');
    const areaEditSheet = document.getElementById('area-edit-sheet');
    const missionSettingsSheet = document.getElementById('mission-settings-sheet');
    const advancedSheet = document.getElementById('advanced-sheet');
    const areaEditToolbar = document.getElementById('area-edit-toolbar');
    const areaEditMessage = document.getElementById('area-edit-message');
    const areaNameInput = document.getElementById('area-name-input');
    const areaPointList = document.getElementById('area-point-list');
    const saveAreaButton = document.getElementById('save-area-button');
    const deleteAreaButton = document.getElementById('delete-area-button');
    const missionSettingsGrid = document.getElementById('mission-settings-grid');
    const runReviewBanner = document.getElementById('run-review-banner');
    const runReviewLabel = document.getElementById('run-review-label');
    const modalBackdrop = document.getElementById('modal-backdrop');
    const startConfirmationSheet = document.getElementById('start-confirmation-sheet');
    const newMissionSheet = document.getElementById('new-mission-sheet');
    const startConfirmationSummary = document.getElementById('start-confirmation-summary');
    const startMissionButton = document.getElementById('start-mission-button');
    const confirmStartMissionButton = document.getElementById('confirm-start-mission-button');
    const renameMissionButton = document.getElementById('rename-mission-button');
    const deleteMapButton = document.getElementById('delete-map-button');
    const recordRosbagToggle = document.getElementById('record-rosbag-toggle');
    const startPositionInput = document.getElementById('start-position');
    const useEndPositionInput = document.getElementById('use-end-position');
    const endPositionInput = document.getElementById('end-position');
    const layerToggles = [...document.querySelectorAll('.layer-toggle')];
    const mapLayerControl = document.getElementById('map-layer-control');
    const mapLayerButton = document.getElementById('map-layer-button');
    const view2dButton = document.getElementById('view-2d-button');
    const view3dButton = document.getElementById('view-3d-button');
    const buildGaussianButton = document.getElementById('build-gaussian-button');
    const saveMapButton = document.getElementById('save-map-button');
    const recordMapElement = document.getElementById('record-map');
    const splatViewElement = document.getElementById('splat-view');
    const splatCanvas = document.getElementById('splat-canvas');
    const gaussianOverlaySummary = document.getElementById('gaussian-overlay-summary');
    const selectedMapStorageKey = 'amr_sweeper.selected_map_id';
    const latestRecordingLabel = 'Last Recording';
    const defaultMissionIds = new Set(['SpotSweep', '3x3Sweep']);
    const protectedMissionIds = new Set(['RecordMap']);
    let mapsCache = [];
    let missionsCache = [];
    let selectedMapId = window.localStorage.getItem(selectedMapStorageKey) || '';
    let missionSearchText = '';
    let missionStatusFilter = 'all';
    let lastFitLatLngs = [];
    let mapNameTouched = false;
    let lastAppliedSourceMapId = null;
    let currentView = '2d';
    let patternTouched = false;
    let countdownTimer = null;
    let countdownSeconds = 20;
    let lastLatestRunId = '';
    let latestGaussianStatus = {{}};
    let activePolyline = null;
    let latestPolyline = null;
    let perimeterPolyline = null;
    let plannedPathPolyline = null;
    let currentMarker = null;
    let startFinishLayer = null;
    let boundaryMaskLayer = null;
    let gaussianLayer = null;
    let semanticZoneLayer = null;
    let obstacleLayer = null;
    let stationLayer = null;
    let draftAreaLayer = null;
    let currentGaussianBuildId = '';
    let gaussianRequestInFlight = false;
    let lastMapSnapshot = {{}};
    let appliedEditorStateKey = '';
    let lastFittedMapKey = '';
    let runReviewRoute = null;
    let areaEditMode = false;
    let activeAreaTool = '';
    let draftAreaPoints = [];
    let editingZoneId = '';

    mapNameInput.value = latestRecordingLabel;
    mapComboList.innerHTML = '<button class="map-combo-option" type="button" disabled>Loading saved maps...</button>';
    gaussianOverlaySummary.style.display = 'none';
    updateTopbarFromStatus({{}}, 'Loading mission');

    const map = L.map('record-map', {{ zoomControl: true }}).setView([55.6761, 12.5683], 18);
    const satelliteTileLayer = L.tileLayer(
      'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}',
      {{ maxZoom: 20, attribution: '&copy; Esri' }}
    );
    const streetTileLayer = L.tileLayer(
      'https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',
      {{ maxZoom: 20, attribution: '&copy; OpenStreetMap contributors' }}
    );
    satelliteTileLayer.addTo(map);

    function setBanner(kind, message) {{
      banner.className = `banner show ${{kind}}`;
      banner.textContent = message;
      window.setTimeout(() => {{
        banner.className = 'toast';
        banner.textContent = '';
      }}, 5000);
    }}

    function missionUiStatus(entry) {{
      if (!entry) {{
        return 'UNAVAILABLE';
      }}
      if (lastMapSnapshot?.active_execution?.mission_id === (entry.mission_id || entry.map_id)) {{
        return 'RUNNING';
      }}
      if (entry.error || entry.gaussian_error || entry.gaussian_splat_error) {{
        return 'NEEDS ATTENTION';
      }}
      if (entry.artifacts_ready === false) {{
        return 'PROCESSING';
      }}
      if (entry.route_available === false && !entry.route_geojson && !entry.navsat_geojson) {{
        return 'UNAVAILABLE';
      }}
      return 'READY';
    }}

    function statusMatchesFilter(entry) {{
      const status = missionUiStatus(entry).toLowerCase();
      if (missionStatusFilter === 'all') {{
        return true;
      }}
      if (missionStatusFilter === 'attention') {{
        return status === 'needs attention' || status === 'unavailable';
      }}
      return status === missionStatusFilter;
    }}

    function missionMetrics(entry) {{
      const area = Number(entry?.area_square_meters || entry?.coverage_area_square_meters || 0);
      const pathLength = Number(entry?.path_length_meters || entry?.actual_path_length_meters || 0);
      const duration = Number(entry?.estimated_duration_seconds || entry?.duration_seconds || 0);
      const parts = [];
      if (area > 0) {{
        parts.push(`${{Math.round(area).toLocaleString()}} m2`);
      }}
      if (pathLength > 0) {{
        parts.push(pathLength >= 1000 ? `${{(pathLength / 1000).toFixed(1)}} km` : `${{Math.round(pathLength)}} m`);
      }}
      if (duration > 0) {{
        const minutes = Math.max(1, Math.round(duration / 60));
        parts.push(minutes >= 60 ? `~${{Math.floor(minutes / 60)}}h ${{minutes % 60}}m` : `~${{minutes}}m`);
      }}
      return parts.join(' | ') || 'Mission artifacts pending';
    }}

    function setSheetState(state) {{
      missionBottomSheet.classList.remove('medium', 'expanded');
      if (state === 'medium' || state === 'expanded') {{
        missionBottomSheet.classList.add(state);
      }}
      const expanded = state === 'medium' || state === 'expanded';
      const summaryToggle = document.getElementById('sheet-summary-toggle');
      const handleToggle = document.getElementById('sheet-handle-button');
      const chevron = document.getElementById('sheet-chevron');
      if (summaryToggle) {{
        summaryToggle.setAttribute('aria-expanded', String(expanded));
      }}
      if (handleToggle) {{
        handleToggle.setAttribute('aria-expanded', String(expanded));
      }}
      if (chevron) {{
        chevron.textContent = expanded ? '⌄' : '⌃';
      }}
      window.setTimeout(() => {{
        missionApp.style.setProperty('--bottom-sheet-height', `${{missionBottomSheet.offsetHeight}}px`);
        map.invalidateSize();
      }}, 0);
    }}

    function closeOverlayPanels() {{
      missionActionMenu.classList.remove('open');
      mapEditSheet.classList.remove('open');
      pathEditSheet.classList.remove('open');
      areaEditSheet.classList.remove('open');
      missionSettingsSheet.classList.remove('open');
      advancedSheet.classList.remove('open');
    }}

    function exitRunReview() {{
      runReviewRoute = null;
      runReviewBanner.classList.remove('show');
      updateMap(lastMapSnapshot);
    }}

    function closeDrawers() {{
      navDrawer.classList.remove('open');
      missionDrawer.classList.remove('open');
      drawerBackdrop.classList.remove('show');
    }}

    function openDrawer(drawer) {{
      closeOverlayPanels();
      closeBatteryPopover();
      navDrawer.classList.toggle('open', drawer === navDrawer);
      missionDrawer.classList.toggle('open', drawer === missionDrawer);
      drawerBackdrop.classList.add('show');
    }}

    function openSideSheet(sheet) {{
      closeDrawers();
      closeBatteryPopover();
      missionActionMenu.classList.remove('open');
      for (const element of [mapEditSheet, pathEditSheet, areaEditSheet, missionSettingsSheet, advancedSheet]) {{
        element.classList.toggle('open', element === sheet);
      }}
    }}

    function detailRow(label, value) {{
      return `<div class="detail-row"><span>${{label}}</span><strong>${{value || '-'}}</strong></div>`;
    }}

    function selectedPattern() {{
      const selected = patternInputs.find((input) => input.checked);
      return selected ? selected.value : '';
    }}

    function layerEnabled(layer) {{
      const input = layerToggles.find((toggle) => toggle.dataset.layer === layer);
      return !input || input.checked;
    }}

    function updateBaseLayer() {{
      const useSatellite = layerEnabled('background');
      const activeLayer = useSatellite ? satelliteTileLayer : streetTileLayer;
      const inactiveLayer = useSatellite ? streetTileLayer : satelliteTileLayer;
      if (map.hasLayer(inactiveLayer)) {{
        map.removeLayer(inactiveLayer);
      }}
      if (!map.hasLayer(activeLayer)) {{
        activeLayer.addTo(map);
      }}
    }}

    function layerVisibility() {{
      const visibility = {{}};
      for (const input of layerToggles) {{
        visibility[input.dataset.layer] = input.checked;
      }}
      return visibility;
    }}

    function syncLayerInputs(source) {{
      for (const input of layerToggles) {{
        if (input !== source && input.dataset.layer === source.dataset.layer) {{
          input.checked = source.checked;
        }}
      }}
    }}

    function updateLayerPanelMode() {{
      const entry = selectedMap();
      for (const label of document.querySelectorAll('[data-layer-mode]')) {{
        const mode = label.dataset.layerMode || 'both';
        const input = label.querySelector('.layer-toggle');
        const layer = input?.dataset?.layer || '';
        label.classList.toggle(
          'hidden',
          (mode !== 'both' && mode !== currentView) || !layerHasData(layer, entry)
        );
      }}
    }}

    function layerHasData(layer, entry) {{
      if (!entry) {{
        return ['background', 'robot'].includes(layer);
      }}
      const zones = entry.semantic_zones || [];
      if (layer === 'work_areas' || layer === 'zones') {{
        return zones.some((zone) => zone.enabled && zone.type === 'WORK_AREA');
      }}
      if (layer === 'no_go') {{
        return zones.some((zone) => zone.enabled && zone.type === 'NO_GO');
      }}
      if (layer === 'transit') {{
        return zones.some((zone) => zone.enabled && zone.type === 'TRANSIT');
      }}
      if (layer === 'station') {{
        return Boolean(entry.station?.position);
      }}
      if (layer === 'obstacles') {{
        return (entry.recorded_obstacle_points || lastMapSnapshot?.latest_recorded_map?.recorded_obstacle_points || []).length > 0;
      }}
      if (layer === 'start_finish' || layer === 'planned' || layer === 'boundary') {{
        return Boolean(entry.route_geojson || entry.navsat_geojson || lastMapSnapshot?.latest_route_geojson);
      }}
      if (layer === 'recorded') {{
        return Boolean(entry.navsat_geojson || lastMapSnapshot?.latest_navsat_geojson || lastMapSnapshot?.active_navsat_geojson);
      }}
      if (layer === 'gaussian') {{
        return Boolean(entry.gaussian_splat_manifest || entry.gaussian_manifest || lastMapSnapshot?.latest_recorded_map?.gaussian_splat_manifest);
      }}
      return true;
    }}

    function applyLayerVisibility(visibility) {{
      if (!visibility || typeof visibility !== 'object') {{
        return;
      }}
      for (const input of layerToggles) {{
        if (Object.prototype.hasOwnProperty.call(visibility, input.dataset.layer)) {{
          input.checked = Boolean(visibility[input.dataset.layer]);
        }}
      }}
    }}

    function selectedMap() {{
      return mapsCache.find((entry) => entry.map_id === selectedMapId) || null;
    }}

    function missionIsProtected(entry) {{
      const missionId = entry?.mission_id || entry?.map_id || selectedMapId || '';
      return Boolean(entry?.readonly) || protectedMissionIds.has(missionId) || defaultMissionIds.has(missionId);
    }}

    function selectedMission() {{
      return missionsCache.find((entry) => entry.mission_id === selectedMapId)
        || selectedMap()
        || null;
    }}

    function selectedMissionGroup(entry) {{
      const missionId = entry?.mission_id || entry?.map_id || '';
      if (missionId === 'RecordMap') {{
        return 'Latest Recording';
      }}
      if (defaultMissionIds.has(missionId)) {{
        return 'Default Missions';
      }}
      return 'Saved Missions';
    }}

    function missionIsSelectable(entry) {{
      const missionId = String(entry?.mission_id || entry?.map_id || '').trim();
      if (!missionId || missionId === 'Teleop') {{
        return false;
      }}
      const directory = String(entry?.directory || entry?.mission_directory || entry?.source_directory || '');
      if (directory.includes('/missions/simulations/') || directory.includes('missions/simulations/')) {{
        return false;
      }}
      const missionType = String(entry?.mission_type || entry?.source || '').toLowerCase();
      return !missionType.includes('simulation');
    }}

    function selectedMissionIsSavedEditable() {{
      const entry = selectedMap();
      return Boolean(entry) && !missionIsProtected(entry);
    }}

    function recordRosbagStorageKey(missionId) {{
      return `amr_sweeper_record_rosbag_${{missionId}}`;
    }}

    function loadRecordRosbagPreference(missionId) {{
      return window.localStorage.getItem(recordRosbagStorageKey(missionId)) === '1';
    }}

    function saveRecordRosbagPreference(missionId, enabled) {{
      window.localStorage.setItem(recordRosbagStorageKey(missionId), enabled ? '1' : '0');
    }}

    function gaussianStatusMatchesSelectedSource(status) {{
      if (!status || Object.keys(status).length === 0) {{
        return false;
      }}
      const captureManifest = String(status.capture_manifest_file || '');
      const outputDirectory = String(status.output_directory || '');
      const missionId = String(status.mission_id || '');
      if (selectedMapId) {{
        return missionId === selectedMapId
          || captureManifest.includes(`/missions/logs/${{selectedMapId}}/_3D_map/gaussian/manifest.json`)
          || outputDirectory.includes(`/missions/logs/${{selectedMapId}}/_3D_map/gaussian_splat`)
          || captureManifest.includes(`/missions/maps/${{selectedMapId}}/gaussian/manifest.json`)
          || outputDirectory.includes(`/missions/maps/${{selectedMapId}}/gaussian_splat`);
      }}
      const latestManifest = String(lastMapSnapshot?.latest_recorded_map?.gaussian_manifest_file || '');
      if (latestManifest && captureManifest) {{
        return latestManifest === captureManifest;
      }}
      return !missionId || missionId === 'RecordMap';
    }}

    function gaussianStatusForSelectedSource() {{
      return gaussianStatusMatchesSelectedSource(latestGaussianStatus) ? latestGaussianStatus : {{}};
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
            const latitude = Number(coordinate[1]);
            const longitude = Number(coordinate[0]);
            if (Number.isFinite(latitude) && Number.isFinite(longitude)) {{
              latlngs.push([latitude, longitude]);
            }}
          }}
        }}
      }}
      return latlngs;
    }}

    function routeFrameFromGeojson(geojson) {{
      for (const feature of geojson?.features || []) {{
        const properties = feature?.properties || {{}};
        const coordinateFrame = String(properties.coordinate_frame || '').trim();
        if (coordinateFrame) {{
          return coordinateFrame;
        }}
        const frameId = String(properties.frame_id || '').trim();
        if (frameId) {{
          return frameId;
        }}
      }}
      return '';
    }}

    function isLeafletLatLngGeojson(geojson) {{
      const frame = routeFrameFromGeojson(geojson).toLowerCase();
      if (frame) {{
        return frame.includes('wgs84') || frame.includes('gps') || frame.includes('navsat') || frame.includes('latlon');
      }}

      const latlngs = lineStringLatLngs(geojson);
      if (latlngs.length === 0) {{
        return false;
      }}
      return latlngs.every((point) => (
        Math.abs(point[0]) <= 90 &&
        Math.abs(point[1]) <= 180
      )) && latlngs.some((point) => Math.abs(point[0]) > 20 || Math.abs(point[1]) > 20);
    }}

    function leafletLatLngs(geojson) {{
      return isLeafletLatLngGeojson(geojson) ? lineStringLatLngs(geojson) : [];
    }}

    function clampPercent(value) {{
      const number = Number(value);
      if (!Number.isFinite(number)) {{
        return 0;
      }}
      return Math.max(0, Math.min(100, number));
    }}

    function plannedPathSegment(latlngs) {{
      if (latlngs.length <= 2) {{
        return latlngs;
      }}
      const startPercent = clampPercent(startPositionInput.value);
      const endPercent = useEndPositionInput.checked ? clampPercent(endPositionInput.value) : 100;
      const lowerPercent = Math.min(startPercent, endPercent);
      const upperPercent = Math.max(startPercent, endPercent);
      const lastIndex = latlngs.length - 1;
      const startIndex = Math.floor((lowerPercent / 100) * lastIndex);
      const endIndex = Math.ceil((upperPercent / 100) * lastIndex);
      return latlngs.slice(startIndex, Math.min(lastIndex, endIndex) + 1);
    }}

    function geometryLatLngs(geometry) {{
      if (!geometry || typeof geometry !== 'object') {{
        return [];
      }}
      if (geometry.type === 'Polygon') {{
        const ring = geometry.coordinates?.[0] || [];
        return ring
          .filter((point) => Array.isArray(point) && point.length >= 2)
          .map((point) => [Number(point[1]), Number(point[0])])
          .filter((point) => Number.isFinite(point[0]) && Number.isFinite(point[1]));
      }}
      if (geometry.type === 'LineString') {{
        return (geometry.coordinates || [])
          .filter((point) => Array.isArray(point) && point.length >= 2)
          .map((point) => [Number(point[1]), Number(point[0])])
          .filter((point) => Number.isFinite(point[0]) && Number.isFinite(point[1]));
      }}
      return [];
    }}

    function zoneLayerEnabled(zoneType) {{
      if (zoneType === 'WORK_AREA') {{
        return layerEnabled('work_areas') || layerEnabled('zones');
      }}
      if (zoneType === 'NO_GO') {{
        return layerEnabled('no_go');
      }}
      if (zoneType === 'TRANSIT') {{
        return layerEnabled('transit');
      }}
      return true;
    }}

    function zoneStyle(zoneType) {{
      if (zoneType === 'NO_GO') {{
        return {{ color: '#ff7b5c', fillColor: '#ff7b5c', fillOpacity: 0.24, weight: 2 }};
      }}
      if (zoneType === 'TRANSIT') {{
        return {{ color: '#38bdf8', weight: 4, dashArray: '10 8' }};
      }}
      return {{ color: '#22c55e', fillColor: '#22c55e', fillOpacity: 0.18, weight: 2 }};
    }}

    function drawSemanticObjects(entry, bounds) {{
      semanticZoneLayer = L.layerGroup();
      for (const zone of entry?.semantic_zones || []) {{
        if (!zone?.enabled || !zoneLayerEnabled(zone.type)) {{
          continue;
        }}
        const latlngs = geometryLatLngs(zone.geometry);
        if (latlngs.length < 2) {{
          continue;
        }}
        const style = zoneStyle(zone.type);
        const layer = zone.type === 'TRANSIT'
          ? L.polyline(latlngs, style)
          : L.polygon(latlngs, style);
        layer.bindTooltip(zone.name || zone.id || 'Area');
        layer.on('click', () => beginEditExistingZone(zone));
        layer.addTo(semanticZoneLayer);
        bounds.push(...latlngs);
      }}
      if (semanticZoneLayer.getLayers().length) {{
        semanticZoneLayer.addTo(map);
      }}

      const station = entry?.station;
      if (layerEnabled('station') && station?.position) {{
        const y = Number(station.position.y);
        const x = Number(station.position.x);
        if (Number.isFinite(x) && Number.isFinite(y)) {{
          stationLayer = L.circleMarker([y, x], {{
            radius: 8,
            color: '#ffffff',
            weight: 2,
            fillColor: '#38bdf8',
            fillOpacity: 1,
          }}).addTo(map);
          stationLayer.bindTooltip(station.name || 'Station');
          bounds.push([y, x]);
        }}
      }}
    }}

    function drawStartFinish(plannedLatLngs, bounds) {{
      if (!layerEnabled('start_finish') || plannedLatLngs.length < 2) {{
        return;
      }}
      startFinishLayer = L.layerGroup();
      L.circleMarker(plannedLatLngs[0], {{
        radius: 7,
        color: '#101214',
        weight: 2,
        fillColor: '#22c55e',
        fillOpacity: 1,
      }}).bindTooltip('Start').addTo(startFinishLayer);
      L.circleMarker(plannedLatLngs[plannedLatLngs.length - 1], {{
        radius: 7,
        color: '#101214',
        weight: 2,
        fillColor: '#ff7b5c',
        fillOpacity: 1,
      }}).bindTooltip('Finish').addTo(startFinishLayer);
      startFinishLayer.addTo(map);
      bounds.push(plannedLatLngs[0], plannedLatLngs[plannedLatLngs.length - 1]);
    }}

    function fitMapToDefaultBounds(data, latlngs) {{
      const key = `${{editorStateKey(data)}}:${{latlngs.length}}`;
      if (key === lastFittedMapKey || latlngs.length === 0) {{
        return;
      }}
      lastFitLatLngs = latlngs;
      map.fitBounds(latlngs, {{ padding: [34, 34], maxZoom: 19 }});
      lastFittedMapKey = key;
    }}

    function fitSelectedMission() {{
      if (lastFitLatLngs.length > 0 && currentView === '2d') {{
        map.fitBounds(lastFitLatLngs, {{ padding: [42, 42], maxZoom: 19 }});
      }} else {{
        renderSplatPreview(lastMapSnapshot);
      }}
    }}

    function updateMap(data) {{
      updateBaseLayer();
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
      if (plannedPathPolyline) {{
        map.removeLayer(plannedPathPolyline);
        plannedPathPolyline = null;
      }}
      if (currentMarker) {{
        map.removeLayer(currentMarker);
        currentMarker = null;
      }}
      if (startFinishLayer) {{
        map.removeLayer(startFinishLayer);
        startFinishLayer = null;
      }}
      if (boundaryMaskLayer) {{
        map.removeLayer(boundaryMaskLayer);
        boundaryMaskLayer = null;
      }}
      if (gaussianLayer) {{
        map.removeLayer(gaussianLayer);
        gaussianLayer = null;
      }}
      if (semanticZoneLayer) {{
        map.removeLayer(semanticZoneLayer);
        semanticZoneLayer = null;
      }}
      if (obstacleLayer) {{
        map.removeLayer(obstacleLayer);
        obstacleLayer = null;
      }}
      if (stationLayer) {{
        map.removeLayer(stationLayer);
        stationLayer = null;
      }}

      const bounds = [];
      const defaultBounds = [];
      const mapEntry = selectedMap();
      const selectedNavsat = runReviewRoute || mapEntry?.navsat_geojson || null;
      const selectedRoute = mapEntry?.route_geojson || null;
      const activeLatLngs = leafletLatLngs(data.active_navsat_geojson);
      if (layerEnabled('recorded') && activeLatLngs.length > 1) {{
        activePolyline = L.polyline(activeLatLngs, {{ color: '#dc2626', weight: 4 }}).addTo(map);
        bounds.push(...activeLatLngs);
      }}

      const latestLatLngs = leafletLatLngs(selectedNavsat || data.latest_navsat_geojson);
      defaultBounds.push(...latestLatLngs);
      if (layerEnabled('recorded') && latestLatLngs.length > 1) {{
        latestPolyline = L.polyline(latestLatLngs, {{ color: '#0f766e', weight: 4 }}).addTo(map);
        bounds.push(...latestLatLngs);
      }}

      const routeLatLngs = leafletLatLngs(selectedRoute || data.latest_route_geojson);
      const perimeterLatLngs = routeLatLngs.length > 0 ? routeLatLngs : latestLatLngs;
      defaultBounds.push(...perimeterLatLngs);
      if (layerEnabled('boundary') && perimeterLatLngs.length > 1) {{
        perimeterPolyline = L.polyline(perimeterLatLngs, {{ color: '#f59e0b', weight: 3, dashArray: '8 8' }}).addTo(map);
        boundaryMaskLayer = L.polygon(perimeterLatLngs, {{
          color: '#fdca0f',
          weight: 1,
          fillColor: '#020617',
          fillOpacity: 0.04,
        }}).addTo(map);
        boundaryMaskLayer.bringToBack();
      }}

      const plannedLatLngs = plannedPathSegment(perimeterLatLngs);
      if (layerEnabled('planned') && plannedLatLngs.length > 1) {{
        plannedPathPolyline = L.polyline(plannedLatLngs, {{
          color: '#fdca0f',
          weight: 5,
          opacity: 0.96,
        }}).addTo(map);
        plannedPathPolyline.bringToFront();
        bounds.push(...plannedLatLngs);
      }}
      drawStartFinish(plannedLatLngs, bounds);
      drawSemanticObjects(mapEntry, bounds);

      if (layerEnabled('obstacles')) {{
        const obstaclePoints = mapEntry?.recorded_obstacle_points || data.latest_recorded_map?.recorded_obstacle_points || [];
        obstacleLayer = L.layerGroup();
        for (const point of obstaclePoints) {{
          const lat = Number(point.latitude ?? point.lat ?? point.y);
          const lon = Number(point.longitude ?? point.lon ?? point.lng ?? point.x);
          if (Number.isFinite(lat) && Number.isFinite(lon)) {{
            L.circleMarker([lat, lon], {{
              radius: 4,
              color: '#101214',
              weight: 1,
              fillColor: '#ff7b5c',
              fillOpacity: 0.88,
            }}).bindTooltip('Obstacle').addTo(obstacleLayer);
            bounds.push([lat, lon]);
          }}
        }}
        if (obstacleLayer.getLayers().length) {{
          obstacleLayer.addTo(map);
        }}
      }}

      if (layerEnabled('gaussian')) {{
        const obstaclePoints = mapEntry?.recorded_obstacle_points || data.latest_recorded_map?.recorded_obstacle_points || [];
        const gaussianLatLngs = latestLatLngs.length > 0 ? latestLatLngs : perimeterLatLngs;
        if (obstaclePoints.length > 0 && gaussianLatLngs.length > 0) {{
          gaussianLayer = L.layerGroup();
          for (const point of gaussianLatLngs.slice(0, Math.min(gaussianLatLngs.length, 120))) {{
            L.circleMarker(point, {{
              radius: 2,
              color: '#38bdf8',
              weight: 1,
              fillColor: '#38bdf8',
              fillOpacity: 0.55,
            }}).addTo(gaussianLayer);
          }}
          gaussianLayer.addTo(map);
        }}
      }}

      const position = data.current_position;
      if (layerEnabled('robot') && position && position.latitude !== undefined && position.longitude !== undefined) {{
        currentMarker = L.circleMarker(
          [Number(position.latitude), Number(position.longitude)],
          {{ radius: 6, color: '#ffffff', weight: 2, fillColor: '#1d4ed8', fillOpacity: 1 }}
        ).addTo(map);
        bounds.push([Number(position.latitude), Number(position.longitude)]);
        if (defaultBounds.length === 0) {{
          defaultBounds.push([Number(position.latitude), Number(position.longitude)]);
        }}
      }}

      fitMapToDefaultBounds(data, defaultBounds.length > 0 ? defaultBounds : bounds);
      renderSplatPreview(data);
    }}

    function gaussianSplatManifest(data) {{
      return selectedMap()?.gaussian_splat_manifest || data.latest_recorded_map?.gaussian_splat_manifest || null;
    }}

    function gaussianTileState(tile) {{
      return String(tile?.state || tile?.status || 'pending');
    }}

    function gaussianTilePercent(tile) {{
      const quality = Number(tile?.quality_progress_percent);
      if (Number.isFinite(quality) && quality > 0) {{
        return Math.max(0, Math.min(100, quality));
      }}
      const iterations = Number(tile?.latest_checkpoint_iteration || tile?.iterations || 0);
      const scopedStatus = gaussianStatusForSelectedSource();
      const target = Number(tile?.target_iterations || scopedStatus?.target_iterations_per_tile || 0);
      if (target > 0) {{
        return Math.max(0, Math.min(100, (iterations / target) * 100));
      }}
      return gaussianTileState(tile) === 'completed' ? 100 : 0;
    }}

    function gaussianStatusTiles(data) {{
      const scopedStatus = gaussianStatusForSelectedSource();
      const statusTiles = scopedStatus?.tiles || [];
      if (statusTiles.length) {{
        return statusTiles;
      }}
      const manifest = gaussianSplatManifest(data);
      return manifest?.tiles || [];
    }}

    function gaussianStateColor(state) {{
      if (state === 'completed') {{
        return {{ fill: 'rgba(34, 197, 94, 0.58)', stroke: 'rgba(134, 239, 172, 0.95)' }};
      }}
      if (state === 'running' || state === 'starting') {{
        return {{ fill: 'rgba(56, 189, 248, 0.52)', stroke: 'rgba(125, 211, 252, 0.96)' }};
      }}
      if (state === 'paused' || state === 'pause_requested' || state === 'partial') {{
        return {{ fill: 'rgba(253, 202, 15, 0.50)', stroke: 'rgba(254, 240, 138, 0.96)' }};
      }}
      if (state === 'failed' || state === 'rejected') {{
        return {{ fill: 'rgba(248, 113, 113, 0.54)', stroke: 'rgba(252, 165, 165, 0.98)' }};
      }}
      return {{ fill: 'rgba(148, 163, 184, 0.40)', stroke: 'rgba(203, 213, 225, 0.82)' }};
    }}

    function updateGaussianOverlaySummary(status, tiles) {{
      const state = status?.state || (tiles.length ? 'available' : 'idle');
      const overall = Number(status?.overall_progress_percent || 0);
      const tileCount = Number(status?.tile_count || tiles.length || 0);
      const completed = Number(
        status?.completed_tile_count ||
        tiles.filter((tile) => gaussianTileState(tile) === 'completed').length
      );
      const currentTile = status?.current_tile_id ? ` | current ${{status.current_tile_id}}` : '';
      const checkpoint = status?.latest_checkpoint_iteration
        ? ` | checkpoint ${{status.latest_checkpoint_iteration}}/${{status.target_iterations_per_tile || '-'}}`
        : '';
      const updated = status?.manifest_updated_at || status?.updated_at || '';
      const message = status?.message ? ` | ${{status.message}}` : '';
      gaussianOverlaySummary.classList.toggle('failed', state === 'failed' || state === 'rejected');
      gaussianOverlaySummary.innerHTML = `
        <strong>3D environment ${{state}}</strong>
        <span>${{completed}}/${{tileCount}} tiles · ${{overall.toFixed(1)}}%${{currentTile}}${{checkpoint}}</span>
        <span style="display:block; margin-top: 4px;">${{updated ? `Updated ${{updated}}` : 'Waiting for first checkpoint.'}}${{message}}</span>
      `;
    }}

    function renderSplatPreview(data) {{
      const context = splatCanvas.getContext('2d');
      const scopedStatus = gaussianStatusForSelectedSource();
      const rect = splatViewElement.getBoundingClientRect();
      const width = Math.max(320, Math.floor(rect.width || splatViewElement.clientWidth || 640));
      const height = Math.max(320, Math.floor(rect.height || 620));
      if (splatCanvas.width !== width || splatCanvas.height !== height) {{
        splatCanvas.width = width;
        splatCanvas.height = height;
      }}
      context.clearRect(0, 0, width, height);
      context.fillStyle = '#07090a';
      context.fillRect(0, 0, width, height);
      const tiles = gaussianStatusTiles(data);
      updateGaussianOverlaySummary(scopedStatus, tiles);
      if (!tiles.length) {{
        context.fillStyle = '#c4bb98';
        context.font = '16px Avenir Next, Segoe UI, sans-serif';
        context.fillText(scopedStatus?.message || 'No 3D environment tiles available yet.', 24, 40);
        return;
      }}
      const bounds = tiles.reduce((acc, tile) => {{
        const b = tile.bounds || {{}};
        return {{
          minX: Math.min(acc.minX, Number(b.min_x ?? 0)),
          minY: Math.min(acc.minY, Number(b.min_y ?? 0)),
          maxX: Math.max(acc.maxX, Number(b.max_x ?? 0)),
          maxY: Math.max(acc.maxY, Number(b.max_y ?? 0)),
        }};
      }}, {{ minX: Infinity, minY: Infinity, maxX: -Infinity, maxY: -Infinity }});
      const spanX = Math.max(1, bounds.maxX - bounds.minX);
      const spanY = Math.max(1, bounds.maxY - bounds.minY);
      const margin = 42;
      for (const tile of tiles) {{
        const b = tile.bounds || {{}};
        const state = gaussianTileState(tile);
        const colors = gaussianStateColor(state);
        const percent = gaussianTilePercent(tile);
        const x0 = margin + ((Number(b.min_x ?? 0) - bounds.minX) / spanX) * (width - margin * 2);
        const x1 = margin + ((Number(b.max_x ?? 0) - bounds.minX) / spanX) * (width - margin * 2);
        const y0 = height - margin - ((Number(b.min_y ?? 0) - bounds.minY) / spanY) * (height - margin * 2);
        const y1 = height - margin - ((Number(b.max_y ?? 0) - bounds.minY) / spanY) * (height - margin * 2);
        const tileWidth = Math.max(46, x1 - x0);
        const tileHeight = Math.max(34, y0 - y1);
        context.fillStyle = colors.fill;
        context.strokeStyle = colors.stroke;
        context.lineWidth = state === 'running' ? 3 : 1.5;
        context.fillRect(x0, y1, tileWidth, tileHeight);
        context.strokeRect(x0, y1, tileWidth, tileHeight);
        context.fillStyle = '#f8fafc';
        context.font = '12px Avenir Next, Segoe UI, sans-serif';
        const iteration = Number(tile.latest_checkpoint_iteration || tile.iterations || 0);
        const target = Number(tile.target_iterations || scopedStatus?.target_iterations_per_tile || 0);
        context.fillText(`${{Math.round(percent)}}%`, x0 + 6, y1 + 16);
        context.fillText(`${{iteration}}/${{target || '-'}}`, x0 + 6, y1 + 31);
      }}
      context.fillStyle = '#f5f1df';
      context.font = '15px Avenir Next, Segoe UI, sans-serif';
      context.fillText(`${{tiles.length}} 3D environment tile${{tiles.length === 1 ? '' : 's'}}`, 24, 32);
    }}

    function formatGaussianStatus(status) {{
      if (!status || Object.keys(status).length === 0) {{
        return '3D environment idle.';
      }}
      const state = status.state || 'idle';
      const message = status.message || '';
      const tiles = Number(status.tile_count || 0);
      const completed = Number(status.completed_tile_count || 0);
      const progress = Number(status.overall_progress_percent || 0);
      const currentTile = status.current_tile_id ? ` · tile ${{status.current_tile_id}}` : '';
      const checkpoint = status.latest_checkpoint_iteration
        ? ` · checkpoint ${{status.latest_checkpoint_iteration}}/${{status.target_iterations_per_tile || '-'}}`
        : '';
      return `${{state}} · ${{completed}}/${{tiles}} environment tiles · ${{progress.toFixed(1)}}%${{currentTile}}${{checkpoint}}${{message ? ` · ${{message}}` : ''}}`;
    }}

    function updateGaussianControls(status) {{
      latestGaussianStatus = status || latestGaussianStatus || {{}};
      const scopedStatus = gaussianStatusForSelectedSource();
      currentGaussianBuildId = scopedStatus.build_id || '';
      const state = scopedStatus.state || 'idle';
      const canResume = Boolean(scopedStatus.can_resume) || ['paused', 'partial', 'failed'].includes(state);
      buildGaussianButton.disabled = gaussianRequestInFlight || defaultMissionIds.has(selectedMapId);
      buildGaussianButton.classList.toggle('stop', state === 'running' || state === 'pause_requested');
      buildGaussianButton.classList.toggle('secondary', !(state === 'running' || state === 'pause_requested'));
      if (gaussianRequestInFlight) {{
        buildGaussianButton.textContent = 'Working...';
      }} else if (state === 'running' || state === 'pause_requested') {{
        buildGaussianButton.textContent = 'Stop build';
      }} else if (canResume) {{
        buildGaussianButton.textContent = 'Resume building 3D environment';
      }} else {{
        buildGaussianButton.textContent = 'Build 3D environment';
      }}
      gaussianBuildStatus.textContent = formatGaussianStatus(scopedStatus);
      renderSplatPreview(lastMapSnapshot);
      updateMissionActionState();
    }}

    async function refreshGaussianStatus() {{
      const response = await fetch('/api/v1/gaussian-splats/status', {{ cache: 'no-store' }});
      const data = await response.json();
      if (!response.ok || data.success === false) {{
        throw new Error(data.message || `Gaussian status failed with HTTP ${{response.status}}`);
      }}
      updateGaussianControls(data.status || {{}});
      return latestGaussianStatus;
    }}

    function setSelectedMapId(nextMapId) {{
      selectedMapId = nextMapId || '';
      if (selectedMapId) {{
        window.localStorage.setItem(selectedMapStorageKey, selectedMapId);
      }} else {{
        window.localStorage.removeItem(selectedMapStorageKey);
      }}
    }}

    function selectedSourceDisplayName() {{
      const entry = selectedMap();
      const mission = selectedMission();
      if (selectedMapId === 'RecordMap') {{
        return latestRecordingLabel;
      }}
      return entry ? (entry.name || entry.map_id) : (mission?.mission_id || latestRecordingLabel);
    }}

    function closeMapCombo() {{
      mapCombo.classList.remove('open');
    }}

    function selectMapSource(mapId) {{
      setSelectedMapId(mapId);
      missionSearchText = '';
      mapNameInput.value = '';
      mapNameTouched = false;
      lastAppliedSourceMapId = selectedMapId;
      recordRosbagToggle.checked = selectedMapId ? loadRecordRosbagPreference(selectedMapId) : false;
      closeMapCombo();
      closeDrawers();
    }}

    function missionLabel(entry) {{
      const missionId = entry?.mission_id || entry?.map_id || '';
      return missionId === 'RecordMap' ? 'Latest Recording' : (entry?.name || missionId);
    }}

    function missionDetail(entry) {{
      const parts = [];
      const missionId = entry?.mission_id || entry?.map_id || '';
      if (missionIsProtected(entry)) {{
        parts.push(missionId === 'RecordMap' ? 'Latest recording' : 'Read-only');
      }} else {{
        parts.push('Saved mission');
      }}
      parts.push(missionMetrics(entry));
      return parts.join(' | ');
    }}

    function appendMapComboOption(mapId, label, detail = '') {{
      const option = document.createElement('button');
      option.type = 'button';
      option.className = 'map-combo-option';
      option.innerHTML = `<strong>${{label}}</strong><span>${{missionUiStatus(selectedMapById(mapId))}} | ${{detail}}</span>`;
      option.classList.toggle('active', mapId === selectedMapId);
      option.addEventListener('click', async () => {{
        selectMapSource(mapId);
        await loadRecordMapSnapshot();
      }});
      mapComboList.appendChild(option);
    }}

    function selectedMapById(mapId) {{
      return mapsCache.find((entry) => entry.map_id === mapId) || null;
    }}

    function populateMapSelect() {{
      mapComboList.innerHTML = '';
      const search = missionSearchText.trim().toLowerCase();
      const groups = new Map([
        ['Latest Recording', []],
        ['Default Missions', []],
        ['Saved Missions', []],
      ]);
      for (const entry of mapsCache) {{
        const text = `${{entry.map_id || ''}} ${{entry.mission_id || ''}} ${{entry.name || ''}}`.toLowerCase();
        if (search && !text.includes(search)) {{
          continue;
        }}
        if (!statusMatchesFilter(entry)) {{
          continue;
        }}
        groups.get(selectedMissionGroup(entry))?.push(entry);
      }}
      for (const [groupName, entries] of groups.entries()) {{
        if (!entries.length) {{
          continue;
        }}
        const groupHeading = document.createElement('h3');
        groupHeading.textContent = groupName;
        groupHeading.style.marginTop = mapComboList.children.length ? '12px' : '0';
        mapComboList.appendChild(groupHeading);
        entries.sort((left, right) => missionLabel(left).localeCompare(missionLabel(right)));
        for (const entry of entries) {{
          appendMapComboOption(entry.map_id, missionLabel(entry), missionDetail(entry));
        }}
      }}
      if (mapsCache.length === 0) {{
        latestMapMessage.textContent = 'No mission artifacts are available yet.';
      }} else if (!mapComboList.children.length) {{
        mapComboList.innerHTML = '<div class="muted">No missions match the current filter.</div>';
      }}
    }}

    function applySelectedMapToEditor() {{
      const entry = selectedMap();
      const sourceChanged = selectedMapId !== lastAppliedSourceMapId;
      if (!mapNameTouched || sourceChanged) {{
        if (sourceChanged) {{
          mapNameInput.value = '';
          missionSearchText = '';
          runReviewRoute = null;
          runReviewBanner.classList.remove('show');
        }}
        mapNameTouched = false;
        lastAppliedSourceMapId = selectedMapId;
      }}
      if (entry) {{
        applyLayerVisibility(entry.layer_visibility || {{}});
        const pattern = patternInputs.find((input) => input.value === (entry.sweep_pattern || 'zigzag'));
        if (pattern) {{
          pattern.checked = true;
        }}
        startPositionInput.value = String(entry.start_position?.percent ?? 0);
        useEndPositionInput.checked = Boolean(entry.end_position);
        endPositionInput.value = String(entry.end_position?.percent ?? 100);
      }}
      updateMissionActionState();
      updateMissionSummary();
    }}

    function updateMissionActionState() {{
      const entry = selectedMap();
      const protectedSelection = missionIsProtected(entry);
      const isRecordMap = selectedMapId === 'RecordMap';
      const isDefault = defaultMissionIds.has(selectedMapId);
      const canSaveAs = Boolean(selectedMapId) && !isDefault;
      const canRename = selectedMissionIsSavedEditable();
      const canDelete = selectedMissionIsSavedEditable();
      const canEditPath = selectedMissionIsSavedEditable();
      document.getElementById('open-area-edit-button').disabled = !canEditPath;
      saveMapButton.disabled = !canSaveAs;
      renameMissionButton.disabled = !canRename;
      deleteMapButton.disabled = !canDelete;
      buildGaussianButton.disabled = gaussianRequestInFlight || isDefault;
      startMissionButton.disabled = !selectedMapId;
      recordRosbagToggle.disabled = !selectedMapId;
      for (const input of patternInputs) {{
        input.disabled = !canEditPath;
      }}
      startPositionInput.disabled = !canEditPath;
      useEndPositionInput.disabled = !canEditPath;
      endPositionInput.disabled = !canEditPath || !useEndPositionInput.checked;
      document.getElementById('save-button').disabled = !canEditPath;
      if (isRecordMap) {{
        latestMapMessage.textContent = entry?.artifacts_ready
          ? 'Latest recording can be saved as a new mission. Rename and delete are disabled for RecordMap.'
          : 'RecordMap is selectable, but no reusable map artifacts are available yet.';
      }} else if (isDefault) {{
        latestMapMessage.textContent = 'Default missions are preview-only here. Start is available; save, rename, and delete are disabled.';
      }} else if (protectedSelection) {{
        latestMapMessage.textContent = 'This mission is preview-only.';
      }}
    }}

    function updateMissionSummary() {{
      const entry = selectedMap();
      const title = selectedSourceDisplayName();
      const status = missionUiStatus(entry);
      const metrics = missionMetrics(entry);
      selectedMissionTitle.textContent = 'Missions';
      selectedMissionState.textContent = status;
      sheetMissionTitle.textContent = title;
      sheetMissionMeta.textContent = `${{status}} | ${{metrics}}`;
      missionDetailGrid.innerHTML = [
        detailRow('Status', status),
        detailRow('Area', entry?.area_square_meters ? `${{Math.round(Number(entry.area_square_meters)).toLocaleString()}} m2` : '-'),
        detailRow('Path length', entry?.path_length_meters ? `${{Math.round(Number(entry.path_length_meters)).toLocaleString()}} m` : '-'),
        detailRow('Estimated duration', entry?.estimated_duration_seconds ? `${{Math.round(Number(entry.estimated_duration_seconds) / 60)}} min` : '-'),
        detailRow('Work areas', String((entry?.semantic_zones || []).filter((zone) => zone.type === 'WORK_AREA').length)),
        detailRow('No-go areas', String((entry?.semantic_zones || []).filter((zone) => zone.type === 'NO_GO').length)),
        detailRow('Mission type', entry?.mission_type || (missionIsProtected(entry) ? 'Protected' : 'Saved')),
        detailRow('Artifacts', entry?.artifacts_ready === false ? 'Processing' : 'Ready'),
      ].join('');
      const settings = entry?.mission_settings || {{}};
      missionSettingsGrid.innerHTML = [
        detailRow('Pattern', settings.pattern || entry?.sweep_pattern || 'zigzag'),
        detailRow('Robot speed', settings.robot_speed || 'standard'),
        detailRow('Sweep intensity', settings.sweep_intensity || 'standard'),
        detailRow('Edge sweep', settings.edge_sweep === false ? 'Off' : 'On'),
        detailRow('Tool', settings.tool?.enabled === false ? 'Off' : 'On'),
      ].join('');
      advancedDetailGrid.innerHTML = [
        detailRow('Mission ID', entry?.mission_id || entry?.map_id || '-'),
        detailRow('Directory', entry?.directory || '-'),
        detailRow('Execution mode', entry?.execution_mode || '-'),
        detailRow('Running profile', entry?.running_profile_id || '-'),
        detailRow('Route available', entry?.route_geojson || entry?.navsat_geojson ? 'Yes' : 'No'),
        detailRow('3D manifest', entry?.gaussian_splat_manifest_file || entry?.gaussian_manifest_file || '-'),
      ].join('');
      renderRunList(entry);
    }}

    function renderRunList(entry) {{
      const runs = entry?.run_history || [];
      if (!runs.length) {{
        runList.innerHTML = '<div class="muted">No recorded runs available.</div>';
        return;
      }}
      runList.innerHTML = '';
      for (const run of runs) {{
        const button = document.createElement('button');
        button.type = 'button';
        button.className = 'run-row';
        const started = run.run_started_at || run.started_at || 'Recorded run';
        const state = run.status || run.outcome || 'Completed';
        const length = Number(run.actual_path_length_meters || 0);
        button.innerHTML = `<strong>${{started}}</strong><span>${{state}}${{length > 0 ? ` | ${{Math.round(length)}} m` : ''}}</span>`;
        button.addEventListener('click', () => {{
          runReviewRoute = run.actual_path_navsat_geojson || run.actual_path_geojson || null;
          runReviewLabel.textContent = `Reviewing run | ${{started}}`;
          runReviewBanner.classList.add('show');
          setSheetState('collapsed');
          setBanner('ok', `Reviewing run ${{started}}`);
          updateMap(lastMapSnapshot);
        }});
        runList.appendChild(button);
      }}
    }}

    function editorStateKey(data) {{
      const latestRunId = String(data.latest_recorded_map?.run_started_at || '');
      return selectedMapId ? `mission:${{selectedMapId}}:${{latestRunId}}` : `latest:${{latestRunId}}`;
    }}

    async function fetchJson(path) {{
      const response = await fetch(path, {{ cache: 'no-store' }});
      const data = await response.json();
      if (!response.ok || data.success === false) {{
        throw new Error(data.message || `${{path}} failed with HTTP ${{response.status}}`);
      }}
      return data;
    }}

    function normalizeMissionEntry(entry) {{
      const missionId = String(entry?.mission_id || entry?.map_id || '').trim();
      return {{
        ...entry,
        map_id: missionId,
        mission_id: missionId,
        name: entry?.name || (missionId === 'RecordMap' ? latestRecordingLabel : missionId),
        readonly: Boolean(entry?.readonly) || protectedMissionIds.has(missionId) || defaultMissionIds.has(missionId),
      }};
    }}

    function mergeMissionAndMapEntries(missionData, mapData, previewData) {{
      const byId = new Map();
      const latest = mapData.latest_recorded_map || null;
      if (latest && !latest.error) {{
        byId.set('RecordMap', normalizeMissionEntry({{
          ...latest,
          map_id: 'RecordMap',
          mission_id: 'RecordMap',
          name: latestRecordingLabel,
          readonly: true,
          artifacts_ready: true,
        }}));
      }} else {{
        byId.set('RecordMap', normalizeMissionEntry({{
          map_id: 'RecordMap',
          mission_id: 'RecordMap',
          name: latestRecordingLabel,
          readonly: true,
          artifacts_ready: false,
        }}));
      }}

      for (const mission of missionData.missions || []) {{
        const normalized = normalizeMissionEntry(mission);
        if (!missionIsSelectable(normalized)) {{
          continue;
        }}
        byId.set(normalized.mission_id, {{
          ...(byId.get(normalized.mission_id) || {{}}),
          ...normalized,
        }});
      }}
      for (const mission of previewData.missions || []) {{
        const normalized = normalizeMissionEntry(mission);
        if (!missionIsSelectable(normalized)) {{
          continue;
        }}
        const existing = byId.get(normalized.mission_id) || {{}};
        byId.set(normalized.mission_id, {{
          ...existing,
          ...normalized,
          readonly: existing.readonly || normalized.readonly,
        }});
      }}

      for (const mapEntry of mapData.maps || []) {{
        const normalized = normalizeMissionEntry(mapEntry);
        if (!missionIsSelectable(normalized)) {{
          continue;
        }}
        const existing = byId.get(normalized.map_id) || {{}};
        byId.set(normalized.map_id, {{
          ...existing,
          ...normalized,
          readonly: existing.readonly || normalized.readonly,
        }});
      }}

      return [...byId.values()].filter((entry) => entry.map_id && missionIsSelectable(entry));
    }}

    async function loadRecordMapSnapshot() {{
      const [data, missionData, previewData] = await Promise.all([
        fetchJson('/api/v1/maps'),
        fetchJson('/api/v1/missions').catch(() => ({{ missions: [] }})),
        fetchJson('/api/v1/map-data').catch(() => ({{ missions: [] }})),
      ]);
      lastMapSnapshot = data;
      missionsCache = missionData.missions || [];
      mapsCache = mergeMissionAndMapEntries(missionData, data, previewData);
      if (selectedMapId && !mapsCache.some((entry) => entry.map_id === selectedMapId)) {{
        setSelectedMapId('RecordMap');
        setBanner('warn', 'Previously selected saved map is no longer available.');
      }}
      if (!selectedMapId) {{
        setSelectedMapId('RecordMap');
      }}
      populateMapSelect();
      const nextEditorStateKey = editorStateKey(data);
      if (nextEditorStateKey !== appliedEditorStateKey) {{
        applySelectedMapToEditor();
        appliedEditorStateKey = nextEditorStateKey;
      }}
      chip.textContent = data.active_recording ? 'RecordMap recording' : 'RecordMap idle';
      chip.className = data.active_recording ? 'status-chip' : 'status-chip idle';

      const latest = data.latest_recorded_map;
      if (latest && !latest.error) {{
        latestRun.textContent = latest.run_started_at || 'Latest recording available';
        latestObstacles.textContent = String(latest.recorded_obstacle_count ?? '-');
        const scopedStatus = gaussianStatusForSelectedSource();
        if (!scopedStatus?.state || scopedStatus.state === 'idle') {{
          const splat = selectedMap()?.gaussian_splat_manifest || latest.gaussian_splat_manifest;
          gaussianBuildStatus.textContent = splat
            ? `3D environment ready: ${{splat.tile_count || 0}} tile(s).`
            : 'Capture ready for manual 3D environment build.';
        }}
        if (!missionIsProtected(selectedMap())) {{
          latestMapMessage.textContent = 'Editing saved mission map metadata and display preferences.';
        }}
        const latestRunId = String(latest.run_started_at || '');
        if (latestRunId && latestRunId !== lastLatestRunId) {{
          startPatternCountdown();
          lastLatestRunId = latestRunId;
        }}
      }} else {{
        latestRun.textContent = 'No recording captured yet.';
        latestObstacles.textContent = '-';
        gaussianBuildStatus.textContent = '3D environment idle.';
        if (!missionIsProtected(selectedMap())) {{
          latestMapMessage.textContent = 'Editing saved mission map metadata and display preferences.';
        }}
        window.clearInterval(countdownTimer);
        patternCountdown.textContent = '';
        lastLatestRunId = '';
      }}

      updateMap(data);
      updateLayerPanelMode();
      updateGaussianControls(latestGaussianStatus);
      updateMissionActionState();
      return data;
    }}

    async function postJson(path, body) {{
      const response = await fetch(path, {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/json' }},
        body: JSON.stringify(body || {{}})
      }});
      const data = await response.json();
      if (!response.ok || data.success === false) {{
        throw new Error(data.message || `${{path}} failed with HTTP ${{response.status}}`);
      }}
      return data;
    }}

    async function refreshTopbarStatus() {{
      try {{
        const response = await fetch('/api/v1/status', {{ cache: 'no-store' }});
        const data = await response.json();
        const missionState = selectedMapId ? (selectedMap()?.name || selectedMapId) : 'Select mission';
        updateTopbarFromStatus(data, missionState);
      }} catch (_error) {{
        markTopbarDisconnected();
      }}
    }}

    function selectedMissionEditableMessage() {{
      return selectedMissionIsSavedEditable()
        ? ''
        : 'Only saved missions can be edited. Default missions and the latest recording are preview-only.';
    }}

    function setAreaEditMessage(message) {{
      areaEditMessage.textContent = message;
    }}

    function redrawDraftArea() {{
      if (draftAreaLayer) {{
        map.removeLayer(draftAreaLayer);
        draftAreaLayer = null;
      }}
      if (!draftAreaPoints.length) {{
        areaPointList.textContent = 'No points selected.';
        return;
      }}
      areaPointList.textContent = `${{draftAreaPoints.length}} point${{draftAreaPoints.length === 1 ? '' : 's'}} selected.`;
      const latlngs = draftAreaPoints.map((point) => [point.lat, point.lng]);
      draftAreaLayer = L.layerGroup();
      if (activeAreaTool === 'TRANSIT') {{
        if (latlngs.length > 1) {{
          L.polyline(latlngs, zoneStyle('TRANSIT')).addTo(draftAreaLayer);
        }}
      }} else if (activeAreaTool === 'STATION') {{
        L.circleMarker(latlngs[latlngs.length - 1], {{
          radius: 8,
          color: '#ffffff',
          weight: 2,
          fillColor: '#38bdf8',
          fillOpacity: 1,
        }}).addTo(draftAreaLayer);
      }} else if (latlngs.length > 1) {{
        L.polygon(latlngs, zoneStyle(activeAreaTool || 'WORK_AREA')).addTo(draftAreaLayer);
      }}
      for (const point of draftAreaPoints) {{
        const marker = L.marker([point.lat, point.lng], {{
          draggable: true,
          icon: L.divIcon({{ className: 'zone-dot', iconSize: [16, 16] }}),
        }});
        marker.on('dragend', () => {{
          const next = marker.getLatLng();
          point.lat = next.lat;
          point.lng = next.lng;
          redrawDraftArea();
        }});
        marker.on('click', () => {{
          draftAreaPoints = draftAreaPoints.filter((candidate) => candidate !== point);
          redrawDraftArea();
        }});
        marker.addTo(draftAreaLayer);
      }}
      draftAreaLayer.addTo(map);
    }}

    function resetAreaDraft() {{
      draftAreaPoints = [];
      editingZoneId = '';
      areaNameInput.value = '';
      if (draftAreaLayer) {{
        map.removeLayer(draftAreaLayer);
        draftAreaLayer = null;
      }}
      redrawDraftArea();
    }}

    function enterAreaEditMode(tool = '') {{
      const message = selectedMissionEditableMessage();
      if (message) {{
        setBanner('error', message);
        return;
      }}
      areaEditMode = true;
      activeAreaTool = tool || activeAreaTool || 'WORK_AREA';
      areaEditToolbar.classList.add('open');
      openSideSheet(areaEditSheet);
      for (const button of document.querySelectorAll('[data-area-tool]')) {{
        button.classList.toggle('active', button.dataset.areaTool === activeAreaTool);
      }}
      setAreaEditMessage(activeAreaTool === 'STATION'
        ? 'Tap the map to place the station.'
        : 'Tap the map to add points. Tap a point marker to remove it, or drag it to adjust.');
    }}

    function exitAreaEditMode() {{
      areaEditMode = false;
      activeAreaTool = '';
      areaEditToolbar.classList.remove('open');
      resetAreaDraft();
      closeOverlayPanels();
    }}

    function beginEditExistingZone(zone) {{
      if (!areaEditMode || !selectedMissionIsSavedEditable()) {{
        return;
      }}
      editingZoneId = zone.id || '';
      activeAreaTool = zone.type || 'WORK_AREA';
      areaNameInput.value = zone.name || '';
      draftAreaPoints = geometryLatLngs(zone.geometry).map((point) => ({{ lat: point[0], lng: point[1] }}));
      if (activeAreaTool !== 'TRANSIT' && draftAreaPoints.length > 1) {{
        const first = draftAreaPoints[0];
        const last = draftAreaPoints[draftAreaPoints.length - 1];
        if (Math.abs(first.lat - last.lat) < 1e-9 && Math.abs(first.lng - last.lng) < 1e-9) {{
          draftAreaPoints.pop();
        }}
      }}
      enterAreaEditMode(activeAreaTool);
      redrawDraftArea();
      setAreaEditMessage('Editing selected area. Drag points to adjust or tap a point to remove it.');
    }}

    function draftGeometry() {{
      if (activeAreaTool === 'STATION') {{
        const point = draftAreaPoints[draftAreaPoints.length - 1];
        if (!point) {{
          throw new Error('Tap the map to place the station.');
        }}
        return {{ position: {{ x: point.lng, y: point.lat }} }};
      }}
      const coordinates = draftAreaPoints.map((point) => [point.lng, point.lat]);
      if (activeAreaTool === 'TRANSIT') {{
        if (coordinates.length < 2) {{
          throw new Error('Transit path needs at least two points.');
        }}
        return {{ type: 'LineString', coordinates }};
      }}
      if (coordinates.length < 3) {{
        throw new Error('Area needs at least three points.');
      }}
      coordinates.push([...coordinates[0]]);
      return {{ type: 'Polygon', coordinates: [coordinates] }};
    }}

    async function reloadAfterAreaEdit(message) {{
      setBanner('ok', message);
      resetAreaDraft();
      await loadRecordMapSnapshot();
      enterAreaEditMode(activeAreaTool || 'WORK_AREA');
    }}

    async function saveDraftArea() {{
      try {{
        if (!selectedMissionIsSavedEditable()) {{
          throw new Error(selectedMissionEditableMessage());
        }}
        const geometry = draftGeometry();
        if (activeAreaTool === 'STATION') {{
          const data = await postJson(`/api/v1/missions/${{encodeURIComponent(selectedMapId)}}/station`, {{
            station: {{
              name: areaNameInput.value || 'Station',
              enabled: true,
              ...geometry,
            }},
          }});
          await reloadAfterAreaEdit(data.message || 'Station saved');
          return;
        }}
        const body = {{
          zone_id: editingZoneId,
          name: areaNameInput.value,
          type: activeAreaTool,
          enabled: true,
          geometry,
        }};
        const path = editingZoneId
          ? `/api/v1/missions/${{encodeURIComponent(selectedMapId)}}/zones/${{encodeURIComponent(editingZoneId)}}`
          : `/api/v1/missions/${{encodeURIComponent(selectedMapId)}}/zones`;
        const data = await postJson(path, body);
        await reloadAfterAreaEdit(data.message || 'Area saved');
      }} catch (error) {{
        setBanner('error', error.message || 'Area save failed');
        setAreaEditMessage(error.message || 'Area save failed');
      }}
    }}

    async function deleteDraftArea() {{
      if (!editingZoneId) {{
        resetAreaDraft();
        setAreaEditMessage('Draft area cleared.');
        return;
      }}
      try {{
        const data = await postJson(
          `/api/v1/missions/${{encodeURIComponent(selectedMapId)}}/zones/${{encodeURIComponent(editingZoneId)}}/delete`,
          {{}}
        );
        await reloadAfterAreaEdit(data.message || 'Area deleted');
      }} catch (error) {{
        setBanner('error', error.message || 'Area delete failed');
      }}
    }}

    document.getElementById('open-nav-button').addEventListener('click', () => openDrawer(navDrawer));
    document.getElementById('close-nav-button').addEventListener('click', closeDrawers);
    document.getElementById('mission-title-button').addEventListener('click', () => openDrawer(missionDrawer));
    document.getElementById('close-mission-drawer-button').addEventListener('click', closeDrawers);
    drawerBackdrop.addEventListener('click', closeDrawers);
    document.getElementById('mission-menu-button').addEventListener('click', () => {{
      closeDrawers();
      missionActionMenu.classList.toggle('open');
    }});
    document.getElementById('open-area-edit-button').addEventListener('click', () => enterAreaEditMode('WORK_AREA'));
    document.getElementById('open-map-edit-button').addEventListener('click', () => openSideSheet(mapEditSheet));
    document.getElementById('open-path-edit-button').addEventListener('click', () => openSideSheet(pathEditSheet));
    document.getElementById('open-mission-settings-button').addEventListener('click', () => openSideSheet(missionSettingsSheet));
    document.getElementById('open-advanced-button').addEventListener('click', () => openSideSheet(advancedSheet));
    for (const button of document.querySelectorAll('.close-side-sheet-button')) {{
      button.addEventListener('click', closeOverlayPanels);
    }}
    document.getElementById('fit-mission-button').addEventListener('click', fitSelectedMission);
    document.getElementById('exit-run-review-button').addEventListener('click', exitRunReview);
    mapComboButton.addEventListener('click', () => openDrawer(missionDrawer));
    for (const button of document.querySelectorAll('[data-area-tool]')) {{
      button.addEventListener('click', () => {{
        resetAreaDraft();
        enterAreaEditMode(button.dataset.areaTool || 'WORK_AREA');
      }});
    }}
    document.getElementById('cancel-area-edit-button').addEventListener('click', exitAreaEditMode);
    document.getElementById('cancel-area-button').addEventListener('click', resetAreaDraft);
    saveAreaButton.addEventListener('click', saveDraftArea);
    deleteAreaButton.addEventListener('click', deleteDraftArea);
    map.on('click', (event) => {{
      if (!areaEditMode || !activeAreaTool || !selectedMissionIsSavedEditable()) {{
        return;
      }}
      if (activeAreaTool === 'STATION') {{
        draftAreaPoints = [{{ lat: event.latlng.lat, lng: event.latlng.lng }}];
      }} else {{
        draftAreaPoints.push({{ lat: event.latlng.lat, lng: event.latlng.lng }});
      }}
      redrawDraftArea();
    }});

    mapNameInput.addEventListener('focus', () => {{}});

    mapNameInput.addEventListener('input', () => {{
      mapNameTouched = true;
      missionSearchText = mapNameInput.value;
      populateMapSelect();
      updateMissionActionState();
    }});

    for (const chipButton of document.querySelectorAll('.filter-chip')) {{
      chipButton.addEventListener('click', () => {{
        missionStatusFilter = chipButton.dataset.filter || 'all';
        for (const other of document.querySelectorAll('.filter-chip')) {{
          other.classList.toggle('active', other === chipButton);
        }}
        populateMapSelect();
      }});
    }}

    function toggleMissionSheet() {{
      if (missionBottomSheet.classList.contains('expanded')) {{
        setSheetState('collapsed');
      }} else if (missionBottomSheet.classList.contains('medium')) {{
        setSheetState('expanded');
      }} else {{
        setSheetState('medium');
      }}
    }}
    document.getElementById('sheet-handle-button').addEventListener('click', toggleMissionSheet);
    document.getElementById('sheet-summary-toggle').addEventListener('click', toggleMissionSheet);
    document.getElementById('mission-tab-button').addEventListener('click', () => {{
      document.getElementById('mission-tab-button').classList.add('active');
      document.getElementById('runs-tab-button').classList.remove('active');
      document.getElementById('mission-tab-panel').classList.remove('hidden');
      document.getElementById('runs-tab-panel').classList.add('hidden');
      setSheetState('medium');
    }});
    document.getElementById('runs-tab-button').addEventListener('click', () => {{
      document.getElementById('runs-tab-button').classList.add('active');
      document.getElementById('mission-tab-button').classList.remove('active');
      document.getElementById('runs-tab-panel').classList.remove('hidden');
      document.getElementById('mission-tab-panel').classList.add('hidden');
      setSheetState('medium');
    }});

    saveMapButton.addEventListener('click', async () => {{
      const suggested = selectedMapId === 'RecordMap' ? '' : `${{selectedMapId}} Copy`;
      const mapName = String(window.prompt('Save as mission', suggested) || '').trim();
      if (!mapName || mapName === latestRecordingLabel) {{
        setBanner('error', 'Enter a mission name before saving.');
        return;
      }}
      if (defaultMissionIds.has(selectedMapId)) {{
        setBanner('error', 'This mission is read-only and cannot be saved over.');
        return;
      }}
      const sourceEntry = selectedMap();
      const source = selectedMapId === 'RecordMap' ? 'latest_recorded_map' : 'saved_map';
      saveMapButton.disabled = true;
      saveMapButton.textContent = 'Saving...';
      latestMapMessage.textContent = `Saving mission '${{mapName}}'...`;
      try {{
        const data = await postJson('/api/v1/maps/save', {{
          map_id: mapName,
          name: mapName,
          source,
          source_map_id: source === 'saved_map' ? (sourceEntry?.map_id || selectedMapId) : '',
          sweep_pattern: selectedPattern(),
          start_position: {{ percent: Number(startPositionInput.value) }},
          end_position: useEndPositionInput.checked ? {{ percent: Number(endPositionInput.value) }} : null,
          layer_visibility: layerVisibility(),
          overwrite_existing: true
        }});
        setSelectedMapId(data.map?.map_id || '');
        mapNameInput.value = data.map?.name || data.map?.map_id || mapName;
        mapNameTouched = false;
        lastAppliedSourceMapId = selectedMapId;
        setBanner('ok', data.message || `Mission '${{mapName}}' saved`);
      }} catch (error) {{
        setBanner('error', error.message || 'Save mission request failed');
        latestMapMessage.textContent = error.message || 'Save mission request failed';
      }} finally {{
        saveMapButton.disabled = false;
        saveMapButton.textContent = 'Save As Mission';
        await loadRecordMapSnapshot().catch((error) => {{
          latestMapMessage.textContent = error.message || 'Failed to reload saved maps.';
        }});
      }}
    }});

    renameMissionButton.addEventListener('click', async () => {{
      const newMissionName = String(window.prompt('Rename mission', selectedMapId) || '').trim();
      if (!selectedMissionIsSavedEditable()) {{
        setBanner('error', 'Only saved missions can be renamed.');
        return;
      }}
      if (!newMissionName || newMissionName === selectedMapId) {{
        setBanner('error', 'Enter a new mission name before renaming.');
        return;
      }}
      renameMissionButton.disabled = true;
      renameMissionButton.textContent = 'Renaming...';
      try {{
        const data = await postJson('/api/v1/missions/rename', {{
          mission_id: selectedMapId,
          new_mission_id: newMissionName,
        }});
        setSelectedMapId(data.map?.map_id || data.mission_id || newMissionName);
        mapNameInput.value = '';
        missionSearchText = '';
        mapNameTouched = false;
        setBanner('ok', data.message || 'Mission renamed');
      }} catch (error) {{
        setBanner('error', error.message || 'Rename mission request failed');
      }} finally {{
        renameMissionButton.disabled = false;
        renameMissionButton.textContent = 'Rename';
        await loadRecordMapSnapshot();
      }}
    }});

    deleteMapButton.addEventListener('click', async () => {{
      if (!selectedMissionIsSavedEditable()) {{
        setBanner('error', 'Only saved missions can be deleted.');
        return;
      }}
      const data = await postJson('/api/v1/maps/delete', {{ map_id: selectedMapId }});
      setBanner(data.success ? 'ok' : 'error', data.message || 'Delete mission request completed');
      if (data.success) {{
        setSelectedMapId('RecordMap');
        mapNameInput.value = latestRecordingLabel;
        mapNameTouched = false;
        lastAppliedSourceMapId = selectedMapId;
      }}
      await loadRecordMapSnapshot();
    }});

    recordRosbagToggle.addEventListener('change', () => {{
      if (!selectedMapId) {{
        recordRosbagToggle.checked = false;
        return;
      }}
      saveRecordRosbagPreference(selectedMapId, recordRosbagToggle.checked);
    }});

    startMissionButton.addEventListener('click', async () => {{
      if (!selectedMapId) {{
        setBanner('error', 'Select a mission before starting it.');
        return;
      }}
      updateMissionSummary();
      startConfirmationSummary.innerHTML = [
        detailRow('Mission', selectedSourceDisplayName()),
        detailRow('Status', missionUiStatus(selectedMap())),
        detailRow('Estimated work', missionMetrics(selectedMap())),
      ].join('');
      recordRosbagToggle.checked = loadRecordRosbagPreference(selectedMapId);
      startConfirmationSheet.classList.remove('hidden');
      newMissionSheet.classList.add('hidden');
      modalBackdrop.classList.add('show');
    }});

    function closeModal() {{
      modalBackdrop.classList.remove('show');
      startConfirmationSheet.classList.remove('hidden');
      newMissionSheet.classList.add('hidden');
    }}

    document.getElementById('cancel-start-button').addEventListener('click', closeModal);
    document.getElementById('cancel-start-x-button').addEventListener('click', closeModal);
    document.getElementById('new-mission-button').addEventListener('click', () => {{
      closeDrawers();
      startConfirmationSheet.classList.add('hidden');
      newMissionSheet.classList.remove('hidden');
      modalBackdrop.classList.add('show');
    }});
    document.getElementById('cancel-new-mission-button').addEventListener('click', closeModal);
    modalBackdrop.addEventListener('click', (event) => {{
      if (event.target === modalBackdrop) {{
        closeModal();
      }}
    }});

    confirmStartMissionButton.addEventListener('click', async () => {{
      if (!selectedMapId) {{
        setBanner('error', 'Select a mission before starting it.');
        return;
      }}
      saveRecordRosbagPreference(selectedMapId, recordRosbagToggle.checked);
      const response = await fetch(`/api/v1/missions/${{encodeURIComponent(selectedMapId)}}/execute`, {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/json' }},
        body: JSON.stringify({{ record_rosbag: recordRosbagToggle.checked }})
      }});
      const data = await response.json();
      setBanner(data.success ? 'ok' : 'error', data.message || 'Mission request completed');
      closeModal();
    }});

    document.getElementById('download-mission-button').addEventListener('click', () => {{
      if (!selectedMapId) {{
        setBanner('error', 'Select a mission before downloading.');
        return;
      }}
      window.location.href = `/api/v1/missions/${{encodeURIComponent(selectedMapId)}}/download`;
    }});

    document.getElementById('upload-file').addEventListener('change', async (event) => {{
      const file = event.target.files && event.target.files[0];
      if (!file) {{
        return;
      }}
      document.getElementById('upload-json').value = await file.text();
    }});

    document.getElementById('upload-button').addEventListener('click', async () => {{
      const data = await postJson('/api/v1/missions/upload-vda5050', {{
        mission_id: document.getElementById('upload-mission-id').value,
        mission_json: document.getElementById('upload-json').value,
        overwrite_existing: document.getElementById('upload-overwrite').checked,
      }});
      setBanner(data.success ? 'ok' : 'error', data.message || 'Upload completed');
      closeModal();
      await loadRecordMapSnapshot();
    }});

    buildGaussianButton.addEventListener('click', async () => {{
      if (gaussianRequestInFlight) {{
        return;
      }}
      gaussianRequestInFlight = true;
      updateGaussianControls(latestGaussianStatus);
      const scopedStatus = gaussianStatusForSelectedSource();
      const state = scopedStatus?.state || 'idle';
      try {{
        let data;
        if (state === 'running' || state === 'pause_requested') {{
          gaussianBuildStatus.textContent = 'Stopping 3D environment build...';
          data = await postJson('/api/v1/gaussian-splats/pause', {{
            build_id: currentGaussianBuildId,
            mode: 'user_pause'
          }});
        }} else if (scopedStatus?.can_resume || ['paused', 'partial', 'failed'].includes(state)) {{
          gaussianBuildStatus.textContent = 'Resuming 3D environment build...';
          data = await postJson('/api/v1/gaussian-splats/resume', {{
            build_id: currentGaussianBuildId,
            additional_iterations_per_tile: 0,
            auto_stop_enabled: false
          }});
        }} else {{
          gaussianBuildStatus.textContent = 'Starting 3D environment build...';
          const snapshot = await loadRecordMapSnapshot();
          const latest = snapshot.latest_recorded_map || {{}};
          const mapEntry = selectedMap();
          data = await postJson('/api/v1/gaussian-splats/build', {{
            map_id: selectedMapId || '',
            mission_id: mapEntry?.map_id || latest.mission_id || 'RecordMap',
            mission_execution_directory: mapEntry?.directory || '',
            gaussian_manifest_file: selectedMapId ? '' : (latest.gaussian_manifest_file || ''),
            force: true
          }});
        }}
        if (data.status) {{
          updateGaussianControls(data.status);
        }}
        setBanner(data.success ? 'ok' : 'error', data.message || '3D environment build request completed');
        gaussianBuildStatus.textContent = data.status ? formatGaussianStatus(data.status) : (data.message || '3D environment build request completed');
        await refreshGaussianStatus();
      }} catch (error) {{
        setBanner('error', error.message || '3D environment build request failed');
        gaussianBuildStatus.textContent = error.message || '3D environment build request failed';
      }} finally {{
        gaussianRequestInFlight = false;
        updateGaussianControls(latestGaussianStatus);
      }}
    }});

    document.getElementById('save-button').addEventListener('click', async () => {{
      ensureDefaultPatternSelection();
      const mapEntry = selectedMap();
      if (!selectedMissionIsSavedEditable()) {{
        setBanner('error', 'Path edits can only be saved to saved missions.');
        return;
      }}
      const data = await postJson('/api/v1/maps/save', {{
        map_id: selectedMapId,
        name: mapEntry.name || selectedMapId,
        source: 'metadata',
        source_map_id: selectedMapId,
        sweep_pattern: selectedPattern(),
        start_position: {{ percent: Number(startPositionInput.value) }},
        end_position: useEndPositionInput.checked ? {{ percent: Number(endPositionInput.value) }} : null,
        layer_visibility: layerVisibility(),
        overwrite_existing: true
      }});
      setBanner(data.success ? 'ok' : 'error', data.message || 'Save map request completed');
      if (data.success) {{
        setSelectedMapId(data.map?.map_id || selectedMapId);
      }}
      await loadRecordMapSnapshot();
    }});

    document.getElementById('refresh-button').addEventListener('click', async () => {{
      await loadRecordMapSnapshot();
    }});

    mapLayerButton.addEventListener('click', () => {{
      mapLayerControl.classList.toggle('open');
    }});

    document.addEventListener('click', (event) => {{
      if (!mapLayerControl.contains(event.target)) {{
        mapLayerControl.classList.remove('open');
      }}
      if (!mapCombo.contains(event.target)) {{
        closeMapCombo();
      }}
    }});

    for (const input of layerToggles) {{
      input.addEventListener('change', () => {{
        syncLayerInputs(input);
        updateMap(lastMapSnapshot);
      }});
    }}

    for (const input of [startPositionInput, endPositionInput, useEndPositionInput]) {{
      input.addEventListener('input', () => updateMap(lastMapSnapshot));
      input.addEventListener('change', () => updateMap(lastMapSnapshot));
    }}
    view2dButton.addEventListener('click', () => {{
      currentView = '2d';
      view2dButton.classList.remove('secondary');
      view3dButton.classList.add('secondary');
      recordMapElement.style.display = '';
      splatViewElement.style.display = 'none';
      mapLayerControl.style.display = '';
      updateLayerPanelMode();
      gaussianOverlaySummary.style.display = 'none';
      window.setTimeout(() => map.invalidateSize(), 0);
    }});
    view3dButton.addEventListener('click', () => {{
      currentView = '3d';
      view3dButton.classList.remove('secondary');
      view2dButton.classList.add('secondary');
      recordMapElement.style.display = 'none';
      splatViewElement.style.display = 'block';
      mapLayerControl.style.display = '';
      updateLayerPanelMode();
      gaussianOverlaySummary.style.display = '';
      renderSplatPreview(lastMapSnapshot);
      setBanner('ok', '3D view uses the saved 3D environment artifacts when available.');
    }});

    ensureDefaultPatternSelection();
    updateLayerPanelMode();
    refreshGaussianStatus().catch((error) => {{
      latestGaussianStatus = {{
        state: 'unknown',
        message: error.message || 'Failed to load Gaussian build status',
      }};
      updateGaussianControls(latestGaussianStatus);
    }});
    loadRecordMapSnapshot().catch((error) => {{
      mapComboList.innerHTML = '';
      appendMapComboOption('', latestRecordingLabel);
      mapNameInput.value = latestRecordingLabel;
      latestMapMessage.textContent = error.message || 'Failed to load saved maps.';
      setBanner('error', error.message || 'Failed to load record map page state');
    }});
    window.setInterval(() => {{
      loadRecordMapSnapshot().catch(() => null);
      refreshTopbarStatus();
    }}, 4000);
    window.setInterval(() => {{
      refreshGaussianStatus().catch((error) => {{
        latestGaussianStatus = {{
          ...latestGaussianStatus,
          state: latestGaussianStatus.state || 'unknown',
          message: error.message || 'Failed to refresh Gaussian build status',
        }};
        updateGaussianControls(latestGaussianStatus);
      }});
    }}, 2000);
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
    html, body {{
      width: 100vw;
      min-height: 100dvh;
      overflow-x: hidden;
    }}
    body {{
      margin: 0;
      font-family: "Avenir Next", "Segoe UI", sans-serif;
      color: var(--ink);
      background: linear-gradient(180deg, var(--bg) 0%, var(--bg-alt) 100%);
      touch-action: manipulation;
    }}
    main {{
      width: 100vw;
      min-height: 100dvh;
      overflow-x: hidden;
      padding: calc(72px + env(safe-area-inset-top)) 14px calc(88px + env(safe-area-inset-bottom));
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
    .app-topbar {{
      position: fixed;
      top: calc(10px + env(safe-area-inset-top));
      left: calc(10px + env(safe-area-inset-left));
      right: calc(10px + env(safe-area-inset-right));
      z-index: 700;
      display: grid;
      grid-template-columns: 46px minmax(0, 1fr) auto auto;
      gap: 10px;
      align-items: center;
      min-height: 52px;
      padding: 5px;
      border: 1px solid var(--line);
      border-radius: 18px;
      background: rgba(24, 27, 29, 0.84);
      box-shadow: 0 16px 36px rgba(0, 0, 0, 0.32);
      backdrop-filter: blur(8px);
    }}
    .app-topbar h1 {{
      overflow: hidden;
      margin: 0;
      color: var(--accent);
      text-overflow: ellipsis;
      white-space: nowrap;
      font-size: 1.08rem;
    }}
    .topbar-status {{
      display: inline-flex;
      align-items: center;
      min-height: 34px;
      border-radius: 999px;
      padding: 6px 10px;
      background: rgba(253, 202, 15, 0.12);
      color: var(--accent);
      font-weight: 700;
      font-size: 0.78rem;
      text-transform: uppercase;
    }}
    .icon-button {{
      width: 46px;
      padding: 0;
      color: var(--ink);
      background: rgba(18, 20, 21, 0.76);
      border: 1px solid var(--line);
      font-size: 1.24rem;
      letter-spacing: 0;
      text-transform: none;
    }}
    .drawer-backdrop {{
      position: fixed;
      inset: 0;
      z-index: 850;
      display: none;
      background: rgba(0, 0, 0, 0.42);
    }}
    .drawer-backdrop.show {{ display: block; }}
    .nav {{
      position: fixed;
      top: 0;
      bottom: 0;
      left: 0;
      z-index: 900;
      display: grid;
      align-content: start;
      gap: 9px;
      width: min(90vw, 390px);
      padding: calc(18px + env(safe-area-inset-top)) 18px calc(18px + env(safe-area-inset-bottom));
      overflow-y: auto;
      background: rgba(42, 46, 48, 0.96);
      border-right: 1px solid var(--line);
      box-shadow: 0 18px 42px rgba(0, 0, 0, 0.36);
      transform: translateX(-105%);
      transition: transform 180ms ease;
      backdrop-filter: blur(8px);
    }}
    .nav.open {{
      transform: translateX(0);
    }}
    .nav-link {{
      display: block;
      text-decoration: none;
      color: var(--ink);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      background: rgba(18, 20, 21, 0.58);
      font-size: 0.92rem;
      text-transform: none;
      letter-spacing: 0;
    }}
    .nav-list {{
      display: grid;
      gap: 9px;
      margin-top: 14px;
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
      height: 100%;
      margin-top: 0;
      border-radius: 18px;
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
      --bar-height: clamp(36px, 5vw, 48px);
      --cluster-gap: clamp(8px, 1.6vw, 12px);
      display: grid;
      grid-template-columns: var(--stick-size);
      grid-template-rows: var(--bar-height) var(--stick-size) var(--bar-height);
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
      width: var(--stick-size);
      min-height: var(--bar-height);
      display: flex;
      align-items: center;
      justify-content: center;
    }}
    .speed-scale {{
      width: var(--stick-size);
      height: var(--bar-height);
      min-height: 0;
      max-height: 48px;
      display: flex;
      flex-direction: row;
      align-items: center;
      justify-content: space-between;
      padding: 0 7px;
      touch-action: none;
      user-select: none;
      cursor: pointer;
    }}
    .scale-segment {{
      width: 8px;
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
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
      justify-items: stretch;
      align-content: center;
    }}
    .center-controls.active-mode .idle-start {{
      display: none;
    }}
    .merged-button {{
      display: none;
      grid-column: 1 / -1;
    }}
    .center-controls.active-mode .merged-button {{
      display: block;
    }}
    .left-column-button {{
      grid-column: 1;
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
    .busy-label {{
      display: inline-block;
      margin-right: 10px;
      vertical-align: middle;
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
      display: none;
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
      position: fixed;
      top: calc(76px + env(safe-area-inset-top));
      left: 50%;
      z-index: 980;
      width: min(92vw, 520px);
      transform: translateX(-50%);
      padding: 12px 14px;
      border-radius: 12px;
      display: none;
      font-weight: 700;
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
        --bar-height: clamp(32px, 9vw, 44px);
        --stick-size: min(78vw, 310px);
      }}
    }}
    .teleop-options-dock {{
      position: fixed;
      left: calc(12px + env(safe-area-inset-left));
      right: calc(12px + env(safe-area-inset-right));
      bottom: calc(12px + env(safe-area-inset-bottom));
      z-index: 720;
      display: grid;
      justify-items: center;
      pointer-events: none;
    }}
    .teleop-options-button {{
      pointer-events: auto;
      width: min(100%, 360px);
      border: 1px solid var(--line);
      color: var(--ink);
      background: rgba(18, 20, 21, 0.92);
    }}
    .teleop-options-list {{
      pointer-events: auto;
      display: none;
      width: min(100%, 360px);
      margin-bottom: 8px;
      padding: 10px;
      border: 1px solid var(--line);
      border-radius: 14px;
      background: rgba(42, 46, 48, 0.98);
      box-shadow: 0 18px 42px rgba(0, 0, 0, 0.36);
      backdrop-filter: blur(8px);
      gap: 8px;
    }}
    .teleop-options-dock.open .teleop-options-list {{
      display: grid;
    }}
    .teleop-option-row {{
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      gap: 12px;
      align-items: center;
      min-height: 48px;
      border: 1px solid var(--line);
      border-radius: 10px;
      padding: 8px 10px;
      background: rgba(18, 20, 21, 0.56);
    }}
    .teleop-option-row span {{
      color: var(--muted);
      font-size: 0.84rem;
    }}
    .teleop-option-row button {{
      min-width: 88px;
    }}
    {self._shared_topbar_css()}
  </style>
</head>
<body>
  <main>
    {self._render_topbar("Teleop", '<button id="teleop-topbar-stop" class="stop" type="button" disabled>Stop</button>')}
    <div id="banner" class="banner" role="status" aria-live="polite"></div>
    <div id="drawer-backdrop" class="drawer-backdrop"></div>
    <nav id="nav-drawer" class="nav" aria-label="Application navigation">
      <h2>O-ROBOTICS</h2>
      <div class="muted">Robot controls</div>
      <div class="nav-list">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Calendar</a>
        <a class="nav-link" href="/missions">Missions</a>
        <a class="nav-link" href="/teleop">Teleop</a>
        <a class="nav-link" href="/developer">Developer</a>
      </div>
    </nav>
    <section class="card" style="display: none;">
      <div class="status-row">
        <div>State: <span class="status-value">--</span></div>
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
          <button id="teleop-start-button" class="idle-start" type="button">Teleop</button>
          <button id="record-map-start-button" class="idle-start" type="button">Record Map</button>
          <button id="teleop-toggle-button" class="merged-button" type="button">Stop</button>
        </section>
        <section id="tools-panel" class="stick-panel tools-panel">
          <h2>Tools</h2>
          <div class="stick-cluster">
            <div id="tool-scale-slot" class="scale-slot"></div>
            <div id="right-stick" class="stick-shell" aria-label="Tool joystick">
              <div id="right-knob" class="stick-knob"></div>
            </div>
            <div class="scale-slot"></div>
          </div>
        </section>
      </div>
      <div id="tool-scale" class="speed-scale" role="slider" aria-label="Tool speed" aria-valuemin="0" aria-valuemax="100" aria-valuenow="50"></div>
    </section>
    <section id="teleop-options-dock" class="teleop-options-dock" aria-label="Teleop options">
      <div id="teleop-options-list" class="teleop-options-list">
        <div class="teleop-option-row">
          <div><strong>Lights</strong><br><span>Front work lights</span></div>
          <button id="lights-button" class="toggle-button" type="button">Off</button>
        </div>
        <div class="teleop-option-row">
          <div><strong>Camera</strong><br><span>Live driving view</span></div>
          <button id="camera-button" class="camera-button" type="button">Off</button>
        </div>
        <div class="teleop-option-row">
          <div><strong>Two Stick</strong><br><span>Separate drive and tool sticks</span></div>
          <button id="two-stick-button" class="mode-button" type="button">Off</button>
        </div>
      </div>
      <button id="teleop-options-button" class="teleop-options-button" type="button" aria-expanded="false" aria-controls="teleop-options-list">Options</button>
    </section>
  </main>

  <script>
    {self._shared_topbar_js()}
    const banner = document.getElementById('banner');
    const teleopStartButton = document.getElementById('teleop-start-button');
    const recordMapStartButton = document.getElementById('record-map-start-button');
    const teleopToggleButton = document.getElementById('teleop-toggle-button');
    const teleopTopbarStopButton = document.getElementById('teleop-topbar-stop');
    const navDrawer = document.getElementById('nav-drawer');
    const drawerBackdrop = document.getElementById('drawer-backdrop');
    const twoStickButton = document.getElementById('two-stick-button');
    const lightsButton = document.getElementById('lights-button');
    const cameraButton = document.getElementById('camera-button');
    const cameraFeed = document.getElementById('camera-feed');
    const teleopStage = document.getElementById('teleop-stage');
    const teleopLayout = document.getElementById('teleop-layout');
    const teleopOptionsDock = document.getElementById('teleop-options-dock');
    const teleopOptionsButton = document.getElementById('teleop-options-button');
    const centerControls = document.querySelector('.center-controls');
    const driveToolScaleSlot = document.getElementById('drive-tool-scale-slot');
    const toolScaleSlot = document.getElementById('tool-scale-slot');
    const speedScales = {{
      wheel: {{ value: 0.5, shell: document.getElementById('wheel-scale'), pointerId: null }},
      tool: {{ value: 0.5, shell: document.getElementById('tool-scale'), pointerId: null }},
    }};
    let teleopReady = false;
    let transitionBusy = false;
    let activeTeleopMode = '';
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
      const x = clamp(event.clientX - rect.left, 0, rect.width);
      scale.value = clamp(x / rect.width, 0, 1);
      renderScale(scale);
    }}
    function createScaleSegments(scale) {{
      const segmentCount = 20;
      for (let index = 0; index < segmentCount; index += 1) {{
        const segment = document.createElement('div');
        segment.className = 'scale-segment';
        segment.style.height = `${{14 + index * 0.9}}px`;
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
      twoStickButton.textContent = twoStickEnabled ? 'On' : 'Off';
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
      cameraButton.textContent = 'Off';
      teleopStage.classList.remove('camera-active', 'camera-waiting');
    }}
    function updateCameraState() {{
      cameraButton.classList.toggle('enabled', cameraEnabled);
      cameraButton.textContent = cameraEnabled ? 'On' : 'Off';
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
      centerControls.classList.add('active-mode');
      teleopToggleButton.disabled = true;
      teleopToggleButton.classList.remove('stop');
      teleopToggleButton.textContent = '';
      const labelSpan = document.createElement('span');
      labelSpan.className = 'busy-label';
      labelSpan.textContent = label;
      const gearSpan = document.createElement('span');
      gearSpan.className = 'gear';
      gearSpan.setAttribute('aria-hidden', 'true');
      gearSpan.innerHTML = '&#9881;';
      teleopToggleButton.append(labelSpan, gearSpan);
    }}
    function renderButton() {{
      if (transitionBusy) {{
        return;
      }}
      centerControls.classList.toggle('active-mode', teleopReady);
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
      activeTeleopMode = active?.mission_id === 'RecordMap' ? 'record_map' : (active?.mission_id === 'Teleop' ? 'teleop' : '');
      teleopReady = String(fsm.current_state || '').toUpperCase() === 'RUNNING' &&
        [220, 225].includes(Number(fsm.current_profile)) &&
        !transitionActive &&
        active &&
        ['Teleop', 'RecordMap'].includes(active.mission_id) &&
        active.active !== false;
      lightsEnabled = Boolean(data.teleop_lights_enabled);
      lightsButton.classList.toggle('enabled', lightsEnabled);
      lightsButton.textContent = lightsEnabled ? 'On' : 'Off';
      transitionBusy = transitionActive && (
        [220, 225].includes(Number(fsm.transitioning_to_profile)) ||
        [220, 225].includes(Number(fsm.current_profile)) ||
        ['Teleop', 'RecordMap'].includes(active.mission_id)
      );
      const teleopStateLabel = teleopReady
        ? (activeTeleopMode === 'record_map' ? 'RECORDING MAP' : 'TELEOP ACTIVE')
        : (display.current_state || fsm.current_state || 'READY');
      updateTopbarFromStatus(data, teleopStateLabel);
      document.getElementById('fsm-profile').textContent = formatProfileValue(
        display.current_profile !== undefined && display.current_profile !== null ? display.current_profile : fsm.current_profile
      );
      document.getElementById('active-mission').textContent = active?.mission_id || '--';
      if (transitionBusy) {{
        setBusyButton(display.transition_progress || 'Transitioning');
      }} else {{
        renderButton();
      }}
      teleopTopbarStopButton.disabled = !teleopReady || transitionBusy;
      teleopTopbarStopButton.style.display = teleopReady || transitionBusy ? '' : 'none';
      updateCameraState();
    }}

    async function startTeleopMode(mode) {{
      try {{
        setBusyButton(mode === 'record_map' ? 'Recording' : 'Starting');
        const result = await postJson('/api/v1/teleop/start', {{ mode }});
        setBanner(result.success ? 'ok' : 'error', result.message || 'Teleop start request completed');
      }} catch (error) {{
        setBanner('error', error.message || 'Teleop request failed');
      }} finally {{
        await loadStatus();
      }}
    }}

    teleopStartButton.addEventListener('click', () => {{
      startTeleopMode('teleop');
    }});

    recordMapStartButton.addEventListener('click', () => {{
      startTeleopMode('record_map');
    }});

    teleopToggleButton.addEventListener('click', async () => {{
      try {{
        if (teleopReady) {{
          setBusyButton('Stopping');
          await sendZeroCommand();
          const result = await postJson('/api/v1/teleop/stop', {{}});
          setBanner(result.success ? 'ok' : 'error', result.message || 'Teleop stop request completed');
        }} else {{
          await startTeleopMode(activeTeleopMode || 'teleop');
          return;
        }}
      }} catch (error) {{
        setBanner('error', error.message || 'Teleop request failed');
      }} finally {{
        await loadStatus();
      }}
    }});
    teleopTopbarStopButton.addEventListener('click', () => {{
      teleopToggleButton.click();
    }});
    document.getElementById('open-nav-button').addEventListener('click', () => {{
      closeBatteryPopover();
      teleopOptionsDock.classList.remove('open');
      teleopOptionsButton.setAttribute('aria-expanded', 'false');
      navDrawer.classList.add('open');
      drawerBackdrop.classList.add('show');
    }});
    drawerBackdrop.addEventListener('click', () => {{
      navDrawer.classList.remove('open');
      drawerBackdrop.classList.remove('show');
    }});
    lightsButton.addEventListener('click', async () => {{
      const nextEnabled = !lightsEnabled;
      lightsButton.disabled = true;
      try {{
        const result = await postJson('/api/v1/teleop/lights', {{ enabled: nextEnabled }});
        if (result.success) {{
          lightsEnabled = nextEnabled;
          lightsButton.classList.toggle('enabled', lightsEnabled);
          lightsButton.textContent = lightsEnabled ? 'On' : 'Off';
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
    teleopOptionsButton.addEventListener('click', (event) => {{
      event.stopPropagation();
      const open = !teleopOptionsDock.classList.contains('open');
      teleopOptionsDock.classList.toggle('open', open);
      teleopOptionsButton.setAttribute('aria-expanded', String(open));
      closeBatteryPopover();
    }});
    document.addEventListener('click', (event) => {{
      if (!teleopOptionsDock.contains(event.target)) {{
        teleopOptionsDock.classList.remove('open');
        teleopOptionsButton.setAttribute('aria-expanded', 'false');
      }}
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

    def render_missions_html(self) -> str:
        return self.render_map_html()

    def render_legacy_missions_html(self) -> str:
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
      padding: calc(82px + env(safe-area-inset-top)) 24px 24px;
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
        <a class="nav-link" href="/missions">Missions</a>
        <a class="nav-link" href="/teleop">Teleop</a>
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
      ['use_gaussian', 'Gaussian Capture'],
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
      use_gaussian: false,
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
      if (['Teleop'].includes(selectedMissionId)) {{
        selectedMissionId = '';
      }}
      if (!selectedMissionId) {{
        const firstPreviewable = (data.missions || []).find(
          (mission) => !['Teleop'].includes(mission.mission_id)
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
      const hiddenMissionIds = new Set(['Teleop']);
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
          const missionTitle = mission.mission_id === 'RecordMap' ? 'Latest Recording' : mission.mission_id;
          const item = document.createElement('div');
          item.className = 'mission';
          item.innerHTML = `
            <div>
              <strong>${{missionTitle}}</strong><br>
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
        ['Teleop'].includes(selectedMissionId) ||
        !missionsCache.some((mission) => mission.mission_id === selectedMissionId)
      ) {{
        selectedMissionId = '';
      }}
      if (!selectedMissionId) {{
        const firstPreviewable = missionsCache.find(
          (mission) => !['Teleop'].includes(mission.mission_id)
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
      --accent: #fdca0f;
      --accent-strong: #ffe06b;
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
      padding: calc(82px + env(safe-area-inset-top)) 24px 24px;
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
    .app-topbar {{
      position: fixed;
      top: calc(10px + env(safe-area-inset-top));
      left: calc(10px + env(safe-area-inset-left));
      right: calc(10px + env(safe-area-inset-right));
      z-index: 700;
      display: grid;
      grid-template-columns: 46px minmax(0, 1fr) auto;
      gap: 10px;
      align-items: center;
      min-height: 52px;
      padding: 5px;
      border: 1px solid var(--line);
      border-radius: 18px;
      background: rgba(24, 27, 29, 0.84);
      box-shadow: 0 16px 36px rgba(0, 0, 0, 0.32);
      backdrop-filter: blur(8px);
    }}
    .app-topbar h1 {{
      overflow: hidden;
      margin: 0;
      color: var(--accent);
      text-overflow: ellipsis;
      white-space: nowrap;
      font-size: 1.08rem;
    }}
    .topbar-status {{
      display: inline-flex;
      align-items: center;
      min-height: 34px;
      border-radius: 999px;
      padding: 6px 10px;
      background: rgba(253, 202, 15, 0.12);
      color: var(--accent);
      font-weight: 700;
      font-size: 0.78rem;
      text-transform: uppercase;
    }}
    .icon-button {{
      width: 46px;
      padding: 0;
      color: var(--ink);
      background: rgba(18, 20, 21, 0.76);
      border: 1px solid var(--line);
      font-size: 1.24rem;
      letter-spacing: 0;
      text-transform: none;
    }}
    .drawer-backdrop {{
      position: fixed;
      inset: 0;
      z-index: 850;
      display: none;
      background: rgba(0, 0, 0, 0.42);
    }}
    .drawer-backdrop.show {{ display: block; }}
    .nav {{
      position: fixed;
      top: 0;
      bottom: 0;
      left: 0;
      z-index: 900;
      display: grid;
      align-content: start;
      gap: 9px;
      width: min(90vw, 390px);
      padding: calc(18px + env(safe-area-inset-top)) 18px calc(18px + env(safe-area-inset-bottom));
      overflow-y: auto;
      background: rgba(42, 46, 48, 0.96);
      border-right: 1px solid var(--line);
      box-shadow: 0 18px 42px rgba(0, 0, 0, 0.36);
      transform: translateX(-105%);
      transition: transform 180ms ease;
      backdrop-filter: blur(8px);
    }}
    .nav.open {{
      transform: translateX(0);
    }}
    .nav-link {{
      display: block;
      text-decoration: none;
      color: var(--ink);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      background: rgba(18, 20, 21, 0.58);
      font-size: 0.92rem;
      text-transform: none;
      letter-spacing: 0;
    }}
    .log-list {{
      display: grid;
      gap: 6px;
      margin-top: 14px;
    }}
    .toggle-list {{
      display: grid;
      gap: 8px;
      margin-top: 14px;
    }}
    .toggle-item {{
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 10px 12px;
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
      border-radius: 8px;
      padding: 8px 10px;
      background: var(--panel);
      display: grid;
      grid-template-columns: 96px minmax(120px, 220px) minmax(0, 1fr);
      gap: 12px;
      align-items: start;
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
      max-height: 58vh;
      overflow: auto;
      white-space: pre-wrap;
      word-break: break-word;
      font-family: "SFMono-Regular", Consolas, "Liberation Mono", monospace;
      color: var(--ink);
    }}
    .muted {{ color: var(--muted); }}
    button {{
      border: 0;
      border-radius: 999px;
      min-height: 44px;
      padding: 10px 14px;
      cursor: pointer;
      color: #08100a;
      background: var(--accent);
      font: inherit;
      font-weight: 700;
    }}
    .developer-tabs {{
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      margin-top: 18px;
    }}
    .developer-tab {{
      color: var(--ink);
      background: rgba(18, 20, 21, 0.58);
      border: 1px solid var(--line);
    }}
    .developer-tab.active {{
      color: #08100a;
      background: var(--accent);
    }}
    .developer-panel {{ display: none; margin-top: 18px; }}
    .developer-panel.active {{ display: block; }}
    .toggle-group {{
      margin-top: 16px;
      color: var(--accent);
      font-size: 0.86rem;
      text-transform: uppercase;
      letter-spacing: 0.06em;
    }}
    @media (max-width: 720px) {{
      main {{ padding: calc(82px + env(safe-area-inset-top)) 14px 18px; }}
      .log-entry {{ grid-template-columns: 1fr; gap: 4px; }}
    }}
    {self._shared_topbar_css()}
  </style>
</head>
<body>
  {self._render_topbar("Developer")}
  <div id="drawer-backdrop" class="drawer-backdrop"></div>
  <nav id="nav-drawer" class="nav" aria-label="Application navigation">
    <h2>O-ROBOTICS</h2>
    <div class="muted">Engineering views</div>
    <a class="nav-link" href="/">Dashboard</a>
    <a class="nav-link" href="/calendar">Calendar</a>
    <a class="nav-link" href="/missions">Missions</a>
    <a class="nav-link" href="/teleop">Teleop</a>
    <a class="nav-link" href="/developer">Developer</a>
  </nav>
  <main>
    <section class="card">
      <h1>Developer</h1>
      <div class="muted">Inspect recent ROS warning/error logs and the raw web status payload.</div>
    </section>

    <div class="developer-tabs" role="tablist" aria-label="Developer views">
      <button class="developer-tab active" type="button" data-tab="logs">Logs</button>
      <button class="developer-tab" type="button" data-tab="profile">Mission profile</button>
      <button class="developer-tab" type="button" data-tab="raw">Raw status</button>
    </div>

    <section id="developer-panel-logs" class="card developer-panel active">
      <h2>System log</h2>
      <div class="muted">Shows recent `WARN`, `ERROR`, and `FATAL` messages from ROS.</div>
      <div id="log-list" class="log-list"></div>
    </section>

    <section id="developer-panel-profile" class="card developer-panel">
      <h2>Mission profile</h2>
      <div id="developer-selected-mission" class="muted">Selected mission: none</div>
      <div id="layer-toggle-list" class="toggle-list"></div>
    </section>

    <section id="developer-panel-raw" class="card developer-panel">
      <div style="display: flex; align-items: center; justify-content: space-between; gap: 12px;">
        <h2>Raw status</h2>
        <button id="copy-raw-status" type="button">Copy JSON</button>
      </div>
      <pre id="raw-status">{{}}</pre>
    </section>
  </main>

  <script>
    {self._shared_topbar_js()}
    const logList = document.getElementById('log-list');
    const rawStatus = document.getElementById('raw-status');
    const selectedMissionElement = document.getElementById('developer-selected-mission');
    const layerToggleList = document.getElementById('layer-toggle-list');
    const developerTabs = [...document.querySelectorAll('.developer-tab')];
    const developerPanels = [...document.querySelectorAll('.developer-panel')];
    const navDrawer = document.getElementById('nav-drawer');
    const drawerBackdrop = document.getElementById('drawer-backdrop');
    const layerGroups = [
      ['Hardware', ['use_amr_sweeper_ros2_control', 'use_amr_sweeper_battery', 'use_amr_sweeper_system_info', 'use_amr_sweeper_usb_cameras', 'use_amr_sweeper_depth_camera', 'use_amr_sweeper_imu', 'use_amr_sweeper_gnss', 'use_ntrip_client']],
      ['Controllers', ['use_amr_sweeper_drive_controller', 'use_amr_sweeper_tool_controller', 'use_amr_sweeper_teleop', 'use_amr_sweeper_sweeping_controller', 'use_amr_sweeper_attitude_controller', 'use_amr_sweeper_collision_detector', 'use_amr_sweeper_safety_controller', 'use_joy_node']],
      ['Navigation', ['use_amr_sweeper_visual_odometry', 'use_amr_sweeper_localization', 'use_amr_sweeper_mapping', 'use_amr_sweeper_navigation']],
      ['Mission', ['use_gaussian', 'auto_start_mission']],
    ];
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
      ['use_gaussian', 'Gaussian Capture'],
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
      use_gaussian: false,
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
      const labelsByKey = new Map(layerToggleDefinitions);
      for (const [groupName, keys] of layerGroups) {{
        const groupHeading = document.createElement('div');
        groupHeading.className = 'toggle-group';
        groupHeading.textContent = groupName;
        layerToggleList.appendChild(groupHeading);
        for (const key of keys) {{
        const label = labelsByKey.get(key) || key;
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
      updateTopbarFromStatus(data, 'Engineering');
      const recentLogs = data.recent_logs || [];

      logList.innerHTML = '';
      if (recentLogs.length === 0) {{
        logList.innerHTML = '<div class="muted">No warning, error, or fatal messages captured yet.</div>';
      }} else {{
        for (const entry of recentLogs) {{
          const item = document.createElement('div');
          item.className = `log-entry ${{String(entry.level || '').toLowerCase()}}`;
          item.innerHTML = `
            <div class="log-meta">${{entry.time || entry.stamp || '--'}}<br>${{entry.level || 'WARN'}}</div>
            <div><strong>${{entry.name || '-'}}</strong><br><span class="muted">line ${{entry.line ?? '-'}}</span></div>
            <div>${{entry.msg || ''}}</div>
          `;
          logList.appendChild(item);
        }}
      }}

      rawStatus.textContent = JSON.stringify(data, null, 2);
    }}

    for (const tab of developerTabs) {{
      tab.addEventListener('click', () => {{
        const target = tab.dataset.tab || 'logs';
        for (const other of developerTabs) {{
          other.classList.toggle('active', other === tab);
        }}
        for (const panel of developerPanels) {{
          panel.classList.toggle('active', panel.id === `developer-panel-${{target}}`);
        }}
      }});
    }}
    document.getElementById('open-nav-button').addEventListener('click', () => {{
      closeBatteryPopover();
      navDrawer.classList.add('open');
      drawerBackdrop.classList.add('show');
    }});
    drawerBackdrop.addEventListener('click', () => {{
      navDrawer.classList.remove('open');
      drawerBackdrop.classList.remove('show');
    }});
    document.getElementById('copy-raw-status').addEventListener('click', async () => {{
      await navigator.clipboard.writeText(rawStatus.textContent || '{{}}');
    }});

    Promise.all([loadStatus(), loadMissions()]).catch(() => markTopbarDisconnected());
    setInterval(() => loadStatus().catch(() => markTopbarDisconnected()), 2000);
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
                if parsed.path == "/missions":
                    self._send_html(node.render_map_html())
                    return
                if parsed.path == "/developer":
                    self._send_html(node.render_developer_html())
                    return
                if parsed.path == "/record-map":
                    self._send_redirect("/missions")
                    return
                if parsed.path == "/teleop":
                    self._send_html(node.render_teleop_html())
                    return
                if parsed.path == "/api/v1/teleop/camera/stream":
                    self._send_teleop_camera_stream()
                    return
                if parsed.path.startswith("/api/v1/gaussian-splats/artifacts/"):
                    self._send_gaussian_splat_artifact(parsed)
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

            def log_message(self, message_format: str, *args: Any) -> None:
                node.get_logger().debug(f"HTTP {self.address_string()} - {message_format % args}")

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
                if self.command == "GET" and parsed.path == "/api/v1/maps":
                    return {"action": "LIST_MAPS", "payload": {}}
                if self.command == "GET" and parsed.path == "/api/v1/record-map":
                    return {"action": "GET_RECORD_MAP", "payload": {}}
                if self.command == "GET" and parsed.path == "/api/v1/gaussian-splats/status":
                    return {"action": "GET_GAUSSIAN_SPLAT_STATUS", "payload": {}}

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

                if self.command == "POST" and parsed.path.startswith("/api/v1/missions/"):
                    parts = [urllib.parse.unquote(part) for part in parsed.path.strip("/").split("/")]
                    if len(parts) >= 5 and parts[:3] == ["api", "v1", "missions"]:
                        mission_id = parts[3]
                        request_payload = dict(payload)
                        request_payload["mission_id"] = mission_id
                        if len(parts) == 5 and parts[4] == "zones":
                            return {"action": "CREATE_MISSION_ZONE", "payload": request_payload}
                        if len(parts) == 5 and parts[4] == "station":
                            return {"action": "SET_MISSION_STATION", "payload": request_payload}
                        if len(parts) == 6 and parts[4] == "zones":
                            request_payload["zone_id"] = parts[5]
                            return {"action": "UPDATE_MISSION_ZONE", "payload": request_payload}
                        if len(parts) == 7 and parts[4] == "zones" and parts[6] == "delete":
                            request_payload["zone_id"] = parts[5]
                            return {"action": "DELETE_MISSION_ZONE", "payload": request_payload}

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
                    "/api/v1/maps/save": "SAVE_MAP",
                    "/api/v1/maps/delete": "DELETE_MAP",
                    "/api/v1/missions/rename": "RENAME_MISSION",
                    "/api/v1/gaussian-splats/build": "BUILD_GAUSSIAN_SPLAT",
                    "/api/v1/gaussian-splats/pause": "PAUSE_GAUSSIAN_SPLAT",
                    "/api/v1/gaussian-splats/resume": "RESUME_GAUSSIAN_SPLAT",
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
                    connection.settimeout(self._backend_timeout_sec(request))
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

            @staticmethod
            def _backend_timeout_sec(request: dict[str, Any]) -> float:
                action = str(request.get("action", "")).strip().upper()
                if action == "SAVE_MAP":
                    return 180.0
                if action in {"BUILD_GAUSSIAN_SPLAT", "PAUSE_GAUSSIAN_SPLAT", "RESUME_GAUSSIAN_SPLAT"}:
                    return 65.0
                return 20.0

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

            def _send_gaussian_splat_artifact(self, parsed: urllib.parse.ParseResult) -> None:
                relative = parsed.path[len("/api/v1/gaussian-splats/artifacts/"):]
                artifact_path = Path(urllib.parse.unquote(relative)).expanduser()
                if not artifact_path.is_absolute():
                    artifact_path = Path.cwd() / artifact_path
                try:
                    resolved = artifact_path.resolve()
                    allowed_roots = gaussian_splat_artifact_allowed_roots()
                    if not any(resolved == root or root in resolved.parents for root in allowed_roots):
                        raise ValueError("Artifact path is outside allowed roots")
                    if not resolved.exists() or not resolved.is_file():
                        raise FileNotFoundError("Gaussian artifact not found")
                    content_type = "application/json; charset=utf-8" if resolved.suffix == ".json" else "application/octet-stream"
                    if resolved.suffix == ".ply":
                        content_type = "application/octet-stream"
                    body = resolved.read_bytes()
                    self._send_bytes(
                        HTTPStatus.OK,
                        body,
                        {
                            "Content-Type": content_type,
                            "Cache-Control": "no-store",
                            "Content-Length": str(len(body)),
                        },
                    )
                except FileNotFoundError as exc:
                    self._send_json(HTTPStatus.NOT_FOUND, {"success": False, "message": str(exc)})
                except ValueError as exc:
                    self._send_json(HTTPStatus.BAD_REQUEST, {"success": False, "message": str(exc)})

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

            def _send_redirect(self, location: str) -> None:
                self._send_bytes(
                    HTTPStatus.FOUND,
                    b"",
                    {
                        "Location": location,
                        "Cache-Control": "no-store",
                        "Content-Length": "0",
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
