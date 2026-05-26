#!/usr/bin/env python3

from __future__ import annotations

import json
import errno
import threading
import urllib.parse
import calendar
from datetime import datetime, timedelta
from collections import deque
from html import escape
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import rclpy
from amr_sweeper_fsm.msg import FSMState, FSMStatus
from amr_sweeper_fsm.srv import RequestState
from amr_sweeper_mission_executor.srv import (
    EndMission,
    ExecuteMission,
    ListExecutableMissions,
    UploadVda5050Mission,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import Log
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
        self._missions_from_db_directory = self.declare_parameter(
            "missions_from_db_directory",
            "src/missions_from_db",
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
        self._fsm_request_service = self.declare_parameter(
            "fsm_request_service",
            "request_state",
        ).value
        self._fsm_state_topic = self.declare_parameter("fsm_state_topic", "fsm_state").value
        self._fsm_status_topic = self.declare_parameter("fsm_status_topic", "fsm_status").value
        self._gnss_topic = self.declare_parameter("gnss_topic", "gnss/navsat").value
        self._battery_topic = self.declare_parameter("battery_topic", "battery_state").value
        self._rosout_topic = self.declare_parameter("rosout_topic", "/rosout").value
        self._max_log_entries = int(self.declare_parameter("max_log_entries", 100).value)

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
        self._fsm_request_client = self.create_client(
            RequestState,
            self._fsm_request_service,
        )

        self._state_lock = threading.Lock()
        self._latest_fsm_state: dict[str, Any] | None = None
        self._latest_fsm_status: dict[str, Any] | None = None
        self._latest_navsat: dict[str, Any] | None = None
        self._latest_battery: dict[str, Any] | None = None
        self._recent_logs: deque[dict[str, Any]] = deque(maxlen=max(1, self._max_log_entries))

        self.create_subscription(FSMState, self._fsm_state_topic, self._handle_fsm_state, 10)
        self.create_subscription(FSMStatus, self._fsm_status_topic, self._handle_fsm_status, 10)
        self.create_subscription(NavSatFix, self._gnss_topic, self._handle_navsat, 10)
        self.create_subscription(BatteryState, self._battery_topic, self._handle_battery, 10)
        self.create_subscription(Log, self._rosout_topic, self._handle_rosout, 100)

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
                if parsed.path == "/api/status":
                    self._send_json(HTTPStatus.OK, node.status_snapshot())
                    return
                if parsed.path == "/api/missions":
                    try:
                        self._send_json(HTTPStatus.OK, node.list_executable_missions())
                    except Exception as exc:  # noqa: BLE001
                        self._send_json(HTTPStatus.BAD_GATEWAY, {"success": False, "message": str(exc)})
                    return
                if parsed.path == "/api/schedule":
                    try:
                        query = urllib.parse.parse_qs(parsed.query)
                        month = query.get("month", [""])[0]
                        self._send_json(HTTPStatus.OK, node.schedule_snapshot(month))
                    except Exception as exc:  # noqa: BLE001
                        self._send_json(HTTPStatus.BAD_GATEWAY, {"success": False, "message": str(exc)})
                    return
                if parsed.path == "/api/map-data":
                    try:
                        self._send_json(HTTPStatus.OK, node.map_snapshot())
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

    def _handle_rosout(self, message: Log) -> None:
        if int(message.level) < int(Log.WARN):
            return
        with self._state_lock:
            self._recent_logs.appendleft(
                {
                    "stamp": self._time_to_dict(message.stamp),
                    "level": self._log_level_name(int(message.level)),
                    "name": message.name,
                    "msg": message.msg,
                    "file": message.file,
                    "function": message.function,
                    "line": int(message.line),
                }
            )

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

    def request_reinitialize(self, payload: dict[str, Any]) -> dict[str, Any]:
        request = RequestState.Request()
        request.target_state = "INITIALIZING"
        request.target_lifecycle = "Active"
        request.target_profile_id = 1
        request.requester = str(payload.get("requester", "web_server_node"))
        request.priority = int(payload.get("priority", 220))
        request.force = bool(payload.get("force", False))
        request.reason = str(payload.get("reason", "operator requested reinitialize profile 001 from HTTP UI"))
        request.mission_execution_directory = ""

        response = self._call_service(
            self._fsm_request_client,
            request,
            timeout_sec=15.0,
            service_name=self._fsm_request_service,
        )
        return {
            "success": bool(response.accepted),
            "message": response.message,
            "current_state": response.current_state,
            "current_profile_id": int(response.current_profile_id),
            "desired_state": response.desired_state,
            "desired_profile_id": int(response.desired_profile_id),
        }

    def status_snapshot(self) -> dict[str, Any]:
        with self._state_lock:
            fsm_state = dict(self._latest_fsm_state) if self._latest_fsm_state is not None else None
            fsm_status = dict(self._latest_fsm_status) if self._latest_fsm_status is not None else None
            navsat = dict(self._latest_navsat) if self._latest_navsat is not None else None
            battery = dict(self._latest_battery) if self._latest_battery is not None else None
            recent_logs = list(self._recent_logs)

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
            "recent_logs": recent_logs,
        }

    def _load_active_execution(self) -> dict[str, Any] | None:
        pointer_path = _resolve_path(self._missions_log_directory) / self._active_execution_pointer_filename
        if not pointer_path.exists():
            return None
        try:
            return json.loads(pointer_path.read_text(encoding="utf-8"))
        except Exception as exc:  # noqa: BLE001
            return {"error": f"Failed to read active execution pointer: {exc}", "path": str(pointer_path)}

    def _discover_schedule_path(self) -> Path | None:
        active_execution = self._load_active_execution() or {}
        schedule_log_path = active_execution.get("schedule_log_path", "")
        if schedule_log_path:
            path = Path(schedule_log_path)
            if path.exists():
                return path

        missions_from_db_directory = _resolve_path(self._missions_from_db_directory)
        candidates = sorted(
            missions_from_db_directory.glob("schedule_*.ics"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        return candidates[0] if candidates else None

    @staticmethod
    def _unfold_ics_lines(text: str) -> list[str]:
        lines: list[str] = []
        for raw_line in text.splitlines():
            line = raw_line.rstrip("\r")
            if line.startswith((" ", "\t")) and lines:
                lines[-1] += line[1:]
            else:
                lines.append(line)
        return lines

    @staticmethod
    def _parse_rrule(rule: str) -> dict[str, str]:
        parsed: dict[str, str] = {}
        for token in rule.split(";"):
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            parsed[key] = value
        return parsed

    @staticmethod
    def _parse_ics_datetime(value: str) -> datetime:
        return datetime.strptime(value.strip(), "%Y%m%dT%H%M%S")

    @staticmethod
    def _parse_ics_duration(value: str) -> timedelta:
        if not value.startswith("P"):
            return timedelta(0)
        hours = 0
        minutes = 0
        seconds = 0
        number = ""
        in_time = False
        for character in value[1:]:
            if character == "T":
                in_time = True
                continue
            if character.isdigit():
                number += character
                continue
            if not in_time or not number:
                continue
            parsed = int(number)
            if character == "H":
                hours = parsed
            elif character == "M":
                minutes = parsed
            elif character == "S":
                seconds = parsed
            number = ""
        return timedelta(hours=hours, minutes=minutes, seconds=seconds)

    def _load_schedule_events(self) -> tuple[Path | None, list[dict[str, Any]]]:
        schedule_path = self._discover_schedule_path()
        if schedule_path is None or not schedule_path.exists():
            return None, []

        lines = self._unfold_ics_lines(schedule_path.read_text(encoding="utf-8"))
        events: list[dict[str, Any]] = []
        current: dict[str, Any] | None = None
        for line in lines:
            if line == "BEGIN:VEVENT":
                current = {}
                continue
            if line == "END:VEVENT":
                if current is not None:
                    events.append(current)
                current = None
                continue
            if current is None or ":" not in line:
                continue

            raw_key, value = line.split(":", 1)
            key = raw_key.split(";", 1)[0]
            current[key] = value
            if raw_key.startswith("DTSTART"):
                current["DTSTART_RAW"] = raw_key
            if raw_key.startswith("DTEND"):
                current["DTEND_RAW"] = raw_key
        return schedule_path, events

    @staticmethod
    def _nth_weekday_of_month(year: int, month: int, weekday: int, setpos: int) -> datetime | None:
        matching_days = []
        month_days = calendar.monthrange(year, month)[1]
        for day in range(1, month_days + 1):
            candidate = datetime(year, month, day)
            if candidate.weekday() == weekday:
                matching_days.append(candidate)
        if not matching_days:
            return None
        index = setpos - 1 if setpos > 0 else setpos
        if abs(index) >= len(matching_days) and setpos < 0:
            return matching_days[0]
        try:
            return matching_days[index]
        except IndexError:
            return None

    def _expand_event_occurrences(self, event: dict[str, Any], year: int, month: int) -> list[dict[str, Any]]:
        if "DTSTART" not in event:
            return []

        start = self._parse_ics_datetime(event["DTSTART"])
        end = self._parse_ics_datetime(event["DTEND"]) if "DTEND" in event else start + self._parse_ics_duration(event.get("DURATION", "PT0S"))
        duration = end - start
        rrule = self._parse_rrule(event.get("RRULE", ""))
        occurrences: list[dict[str, Any]] = []

        def append_occurrence(occurrence_start: datetime) -> None:
            occurrence_end = occurrence_start + duration
            occurrences.append(
                {
                    "uid": event.get("UID", ""),
                    "summary": event.get("SUMMARY", ""),
                    "description": event.get("DESCRIPTION", ""),
                    "schedule_type": event.get("X-SCHEDULE-TYPE", ""),
                    "mission_id": event.get("X-MISSION-ID", ""),
                    "robot_id": event.get("X-ROBOT-ID", ""),
                    "start": occurrence_start.isoformat(),
                    "end": occurrence_end.isoformat(),
                    "date": occurrence_start.strftime("%Y-%m-%d"),
                    "time": occurrence_start.strftime("%H:%M"),
                }
            )

        if not rrule:
            if start.year == year and start.month == month:
                append_occurrence(start)
            return occurrences

        freq = rrule.get("FREQ", "")
        if freq == "DAILY":
            current = start
            while current.year < year or (current.year == year and current.month < month):
                current += timedelta(days=1)
            while current.year == year and current.month == month:
                append_occurrence(current)
                current += timedelta(days=1)
            return occurrences

        if freq == "MONTHLY":
            byday = rrule.get("BYDAY", "")
            bysetpos = int(rrule.get("BYSETPOS", "1"))
            weekday_lookup = {
                "MO": 0,
                "TU": 1,
                "WE": 2,
                "TH": 3,
                "FR": 4,
                "SA": 5,
                "SU": 6,
            }
            weekday = weekday_lookup.get(byday)
            if weekday is not None:
                occurrence_day = self._nth_weekday_of_month(year, month, weekday, bysetpos)
                if occurrence_day is not None:
                    append_occurrence(
                        occurrence_day.replace(
                            hour=start.hour,
                            minute=start.minute,
                            second=start.second,
                        )
                    )
            return occurrences

        if start.year == year and start.month == month:
            append_occurrence(start)
        return occurrences

    def schedule_snapshot(self, month: str) -> dict[str, Any]:
        if month:
            selected_year, selected_month = month.split("-", 1)
            year = int(selected_year)
            month_number = int(selected_month)
        else:
            now = datetime.now()
            year = now.year
            month_number = now.month

        schedule_path, events = self._load_schedule_events()
        occurrences: list[dict[str, Any]] = []
        for event in events:
            occurrences.extend(self._expand_event_occurrences(event, year, month_number))
        occurrences.sort(key=lambda item: item["start"])

        return {
            "success": True,
            "schedule_path": str(schedule_path) if schedule_path is not None else "",
            "month": f"{year:04d}-{month_number:02d}",
            "month_name": f"{calendar.month_name[month_number]} {year}",
            "events": occurrences,
        }

    @staticmethod
    def _load_geojson_feature_collection(path: Path) -> dict[str, Any] | None:
        if not path.exists():
            return None
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return None

    @staticmethod
    def _route_geojson_from_vda5050(document: dict[str, Any], mission_id: str) -> dict[str, Any] | None:
        nodes = {node["nodeId"]: node["nodePosition"] for node in document.get("nodes", []) if "nodeId" in node and "nodePosition" in node}
        edges = {edge["edgeId"]: edge for edge in document.get("edges", []) if "edgeId" in edge}
        coverage_edge_ids = document.get("missionGeometries", {}).get("coveragePathEdgeIds", [])
        if not coverage_edge_ids:
            return None

        coordinates: list[list[float]] = []
        tail_node_id = None
        for edge_id in coverage_edge_ids:
            edge = edges.get(edge_id)
            if edge is None:
                continue
            start_id = edge.get("startNodeId")
            end_id = edge.get("endNodeId")
            ordered_ids = [start_id, end_id]
            if tail_node_id == end_id:
                ordered_ids = [end_id, start_id]
            elif tail_node_id == start_id:
                ordered_ids = [start_id, end_id]
            for node_id in ordered_ids:
                node_position = nodes.get(node_id)
                if node_position is None:
                    continue
                point = [float(node_position.get("x", 0.0)), float(node_position.get("y", 0.0))]
                if not coordinates or coordinates[-1] != point:
                    coordinates.append(point)
            tail_node_id = ordered_ids[-1]

        if len(coordinates) < 2:
            return None

        return {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {"name": mission_id, "source": "vda5050_json"},
                    "geometry": {"type": "LineString", "coordinates": coordinates},
                }
            ],
        }

    def map_snapshot(self) -> dict[str, Any]:
        missions_directory = _resolve_path(self._missions_from_db_directory)
        missions: list[dict[str, Any]] = []
        for mission_file in sorted(missions_directory.glob("*.json")):
            mission_id = mission_file.stem
            document = self._load_geojson_feature_collection(mission_file)
            route_geojson = None
            route_path = missions_directory / mission_id / f"{mission_id}_path.geojson"
            if route_path.exists():
                route_geojson = self._load_geojson_feature_collection(route_path)
            elif document is not None:
                route_geojson = self._route_geojson_from_vda5050(document, mission_id)
            missions.append(
                {
                    "mission_id": mission_id,
                    "route_geojson": route_geojson,
                    "route_available": route_geojson is not None,
                }
            )

        active_execution = self._load_active_execution() or {}
        active_route = None
        active_route_path = active_execution.get("mission_route_file", "")
        if active_route_path:
            active_route = self._load_geojson_feature_collection(Path(active_route_path))

        return {
            "success": True,
            "missions": missions,
            "active_execution": active_execution,
            "active_route_geojson": active_route,
        }

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

    @staticmethod
    def _log_level_name(level: int) -> str:
        if level >= int(Log.FATAL):
            return "FATAL"
        if level >= int(Log.ERROR):
            return "ERROR"
        if level >= int(Log.WARN):
            return "WARN"
        if level >= int(Log.INFO):
            return "INFO"
        return "DEBUG"

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
      background: rgba(255, 253, 248, 0.95);
      font-size: 0.92rem;
    }}
    .actions {{
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      margin-top: 12px;
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
      background: #fffdf8;
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
  </style>
</head>
<body>
  <main>
    <section class="hero">
      <h1>{title}</h1>
      <p class="muted">Open <strong>{public_base_url}</strong> on the robot Wi-Fi for manual mission launch, saved autonomous mission execution, VDA5050 upload, and live runtime status.</p>
      <div id="banner" class="banner"></div>
      <div class="nav">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Schedule Calendar</a>
        <a class="nav-link" href="/map">Mission Map</a>
      </div>
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
        <div class="actions">
          <button class="stop" id="stop-button" disabled>Stop Mission</button>
          <button id="reboot-button">Reboot</button>
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
    const banner = document.getElementById('banner');
    const missionList = document.getElementById('mission-list');
    const logList = document.getElementById('log-list');
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
      const recentLogs = data.recent_logs || [];
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

    def render_calendar_html(self) -> str:
        title = escape(self._site_title)
        return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{title} - Schedule Calendar</title>
  <style>
    :root {{
      --bg: #f4f1ea;
      --card: #fffaf2;
      --ink: #10212b;
      --muted: #5d6c74;
      --accent: #0f766e;
      --line: #d9d3c7;
      --work: #0f766e;
      --nowork: #b45309;
      --safety: #b91c1c;
    }}
    body {{
      margin: 0;
      font-family: "Segoe UI", "Helvetica Neue", sans-serif;
      color: var(--ink);
      background: linear-gradient(160deg, #f5f1e7 0%, #ece8df 100%);
    }}
    main {{
      max-width: 1200px;
      margin: 0 auto;
      padding: 24px;
    }}
    .card {{
      background: rgba(255, 250, 242, 0.96);
      border: 1px solid var(--line);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 10px 28px rgba(16, 33, 43, 0.06);
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
      background: rgba(255, 253, 248, 0.95);
      font-size: 0.92rem;
    }}
    .toolbar {{
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 12px;
      flex-wrap: wrap;
      margin: 18px 0;
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
    .calendar-grid {{
      display: grid;
      grid-template-columns: repeat(7, minmax(0, 1fr));
      gap: 8px;
    }}
    .day-head, .day-cell {{
      border: 1px solid var(--line);
      border-radius: 14px;
      background: #fffdf8;
      min-height: 110px;
      padding: 10px;
    }}
    .day-head {{
      min-height: auto;
      font-weight: 700;
      text-align: center;
      background: #f6f1e8;
    }}
    .day-number {{
      font-weight: 700;
      margin-bottom: 8px;
    }}
    .event-chip {{
      display: block;
      border-radius: 10px;
      padding: 6px 8px;
      font-size: 0.8rem;
      color: white;
      margin-bottom: 6px;
    }}
    .event-chip.WORK {{ background: var(--work); }}
    .event-chip.NO_WORK {{ background: var(--nowork); }}
    .event-chip.SAFETY {{ background: var(--safety); }}
    .muted {{ color: var(--muted); }}
    .legend {{
      display: flex;
      gap: 12px;
      flex-wrap: wrap;
      margin-top: 12px;
      font-size: 0.9rem;
    }}
  </style>
</head>
<body>
  <main>
    <section class="card">
      <h1>Schedule Calendar</h1>
      <div class="muted">View the currently active ICS schedule as a calendar month.</div>
      <div class="nav">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Schedule Calendar</a>
        <a class="nav-link" href="/map">Mission Map</a>
      </div>
    </section>
    <section class="toolbar">
      <button id="prev-month">Previous</button>
      <div id="month-label" style="font-size: 1.2rem; font-weight: 700;">Loading...</div>
      <button id="next-month">Next</button>
    </section>
    <section class="card">
      <div id="schedule-path" class="muted" style="margin-bottom: 12px;">Schedule: -</div>
      <div id="calendar-grid" class="calendar-grid"></div>
      <div class="legend">
        <span><strong style="color: var(--work);">WORK</strong> mission windows</span>
        <span><strong style="color: var(--nowork);">NO_WORK</strong> blackout windows</span>
        <span><strong style="color: var(--safety);">SAFETY</strong> logged safety events</span>
      </div>
    </section>
  </main>
  <script>
    let activeMonth = '';
    const weekdayNames = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'];

    function shiftMonth(month, delta) {{
      const [year, monthNumber] = month.split('-').map(Number);
      const date = new Date(year, monthNumber - 1 + delta, 1);
      return `${{date.getFullYear()}}-${{String(date.getMonth() + 1).padStart(2, '0')}}`;
    }}

    async function loadCalendar(month) {{
      const response = await fetch(`/api/schedule?month=${{encodeURIComponent(month)}}`, {{ cache: 'no-store' }});
      const data = await response.json();
      activeMonth = data.month;
      document.getElementById('month-label').textContent = data.month_name || data.month;
      document.getElementById('schedule-path').textContent = `Schedule: ${{data.schedule_path || '-'}}`;

      const grid = document.getElementById('calendar-grid');
      grid.innerHTML = '';
      for (const name of weekdayNames) {{
        const head = document.createElement('div');
        head.className = 'day-head';
        head.textContent = name;
        grid.appendChild(head);
      }}

      const [year, monthNumber] = data.month.split('-').map(Number);
      const firstDay = new Date(year, monthNumber - 1, 1);
      const monthDays = new Date(year, monthNumber, 0).getDate();
      const firstWeekday = (firstDay.getDay() + 6) % 7;
      const eventsByDate = new Map();
      for (const event of data.events || []) {{
        const existing = eventsByDate.get(event.date) || [];
        existing.push(event);
        eventsByDate.set(event.date, existing);
      }}

      for (let i = 0; i < firstWeekday; i += 1) {{
        const empty = document.createElement('div');
        empty.className = 'day-cell';
        grid.appendChild(empty);
      }}

      for (let day = 1; day <= monthDays; day += 1) {{
        const dateKey = `${{year}}-${{String(monthNumber).padStart(2, '0')}}-${{String(day).padStart(2, '0')}}`;
        const cell = document.createElement('div');
        cell.className = 'day-cell';
        const events = eventsByDate.get(dateKey) || [];
        cell.innerHTML = `<div class="day-number">${{day}}</div>`;
        for (const event of events) {{
          const chip = document.createElement('div');
          chip.className = `event-chip ${{event.schedule_type || 'WORK'}}`;
          chip.textContent = `${{event.time}} ${{event.summary || event.schedule_type || 'Event'}}`;
          chip.title = event.description || event.summary || '';
          cell.appendChild(chip);
        }}
        grid.appendChild(cell);
      }}
    }}

    document.getElementById('prev-month').addEventListener('click', async () => {{
      await loadCalendar(shiftMonth(activeMonth, -1));
    }});

    document.getElementById('next-month').addEventListener('click', async () => {{
      await loadCalendar(shiftMonth(activeMonth, 1));
    }});

    const now = new Date();
    loadCalendar(`${{now.getFullYear()}}-${{String(now.getMonth() + 1).padStart(2, '0')}}`);
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
  <title>{title} - Mission Map</title>
  <style>
    :root {{
      --bg: #f4f1ea;
      --card: #fffaf2;
      --ink: #10212b;
      --muted: #5d6c74;
      --accent: #0f766e;
      --line: #d9d3c7;
    }}
    body {{
      margin: 0;
      font-family: "Segoe UI", "Helvetica Neue", sans-serif;
      color: var(--ink);
      background: linear-gradient(160deg, #f5f1e7 0%, #ece8df 100%);
    }}
    main {{
      max-width: 1200px;
      margin: 0 auto;
      padding: 24px;
    }}
    .card {{
      background: rgba(255, 250, 242, 0.96);
      border: 1px solid var(--line);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 10px 28px rgba(16, 33, 43, 0.06);
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
      background: rgba(255, 253, 248, 0.95);
      font-size: 0.92rem;
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
      background: #fffdf8;
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
      background: #fffdf8;
    }}
    .muted {{ color: var(--muted); }}
    @media (max-width: 900px) {{
      .map-layout {{ grid-template-columns: 1fr; }}
    }}
  </style>
</head>
<body>
  <main>
    <section class="card">
      <h1>Mission Map</h1>
      <div class="muted">Preview built or decoded mission routes from the synced mission database.</div>
      <div class="nav">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Schedule Calendar</a>
        <a class="nav-link" href="/map">Mission Map</a>
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
  </main>
  <script>
    const palette = ['#0f766e', '#b45309', '#1d4ed8', '#be123c', '#4338ca', '#0f172a'];

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

      if (layers.length === 0) {{
        frame.textContent = 'No mission route geometry is available yet.';
        legend.innerHTML = '<div class="muted">Built route geometry will appear here once mission artifacts are available.</div>';
        return;
      }}

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
          <rect x="0" y="0" width="${{viewWidth}}" height="${{viewHeight}}" fill="#fffdf8" />
          <g opacity="0.15">
            <line x1="40" y1="40" x2="40" y2="${{viewHeight - 40}}" stroke="#10212b" />
            <line x1="40" y1="${{viewHeight - 40}}" x2="${{viewWidth - 40}}" y2="${{viewHeight - 40}}" stroke="#10212b" />
          </g>
          ${{polylines}}
        </svg>
      `;

      for (const layer of layers) {{
        const item = document.createElement('div');
        item.className = 'legend-item';
        item.innerHTML = `
          <div><strong>${{layer.mission_id}}</strong></div>
          <div class="muted">Color: <span style="color:${{layer.color}};">${{layer.color}}</span></div>
        `;
        legend.appendChild(item);
      }}
    }}

    async function loadMap() {{
      const response = await fetch('/api/map-data', {{ cache: 'no-store' }});
      const data = await response.json();
      renderMap(data.missions || [], data.active_route_geojson || null);
    }}

    loadMap();
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
