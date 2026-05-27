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
    CreateRecordedMission,
    EndMission,
    ExecuteMission,
    ListExecutableMissions,
    UploadVda5050Mission,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import Log
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger


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
        self._site_title = self.declare_parameter("site_title", "AMR-Sweeper").value
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
        self._create_recorded_mission_service = self.declare_parameter(
            "create_recorded_mission_service",
            "create_recorded_mission",
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
        self._safety_web_status_topic = self.declare_parameter(
            "safety_web_status_topic",
            "safety_controller/web_status",
        ).value
        self._clear_safety_stop_service = self.declare_parameter(
            "clear_safety_stop_service",
            "amr_sweeper_safety_controller/clear_safety_stop",
        ).value
        self._brand_logo_path = Path(__file__).resolve().parent.parent / "assets" / "logo_o_robotics.svg"

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
        self._create_recorded_mission_client = self.create_client(
            CreateRecordedMission,
            self._create_recorded_mission_service,
        )
        self._end_mission_client = self.create_client(
            EndMission,
            self._end_mission_service,
        )
        self._fsm_request_client = self.create_client(
            RequestState,
            self._fsm_request_service,
        )
        self._clear_safety_stop_client = self.create_client(
            Trigger,
            self._clear_safety_stop_service,
        )

        self._state_lock = threading.Lock()
        self._latest_fsm_state: dict[str, Any] | None = None
        self._latest_fsm_status: dict[str, Any] | None = None
        self._latest_navsat: dict[str, Any] | None = None
        self._latest_battery: dict[str, Any] | None = None
        self._latest_safety_status: dict[str, Any] | None = None
        self._recent_logs: deque[dict[str, Any]] = deque(maxlen=max(1, self._max_log_entries))

        self.create_subscription(FSMState, self._fsm_state_topic, self._handle_fsm_state, 10)
        self.create_subscription(FSMStatus, self._fsm_status_topic, self._handle_fsm_status, 10)
        self.create_subscription(NavSatFix, self._gnss_topic, self._handle_navsat, 10)
        self.create_subscription(BatteryState, self._battery_topic, self._handle_battery, 10)
        self.create_subscription(Log, self._rosout_topic, self._handle_rosout, 100)
        self.create_subscription(String, self._safety_web_status_topic, self._handle_safety_web_status, 10)

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
                if parsed.path == "/assets/logo-o-robotics.svg":
                    self._send_file(node._brand_logo_path, "image/svg+xml; charset=utf-8")
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

    def _handle_safety_web_status(self, message: String) -> None:
        try:
            decoded = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Ignoring invalid JSON on safety web status topic")
            return
        if not isinstance(decoded, dict):
            return
        with self._state_lock:
            self._latest_safety_status = decoded

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

    def start_record_map(self, payload: dict[str, Any]) -> dict[str, Any]:
        request_payload = dict(payload)
        request_payload.setdefault("reason", "record map requested from HTTP UI")
        request_payload.setdefault("priority", 200)
        return self.execute_manual_mission("RecordMap", request_payload)

    def stop_record_map(self, payload: dict[str, Any]) -> dict[str, Any]:
        request_payload = dict(payload)
        request_payload.setdefault("mission_id", "RecordMap")
        request_payload.setdefault("reason", "record map stop requested from HTTP UI")
        request_payload.setdefault("outcome", "completed")
        request_payload.setdefault("request_idling", True)
        return self.stop_active_mission(request_payload)

    def create_recorded_mission(self, payload: dict[str, Any]) -> dict[str, Any]:
        request = CreateRecordedMission.Request()
        request.mission_name = str(payload.get("mission_name", ""))
        request.sweep_pattern = str(payload.get("sweep_pattern", ""))
        request.overwrite_existing = bool(payload.get("overwrite_existing", False))

        response = self._call_service(
            self._create_recorded_mission_client,
            request,
            timeout_sec=15.0,
            service_name=self._create_recorded_mission_service,
        )
        return {
            "success": bool(response.success),
            "message": response.message,
            "mission_id": response.mission_id,
            "mission_file": response.mission_file,
            "mission_folder": response.mission_folder,
            "applied_sweep_pattern": response.applied_sweep_pattern,
            "latest_recorded_map_file": response.latest_recorded_map_file,
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

    def clear_safety_stop(self, payload: dict[str, Any]) -> dict[str, Any]:
        del payload
        response = self._call_service(
            self._clear_safety_stop_client,
            Trigger.Request(),
            timeout_sec=10.0,
            service_name=self._clear_safety_stop_service,
        )
        return {
            "success": bool(response.success),
            "message": response.message,
        }

    def status_snapshot(self) -> dict[str, Any]:
        with self._state_lock:
            fsm_state = dict(self._latest_fsm_state) if self._latest_fsm_state is not None else None
            fsm_status = dict(self._latest_fsm_status) if self._latest_fsm_status is not None else None
            navsat = dict(self._latest_navsat) if self._latest_navsat is not None else None
            battery = dict(self._latest_battery) if self._latest_battery is not None else None
            safety_status = dict(self._latest_safety_status) if self._latest_safety_status is not None else None
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
            "safety_stop": safety_status,
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

    def _discover_planned_schedule_path(self) -> Path | None:
        missions_from_db_directory = _resolve_path(self._missions_from_db_directory)
        candidates = sorted(
            missions_from_db_directory.glob("schedule_*.ics"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        return candidates[0] if candidates else None

    def _discover_actual_schedule_path(self) -> Path | None:
        active_execution = self._load_active_execution() or {}
        actual_schedule_log_path = active_execution.get("actual_schedule_log_path", "")
        if actual_schedule_log_path:
            path = Path(actual_schedule_log_path)
            if path.exists():
                return path

        missions_log_directory = _resolve_path(self._missions_log_directory)
        fixed_path = missions_log_directory / "actual_schedule.ics"
        if fixed_path.exists():
            return fixed_path
        return None

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

    def _load_schedule_events(self, schedule_path: Path | None) -> tuple[Path | None, list[dict[str, Any]]]:
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

    def _expand_event_occurrences(
        self,
        event: dict[str, Any],
        range_start: datetime,
        range_end: datetime,
    ) -> list[dict[str, Any]]:
        if "DTSTART" not in event:
            return []

        start = self._parse_ics_datetime(event["DTSTART"])
        end = (
            self._parse_ics_datetime(event["DTEND"])
            if "DTEND" in event
            else start + self._parse_ics_duration(event.get("DURATION", "PT0S"))
        )
        duration = end - start
        rrule = self._parse_rrule(event.get("RRULE", ""))
        occurrences: list[dict[str, Any]] = []

        def append_occurrence(occurrence_start: datetime) -> None:
            occurrence_end = occurrence_start + duration
            if occurrence_end <= range_start or occurrence_start >= range_end:
                return
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
                    "end_time": occurrence_end.strftime("%H:%M"),
                }
            )

        if not rrule:
            append_occurrence(start)
            return occurrences

        freq = rrule.get("FREQ", "")
        if freq == "DAILY":
            current = start
            while current + duration <= range_start:
                current += timedelta(days=1)
            while current < range_end:
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
                current_month = datetime(range_start.year, range_start.month, 1)
                last_month = datetime(range_end.year, range_end.month, 1)
                while current_month <= last_month:
                    occurrence_day = self._nth_weekday_of_month(
                        current_month.year,
                        current_month.month,
                        weekday,
                        bysetpos,
                    )
                    if occurrence_day is not None:
                        append_occurrence(
                            occurrence_day.replace(
                                hour=start.hour,
                                minute=start.minute,
                                second=start.second,
                            )
                        )
                    if current_month.month == 12:
                        current_month = datetime(current_month.year + 1, 1, 1)
                    else:
                        current_month = datetime(current_month.year, current_month.month + 1, 1)
            return occurrences

        append_occurrence(start)
        return occurrences

    def schedule_snapshot(self, week: str) -> dict[str, Any]:
        if week:
            selected_year, selected_week = week.split("-W", 1)
            week_start = datetime.fromisocalendar(int(selected_year), int(selected_week), 1)
        else:
            now = datetime.now()
            iso_year, iso_week, _ = now.isocalendar()
            week_start = datetime.fromisocalendar(iso_year, iso_week, 1)

        week_end = week_start + timedelta(days=7)
        planned_path, planned_events = self._load_schedule_events(self._discover_planned_schedule_path())
        actual_path, actual_events = self._load_schedule_events(self._discover_actual_schedule_path())
        planned_occurrences: list[dict[str, Any]] = []
        actual_occurrences: list[dict[str, Any]] = []
        for event in planned_events:
            for occurrence in self._expand_event_occurrences(event, week_start, week_end):
                occurrence["source"] = "planned"
                planned_occurrences.append(occurrence)
        for event in actual_events:
            for occurrence in self._expand_event_occurrences(event, week_start, week_end):
                occurrence["source"] = "actual"
                actual_occurrences.append(occurrence)
        planned_occurrences.sort(key=lambda item: item["start"])
        actual_occurrences.sort(key=lambda item: item["start"])

        iso_year, iso_week, _ = week_start.isocalendar()

        return {
            "success": True,
            "planned_schedule_path": str(planned_path) if planned_path is not None else "",
            "actual_schedule_path": str(actual_path) if actual_path is not None else "",
            "week": f"{iso_year:04d}-W{iso_week:02d}",
            "week_number": int(iso_week),
            "week_label": f"Week {iso_week:02d} · {week_start.strftime('%d %b')} - {(week_end - timedelta(days=1)).strftime('%d %b %Y')}",
            "week_start": week_start.strftime("%Y-%m-%d"),
            "week_end": (week_end - timedelta(days=1)).strftime("%Y-%m-%d"),
            "planned_events": planned_occurrences,
            "actual_events": actual_occurrences,
        }

    @staticmethod
    def _load_geojson_feature_collection(path: Path) -> dict[str, Any] | None:
        if not path.exists():
            return None
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return None

    def record_map_snapshot(self) -> dict[str, Any]:
        with self._state_lock:
            navsat = dict(self._latest_navsat) if self._latest_navsat is not None else None

        active_execution = self._load_active_execution() or {}
        active_recording = (
            active_execution.get("mission_id") == "RecordMap" and
            active_execution.get("active", True) is not False
        )
        active_navsat_geojson = None
        active_navsat_file = active_execution.get("actual_path_navsat_file", "")
        if active_navsat_file:
            active_navsat_geojson = self._load_geojson_feature_collection(Path(active_navsat_file))
        elif active_execution.get("execution_context_file"):
            try:
                context_document = json.loads(
                    Path(active_execution["execution_context_file"]).read_text(encoding="utf-8")
                )
                navsat_path = context_document.get("actual_path_navsat_file", "")
                if navsat_path:
                    active_navsat_geojson = self._load_geojson_feature_collection(Path(navsat_path))
            except Exception:
                active_navsat_geojson = None

        latest_directory = _resolve_path(self._missions_log_directory) / "latest_recorded_map"
        latest_metadata_file = latest_directory / "latest_recorded_map.json"
        latest_metadata = None
        latest_route_geojson = None
        latest_navsat_geojson = None
        if latest_metadata_file.exists():
            try:
                latest_metadata = json.loads(latest_metadata_file.read_text(encoding="utf-8"))
                route_path = Path(latest_metadata.get("recorded_work_area_route_file", ""))
                navsat_path = Path(latest_metadata.get("recorded_work_area_navsat_file", ""))
                if route_path:
                    latest_route_geojson = self._load_geojson_feature_collection(route_path)
                if navsat_path:
                    latest_navsat_geojson = self._load_geojson_feature_collection(navsat_path)
            except Exception as exc:  # noqa: BLE001
                latest_metadata = {"error": str(exc), "path": str(latest_metadata_file)}

        return {
            "success": True,
            "patterns": ["zigzag", "random", "spiral"],
            "default_pattern": "zigzag",
            "active_recording": active_recording,
            "active_execution": active_execution,
            "current_position": navsat,
            "active_navsat_geojson": active_navsat_geojson,
            "latest_recorded_map": latest_metadata,
            "latest_route_geojson": latest_route_geojson,
            "latest_navsat_geojson": latest_navsat_geojson,
        }

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
        missions_log_directory = _resolve_path(self._missions_log_directory)
        missions: list[dict[str, Any]] = []
        mission_files = list(missions_directory.glob("*.json"))
        mission_files.extend(missions_directory.glob("*/*.json"))
        seen_paths: set[Path] = set()
        for mission_file in sorted(mission_files):
            if mission_file in seen_paths:
                continue
            seen_paths.add(mission_file)
            mission_id = mission_file.stem
            document = self._load_geojson_feature_collection(mission_file)
            route_geojson = None
            route_path = mission_file.parent / f"{mission_id}_path.geojson"
            if not route_path.exists():
                route_path = missions_log_directory / mission_id / f"{mission_id}_path.geojson"
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
    .brand-band {{
      width: min(280px, 100%);
      height: 12px;
      border-radius: 999px;
      background:
        linear-gradient(90deg, var(--accent) 0 24%, transparent 24% 28%, var(--accent) 28% 52%, transparent 52% 56%, var(--accent) 56% 100%),
        repeating-linear-gradient(135deg, rgba(0, 0, 0, 0.35) 0 8px, rgba(0, 0, 0, 0.1) 8px 16px);
      box-shadow: inset 0 0 0 1px rgba(253, 202, 15, 0.32);
    }}
    .brand-logo {{
      width: min(320px, 70vw);
      height: auto;
      display: block;
      filter: brightness(0) saturate(100%) invert(85%) sepia(64%) saturate(872%) hue-rotate(356deg) brightness(103%) contrast(98%);
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
    .mission-list {{
      display: grid;
      gap: 12px;
      margin-top: 16px;
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
    .cause-list {{
      display: grid;
      gap: 8px;
      margin-top: 12px;
    }}
    .cause-item {{
      border: 1px solid var(--line);
      border-radius: 12px;
      padding: 10px 12px;
      background: rgba(10, 16, 12, 0.5);
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
      <h1>{title}</h1>
      <img class="brand-logo" src="/assets/logo-o-robotics.svg" alt="O-Robotics logo">
      <div class="brand-band"></div>
      <div id="banner" class="banner"></div>
      <div class="nav">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Calendar</a>
        <a class="nav-link" href="/map">Missions</a>
        <a class="nav-link" href="/developer">Developer</a>
        <a class="nav-link" href="/record-map">Record Map</a>
      </div>
    </section>

    <section class="grid">
      <article id="safety-card" class="card safety-card status-card">
        <h2>System Status</h2>
        <div class="status-sections">
          <section class="status-panel">
            <h3>FSM</h3>
            <div id="fsm-state" class="stat">Waiting...</div>
            <div id="fsm-profile" class="muted">Profile: -</div>
            <div id="fsm-transition" class="muted">Transition: -</div>
          </section>
          <section class="status-panel">
            <h3>Active Mission</h3>
            <div id="active-mission" class="stat">No Active Missions</div>
            <div id="active-directory" class="muted">Run folder: -</div>
            <div class="actions">
              <button class="stop" id="stop-button" disabled>Stop Mission</button>
              <button id="reboot-button">Reboot</button>
            </div>
          </section>
          <section class="status-panel">
            <h3>Safety Stop</h3>
            <div id="safety-state" class="stat">Clear</div>
            <div id="safety-summary" class="muted">No active safety stop.</div>
            <div class="actions">
              <button class="stop" id="clear-safety-button">Clear Safety Stop</button>
            </div>
            <div id="safety-causes" class="cause-list"></div>
          </section>
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
  </main>

  <script>
    const banner = document.getElementById('banner');
    const missionList = document.getElementById('mission-list');

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
      const safetyCauses = Array.isArray(safety.causes) ? safety.causes : [];
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

      const safetyCard = document.getElementById('safety-card');
      const safetyLatched = Boolean(safety.latched);
      document.getElementById('safety-state').textContent = safetyLatched ? 'Latched' : 'Clear';
      document.getElementById('safety-summary').textContent = safetyLatched
        ? `${{safety.active_sender || 'unknown_sender'}}: ${{safety.active_reason || 'stop requested'}}`
        : 'No active safety stop.';
      document.getElementById('clear-safety-button').disabled = !safetyLatched;
      safetyCard.classList.toggle('active', safetyLatched);

      const safetyCauseList = document.getElementById('safety-causes');
      safetyCauseList.innerHTML = '';
      if (!safetyLatched || safetyCauses.length === 0) {{
        safetyCauseList.innerHTML = '<div class="muted">No latched safety-stop causes are being reported.</div>';
      }} else {{
        for (const cause of safetyCauses) {{
          const item = document.createElement('div');
          item.className = 'cause-item';
          item.innerHTML = `
            <strong>${{cause.sender || 'unknown_sender'}}</strong><br>
            <span class="muted">${{cause.reason || 'stop requested'}}</span>
          `;
          safetyCauseList.appendChild(item);
        }}
      }}

      document.getElementById('active-mission').textContent =
        hasActiveMission ? active.mission_id : 'No Active Missions';
      document.getElementById('active-directory').textContent =
        `Run folder: ${{hasActiveMission ? (active.mission_run_directory || '-') : '-'}}`;
      document.getElementById('stop-button').disabled = !hasActiveMission;
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

    document.getElementById('clear-safety-button').addEventListener('click', async () => {{
      const response = await fetch('/api/safety/clear', {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/json' }},
        body: JSON.stringify({{}})
      }});
      const data = await response.json();
      setBanner(data.success ? 'ok' : 'error', data.message || 'Safety clear request completed');
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
    .brand-band {{
      width: min(240px, 100%);
      height: 12px;
      margin: 10px 0 0;
      border-radius: 999px;
      background:
        linear-gradient(90deg, var(--accent) 0 24%, transparent 24% 28%, var(--accent) 28% 52%, transparent 52% 56%, var(--accent) 56% 100%),
        repeating-linear-gradient(135deg, rgba(0, 0, 0, 0.35) 0 8px, rgba(0, 0, 0, 0.1) 8px 16px);
    }}
    .brand-logo {{
      width: min(300px, 68vw);
      height: auto;
      display: block;
      margin-top: 10px;
      filter: brightness(0) saturate(100%) invert(85%) sepia(64%) saturate(872%) hue-rotate(356deg) brightness(103%) contrast(98%);
    }}
    .brand-logo {{
      width: min(300px, 68vw);
      height: auto;
      display: block;
      margin-top: 10px;
      filter: brightness(0) saturate(100%) invert(85%) sepia(64%) saturate(872%) hue-rotate(356deg) brightness(103%) contrast(98%);
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
    @media (max-width: 900px) {{
      .toolbar {{
        align-items: flex-start;
      }}
    }}
  </style>
</head>
<body>
  <main>
    <section class="card">
      <h1>Calendar</h1>
      <img class="brand-logo" src="/assets/logo-o-robotics.svg" alt="O-Robotics logo">
      <div class="brand-band"></div>
      <div class="muted">View the active schedule as a weekly planner with full 24-hour day lanes.</div>
      <div class="nav">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Calendar</a>
        <a class="nav-link" href="/map">Missions</a>
        <a class="nav-link" href="/developer">Developer</a>
        <a class="nav-link" href="/record-map">Record Map</a>
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
      </div>
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

    function minutesSinceMidnight(dateText) {{
      const date = new Date(dateText);
      return (date.getHours() * 60) + date.getMinutes();
    }}

    function startOfDay(date) {{
      return new Date(date.getFullYear(), date.getMonth(), date.getDate());
    }}

    function clamp(value, min, max) {{
      return Math.max(min, Math.min(max, value));
    }}

    async function loadCalendar(week) {{
      const response = await fetch(`/api/schedule?week=${{encodeURIComponent(week)}}`, {{ cache: 'no-store' }});
      const data = await response.json();
      activeWeek = data.week;
      document.getElementById('week-label').textContent = data.week_label || data.week;
      document.getElementById('week-number').textContent = `CW ${{data.week_number ?? '--'}}`;
      document.getElementById('schedule-path').textContent =
        `Planned: ${{data.planned_schedule_path || '-'}} | Actual: ${{data.actual_schedule_path || '-'}}`;

      const grid = document.getElementById('calendar-grid');
      grid.innerHTML = '';
      const weekStart = new Date(`${{data.week_start}}T00:00:00`);

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
          const start = new Date(event.start);
          const end = new Date(event.end);
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
                <div class="event-time">${{segmentStart.toLocaleTimeString([], {{ hour: '2-digit', minute: '2-digit', hour12: false }})}} - ${{segmentEnd >= nextDay ? '24:00' : segmentEnd.toLocaleTimeString([], {{ hour: '2-digit', minute: '2-digit', hour12: false }})}}</div>
                <div><strong>${{event.summary || event.schedule_type || 'Event'}}</strong></div>
                <div>${{event.mission_id || event.robot_id || ''}}</div>
                <div class="event-source">${{sourceLabel}}</div>
              `;
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

    loadCalendar(toIsoWeekString(new Date()));
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
    .brand-band {{
      width: min(260px, 100%);
      height: 12px;
      border-radius: 999px;
      background:
        linear-gradient(90deg, var(--accent) 0 24%, transparent 24% 28%, var(--accent) 28% 52%, transparent 52% 56%, var(--accent) 56% 100%),
        repeating-linear-gradient(135deg, rgba(0, 0, 0, 0.35) 0 8px, rgba(0, 0, 0, 0.1) 8px 16px);
    }}
    .brand-logo {{
      width: min(320px, 72vw);
      height: auto;
      display: block;
      filter: brightness(0) saturate(100%) invert(85%) sepia(64%) saturate(872%) hue-rotate(356deg) brightness(103%) contrast(98%);
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
      <img class="brand-logo" src="/assets/logo-o-robotics.svg" alt="O-Robotics logo">
      <div class="brand-band"></div>
      <div class="muted">Drive the robot around the working-area perimeter, let RecordMap update the latest recorded map, then create one or more named autonomous missions from that map using a sweep pattern.</div>
      <div id="banner" class="banner"></div>
      <div class="nav">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Calendar</a>
        <a class="nav-link" href="/map">Missions</a>
        <a class="nav-link" href="/developer">Developer</a>
        <a class="nav-link" href="/record-map">Record Map</a>
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
    .brand-band {{
      width: min(240px, 100%);
      height: 12px;
      margin: 10px 0 0;
      border-radius: 999px;
      background:
        linear-gradient(90deg, var(--accent) 0 24%, transparent 24% 28%, var(--accent) 28% 52%, transparent 52% 56%, var(--accent) 56% 100%),
        repeating-linear-gradient(135deg, rgba(0, 0, 0, 0.35) 0 8px, rgba(0, 0, 0, 0.1) 8px 16px);
    }}
    .brand-logo {{
      width: min(300px, 68vw);
      height: auto;
      display: block;
      margin-top: 10px;
      filter: brightness(0) saturate(100%) invert(85%) sepia(64%) saturate(872%) hue-rotate(356deg) brightness(103%) contrast(98%);
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
      <img class="brand-logo" src="/assets/logo-o-robotics.svg" alt="O-Robotics logo">
      <div class="brand-band"></div>
      <div class="muted">Preview built or decoded mission routes from the synced mission database, and upload VDA5050 missions.</div>
      <div id="banner" class="banner"></div>
      <div class="nav">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Calendar</a>
        <a class="nav-link" href="/map">Missions</a>
        <a class="nav-link" href="/developer">Developer</a>
        <a class="nav-link" href="/record-map">Record Map</a>
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
          <rect x="0" y="0" width="${{viewWidth}}" height="${{viewHeight}}" fill="#2c3032" />
          <g opacity="0.15">
            <line x1="40" y1="40" x2="40" y2="${{viewHeight - 40}}" stroke="#eef3eb" />
            <line x1="40" y1="${{viewHeight - 40}}" x2="${{viewWidth - 40}}" y2="${{viewHeight - 40}}" stroke="#eef3eb" />
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

    loadMap();
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
    .brand-band {{
      width: min(240px, 100%);
      height: 12px;
      margin: 10px 0 0;
      border-radius: 999px;
      background:
        linear-gradient(90deg, #fdca0f 0 24%, transparent 24% 28%, #fdca0f 28% 52%, transparent 52% 56%, #fdca0f 56% 100%),
        repeating-linear-gradient(135deg, rgba(0, 0, 0, 0.35) 0 8px, rgba(0, 0, 0, 0.1) 8px 16px);
    }}
    .brand-logo {{
      width: min(300px, 68vw);
      height: auto;
      display: block;
      margin-top: 10px;
      filter: brightness(0) saturate(100%) invert(85%) sepia(64%) saturate(872%) hue-rotate(356deg) brightness(103%) contrast(98%);
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
      <img class="brand-logo" src="/assets/logo-o-robotics.svg" alt="O-Robotics logo">
      <div class="brand-band"></div>
      <div class="muted">Inspect recent ROS warning/error logs and the raw web status payload.</div>
      <div class="nav">
        <a class="nav-link" href="/">Dashboard</a>
        <a class="nav-link" href="/calendar">Calendar</a>
        <a class="nav-link" href="/map">Missions</a>
        <a class="nav-link" href="/developer">Developer</a>
        <a class="nav-link" href="/record-map">Record Map</a>
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
