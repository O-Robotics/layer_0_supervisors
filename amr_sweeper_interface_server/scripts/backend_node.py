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

import calendar
from collections import deque
from datetime import datetime, timedelta, timezone
import errno
from http import HTTPStatus
import json
import os
from pathlib import Path
import re
import shutil
import socket
import socketserver
import stat
import struct
import threading
import time
from typing import Any
from uuid import uuid4
from zoneinfo import ZoneInfo

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from amr_sweeper_fsm.msg import FSMState, FSMStatus
from amr_sweeper_fsm.srv import RequestState
from amr_sweeper_mission_builder.srv import (
    BuildGaussianSplat,
    PauseGaussianSplatBuild,
    ResumeGaussianSplatBuild,
)
from amr_sweeper_mission_executor.srv import (
    CreateRecordedMission,
    EndMission,
    ExecuteMission,
    ListExecutableMissions,
    UploadVda5050Mission,
)
from amr_sweeper_safety_msgs.msg import SafetyStop
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
import yaml

MISSION_LAYER_OVERRIDE_KEYS = (
    "use_amr_sweeper_ros2_control",
    "use_amr_sweeper_battery",
    "use_amr_sweeper_system_info",
    "use_amr_sweeper_usb_cameras",
    "use_amr_sweeper_depth_camera",
    "use_amr_sweeper_imu",
    "use_amr_sweeper_gnss",
    "use_ntrip_client",
    "use_amr_sweeper_drive_controller",
    "use_amr_sweeper_tool_controller",
    "use_amr_sweeper_teleop",
    "use_amr_sweeper_sweeping_controller",
    "use_amr_sweeper_attitude_controller",
    "use_amr_sweeper_collision_detector",
    "use_amr_sweeper_safety_controller",
    "use_joy_node",
    "use_amr_sweeper_visual_odometry",
    "use_amr_sweeper_localization",
    "use_amr_sweeper_mapping",
    "use_amr_sweeper_navigation",
    "use_gaussian",
    "auto_start_mission",
)

MISSION_EXECUTION_BOOLEAN_KEYS = (
    "record_rosbag",
)

MISSION_LAYER_OVERRIDE_FALLBACKS = {
    "use_amr_sweeper_ros2_control": True,
    "use_amr_sweeper_battery": True,
    "use_amr_sweeper_system_info": True,
    "use_amr_sweeper_usb_cameras": True,
    "use_amr_sweeper_depth_camera": True,
    "use_amr_sweeper_imu": True,
    "use_amr_sweeper_gnss": True,
    "use_ntrip_client": True,
    "use_amr_sweeper_drive_controller": True,
    "use_amr_sweeper_tool_controller": True,
    "use_amr_sweeper_teleop": True,
    "use_amr_sweeper_sweeping_controller": True,
    "use_amr_sweeper_attitude_controller": True,
    "use_amr_sweeper_collision_detector": True,
    "use_amr_sweeper_safety_controller": True,
    "use_joy_node": False,
    "use_amr_sweeper_visual_odometry": False,
    "use_amr_sweeper_localization": True,
    "use_amr_sweeper_mapping": False,
    "use_amr_sweeper_navigation": True,
    "use_gaussian": False,
    "auto_start_mission": True,
}

TELEOP_MISSION_ID = "Teleop"
TELEOP_PROFILE_ID = 220
TELEOP_DRIVE_LINEAR_SCALE = 0.5
TELEOP_DRIVE_ANGULAR_SCALE = 0.785
TELEOP_TOOL_LINEAR_SCALE = 0.10
TELEOP_TOOL_ANGULAR_SCALE = 0.10
TELEOP_INPUT_DEADZONE = 0.05
TELEOP_DEFAULT_SPEED_SCALE = 0.5

LED_MODULE_COMMAND_IDS = {
    "front_left": 0x400,
    "front_right": 0x425,
    "rear_left": 0x450,
    "rear_right": 0x475,
}

ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-9;]*m")


def _resolve_path(configured_path: str) -> Path:
    path = Path(configured_path).expanduser()
    if path.is_absolute():
        return path
    return Path.cwd() / path


def _existing_paths(candidates: list[Path]) -> list[Path]:
    return [path for path in candidates if path.exists()]


def _execution_context_candidates(missions_log_directory: Path) -> list[Path]:
    candidates = list(missions_log_directory.rglob("*_context.json"))
    legacy_candidates = list(missions_log_directory.rglob("execution_context.json"))
    return sorted({*candidates, *legacy_candidates})


def _escape_ics_text(value: str) -> str:
    return (
        str(value)
        .replace("\\", "\\\\")
        .replace("\n", "\\n")
        .replace(",", "\\,")
        .replace(";", "\\;")
    )


def _unescape_ics_text(value: str) -> str:
    return (
        str(value)
        .replace("\\n", "\n")
        .replace("\\,", ",")
        .replace("\\;", ";")
        .replace("\\\\", "\\")
    )


def _coerce_bool_value(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes", "on"}:
            return True
        if lowered in {"false", "0", "no", "off"}:
            return False
    return None


def _parse_transition_completion_message(message: str) -> tuple[str, int] | None:
    prefix = "FSM state transition completed, now running:"
    if not isinstance(message, str) or prefix not in message:
        return None
    remainder = message.split(prefix, 1)[1].strip()
    if not remainder or "(" not in remainder or ")" not in remainder:
        return None
    state_part, profile_part = remainder.rsplit("(", 1)
    state = state_part.strip()
    profile_text = profile_part.split(")", 1)[0].strip()
    try:
        profile = int(profile_text)
    except (TypeError, ValueError):
        return None
    if not state:
        return None
    return state, profile


def _strip_ansi_text(message: str) -> str:
    if not isinstance(message, str):
        return ""
    return ANSI_ESCAPE_RE.sub("", message)


def _parse_started_command_from_log(message: str) -> str | None:
    normalized = _strip_ansi_text(message).strip()
    prefix = "Started: "
    if not normalized.startswith(prefix):
        return None
    command = normalized[len(prefix):].strip()
    return command or None


def _parse_command_launch_arg(command: str, key: str) -> str | None:
    prefix = f"{key}:="
    for token in str(command).split():
        if token.startswith(prefix):
            return token[len(prefix):]
    return None


def _parse_command_launch_bool(command: str, key: str, default: bool) -> bool:
    value = _parse_command_launch_arg(command, key)
    if value is None:
        return default
    parsed = _coerce_bool_value(value)
    return default if parsed is None else parsed


def _parse_command_launch_float(command: str, key: str, default: float) -> float:
    value = _parse_command_launch_arg(command, key)
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _layer3_transition_progress_steps_from_command(command: str) -> list[tuple[float, str]]:
    steps: list[tuple[float, str]] = []
    if _parse_command_launch_bool(command, "use_amr_sweeper_localization", True):
        steps.append((0.0, "Starting Localization"))
    if _parse_command_launch_bool(command, "use_amr_sweeper_mapping", True):
        steps.append((
            max(0.0, _parse_command_launch_float(command, "mapping_startup_delay_sec", 0.0)),
            "Starting Mapping",
        ))
    if _parse_command_launch_bool(command, "use_amr_sweeper_navigation", True):
        navigation_delay_sec = _parse_command_launch_float(command, "navigation_startup_delay_sec", 3.0)
        waypoint_delay_sec = _parse_command_launch_float(command, "waypoint_follower_startup_delay_sec", 3.0)
        if navigation_delay_sec == 3.0 and waypoint_delay_sec != 3.0:
            navigation_delay_sec = waypoint_delay_sec
        if navigation_delay_sec == 3.0:
            navigation_delay_sec = 5.0
        steps.append((max(0.0, navigation_delay_sec), "Starting Navigation"))
    return sorted(steps, key=lambda item: item[0])


def _transition_progress_message_from_log(message: str) -> str | None:
    if not isinstance(message, str):
        return None
    normalized = _strip_ansi_text(message).strip()
    if normalized == "Critical profile process ready: amr_sweeper_layer_1_hardware_bringup":
        return "Hardware ready"
    if normalized == "Critical profile process ready: amr_sweeper_layer_2_controllers_bringup":
        return "Controllers ready"
    return None


def _write_execution_context_preferences(
    execution_context_file: str,
    layer_overrides: Any,
    boolean_preferences: dict[str, Any],
) -> None:
    if not execution_context_file:
        return

    context_path = Path(execution_context_file)
    if not context_path.exists():
        return
    try:
        context_document = json.loads(context_path.read_text(encoding="utf-8"))
    except Exception:
        return
    if not isinstance(context_document, dict):
        return

    normalized_overrides: dict[str, bool] = {}
    if isinstance(layer_overrides, dict):
        for key, value in layer_overrides.items():
            if key not in MISSION_LAYER_OVERRIDE_KEYS:
                continue
            parsed = _coerce_bool_value(value)
            if parsed is None:
                continue
            normalized_overrides[key] = parsed
    if normalized_overrides:
        context_document["layer_overrides"] = normalized_overrides

    for key in MISSION_EXECUTION_BOOLEAN_KEYS:
        if key not in boolean_preferences:
            continue
        parsed = _coerce_bool_value(boolean_preferences.get(key))
        if parsed is None:
            continue
        context_document[key] = parsed

    try:
        context_path.write_text(json.dumps(context_document, indent=2), encoding="utf-8")
    except Exception:
        return


def _running_profiles_yaml_path() -> Path:
    candidates: list[Path] = []
    try:
        candidates.append(
            Path(get_package_share_directory("amr_sweeper_fsm")) / "config" / "profiles" / "running_profiles.yaml"
        )
    except PackageNotFoundError:
        pass
    candidates.append(
        Path.cwd() / "src" / "layer_0_supervisors" / "amr_sweeper_fsm" / "config" / "profiles" / "running_profiles.yaml"
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[-1]


def _load_running_profile_default_overrides() -> dict[int, dict[str, bool]]:
    path = _running_profiles_yaml_path()
    if not path.exists():
        return {}
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception:
        return {}
    if not isinstance(document, dict):
        return {}

    overrides_by_profile: dict[int, dict[str, bool]] = {}
    for entry in document.get("profiles", []):
        profile = entry.get("profile", {}) if isinstance(entry, dict) else {}
        try:
            profile_id = int(profile.get("id"))
        except (TypeError, ValueError):
            continue
        profile_overrides: dict[str, bool] = {}
        for process in profile.get("processes", []):
            startup = process.get("startup", {}) if isinstance(process, dict) else {}
            for argument in startup.get("args", []):
                if not isinstance(argument, str) or ":=" not in argument:
                    continue
                key, raw_value = argument.split(":=", 1)
                if key not in MISSION_LAYER_OVERRIDE_KEYS:
                    continue
                parsed = _coerce_bool_value(raw_value)
                if parsed is None:
                    continue
                profile_overrides[key] = parsed
        if profile_overrides:
            overrides_by_profile[profile_id] = profile_overrides
    return overrides_by_profile


DEFAULT_BACKEND_SOCKET_PATH = "/tmp/amr_sweeper_interface_backend.sock"


class MissionBackendUnixJSONLServer(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):

    daemon_threads = True


class MissionBackendNode(Node):

    def __init__(self, node_name: str = "backend_node") -> None:
        super().__init__(node_name)

        self._backend_socket_path = str(
            self.declare_parameter("backend_socket_path", DEFAULT_BACKEND_SOCKET_PATH).value
        )
        self._site_title = self.declare_parameter("site_title", "AMR-Sweeper").value
        self._public_base_url = self.declare_parameter(
            "public_base_url",
            "http://192.168.2.1:8080",
        ).value
        self._missions_log_directory = self.declare_parameter(
            "missions_log_directory",
            "missions/logs",
        ).value
        self._actual_schedule_log_directory = self.declare_parameter(
            "actual_schedule_log_directory",
            "missions/logs",
        ).value
        self._missions_from_db_directory = self.declare_parameter(
            "missions_from_db_directory",
            "missions/database",
        ).value
        self._maps_directory = self.declare_parameter(
            "maps_directory",
            "missions/maps",
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
        self._build_gaussian_splat_service = self.declare_parameter(
            "build_gaussian_splat_service",
            "build_gaussian_splat",
        ).value
        self._pause_gaussian_splat_build_service = self.declare_parameter(
            "pause_gaussian_splat_build_service",
            "pause_gaussian_splat_build",
        ).value
        self._resume_gaussian_splat_build_service = self.declare_parameter(
            "resume_gaussian_splat_build_service",
            "resume_gaussian_splat_build",
        ).value
        self._gaussian_splat_status_service = self.declare_parameter(
            "gaussian_splat_status_service",
            "get_gaussian_splat_status",
        ).value
        self._end_mission_service = self.declare_parameter(
            "end_mission_service",
            "end_mission",
        ).value
        self._fsm_request_service = self.declare_parameter(
            "fsm_request_service",
            "request_state",
        ).value
        self._fsm_state_topic = self.declare_parameter(
            "fsm_state_topic", "fsm/supervisor_node/fsm_state").value
        self._fsm_status_topic = self.declare_parameter(
            "fsm_status_topic", "fsm/supervisor_node/fsm_status").value
        self._gnss_topic = self.declare_parameter("gnss_topic", "gnss/navsat").value
        self._battery_topic = self.declare_parameter(
            "battery_topic", "battery/battery_state").value
        self._rosout_topic = self.declare_parameter("rosout_topic", "/rosout").value
        self._max_log_entries = int(self.declare_parameter("max_log_entries", 100).value)
        self._safety_web_status_topic = self.declare_parameter(
            "safety_web_status_topic",
            "safety_controller/web_status",
        ).value
        self._safety_stop_topic = self.declare_parameter(
            "safety_stop_topic",
            "safety_msgs/stop",
        ).value
        self._clear_safety_stop_service = self.declare_parameter(
            "clear_safety_stop_service",
            "amr_sweeper_safety_controller/clear_safety_stop",
        ).value
        self._safety_clear_min_delay_sec = float(
            self.declare_parameter("safety_clear_min_delay_sec", 2.0).value
        )
        self._teleop_drive_command_topic = self.declare_parameter(
            "teleop_drive_command_topic",
            "teleop/cmd_vel_drive",
        ).value
        self._teleop_tool_command_topic = self.declare_parameter(
            "teleop_tool_command_topic",
            "teleop/cmd_vel_tools",
        ).value
        self._teleop_control_mode_topic = self.declare_parameter(
            "teleop_control_mode_topic",
            "teleop/control_mode",
        ).value
        self._teleop_tool_scale_topic = self.declare_parameter(
            "teleop_tool_scale_topic",
            "teleop/tool_scale",
        ).value
        self._teleop_wheel_scale_topic = self.declare_parameter(
            "teleop_wheel_scale_topic",
            "teleop/wheel_scale",
        ).value
        self._led_can_interface = str(self.declare_parameter("led_can_interface", "can0").value)
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
        self._build_gaussian_splat_client = self.create_client(
            BuildGaussianSplat,
            self._build_gaussian_splat_service,
        )
        self._pause_gaussian_splat_build_client = self.create_client(
            PauseGaussianSplatBuild,
            self._pause_gaussian_splat_build_service,
        )
        self._resume_gaussian_splat_build_client = self.create_client(
            ResumeGaussianSplatBuild,
            self._resume_gaussian_splat_build_service,
        )
        self._gaussian_splat_status_client = self.create_client(
            Trigger,
            self._gaussian_splat_status_service,
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
        safety_stop_qos = QoSProfile(depth=10)
        safety_stop_qos.reliability = ReliabilityPolicy.RELIABLE
        safety_stop_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._safety_stop_publisher = self.create_publisher(
            SafetyStop,
            self._safety_stop_topic,
            safety_stop_qos,
        )
        self._teleop_drive_publisher = self.create_publisher(
            Twist,
            self._teleop_drive_command_topic,
            10,
        )
        self._teleop_tool_publisher = self.create_publisher(
            Twist,
            self._teleop_tool_command_topic,
            10,
        )
        self._teleop_control_mode_publisher = self.create_publisher(
            String,
            self._teleop_control_mode_topic,
            10,
        )
        self._teleop_tool_scale_publisher = self.create_publisher(
            Float32,
            self._teleop_tool_scale_topic,
            10,
        )
        self._teleop_wheel_scale_publisher = self.create_publisher(
            Float32,
            self._teleop_wheel_scale_topic,
            10,
        )

        self._state_lock = threading.Lock()
        self._led_can_lock = threading.Lock()
        self._led_can_socket: socket.socket | None = None
        self._led_lights_enabled = False
        self._latest_fsm_state: dict[str, Any] | None = None
        self._latest_fsm_status: dict[str, Any] | None = None
        self._latest_navsat: dict[str, Any] | None = None
        self._latest_battery: dict[str, Any] | None = None
        self._latest_safety_status: dict[str, Any] | None = None
        self._recent_logs: deque[dict[str, Any]] = deque(maxlen=max(1, self._max_log_entries))
        self._last_cleared_running_profile: int | None = None
        self._display_fsm_state: str | None = None
        self._display_fsm_profile: int | None = None
        self._display_transition_active = False
        self._display_transition_progress = ""
        self._scheduled_transition_progress_steps: list[tuple[float, str]] = []
        self._scheduled_transition_progress_started_at: float | None = None

        self.create_subscription(FSMState, self._fsm_state_topic, self._handle_fsm_state, 10)
        self.create_subscription(FSMStatus, self._fsm_status_topic, self._handle_fsm_status, 10)
        self.create_subscription(NavSatFix, self._gnss_topic, self._handle_navsat, 10)
        battery_qos = QoSProfile(depth=10)
        battery_qos.reliability = ReliabilityPolicy.RELIABLE
        battery_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(BatteryState, self._battery_topic, self._handle_battery, battery_qos)
        self.create_subscription(Log, self._rosout_topic, self._handle_rosout, 100)
        self.create_subscription(String, self._safety_web_status_topic, self._handle_safety_web_status, 10)

        self._ipc_server: MissionBackendUnixJSONLServer | None = None

    def start_ipc_server(self) -> None:
        handler = self._build_handler()
        socket_path = Path(self._backend_socket_path)
        if socket_path.exists():
            if not stat.S_ISSOCK(socket_path.stat().st_mode):
                raise RuntimeError(f"Backend socket path exists and is not a socket: {socket_path}")
            socket_path.unlink()
        socket_path.parent.mkdir(parents=True, exist_ok=True)
        old_umask = os.umask(0o177)
        try:
            self._ipc_server = MissionBackendUnixJSONLServer(str(socket_path), handler)
        except OSError as exc:
            os.umask(old_umask)
            if exc.errno == errno.EADDRINUSE:
                raise RuntimeError(f"Backend socket path is already in use: {socket_path}") from exc
            raise
        finally:
            os.umask(old_umask)
        socket_path.chmod(0o600)
        self.get_logger().info(
            f"Interface backend raw JSONL API listening on Unix socket {socket_path}"
        )

    def stop_ipc_server(self) -> None:
        with self._led_can_lock:
            self._close_led_can_socket_locked()
        if self._ipc_server is None:
            return
        self._ipc_server.shutdown()
        self._ipc_server.server_close()
        self._ipc_server = None
        socket_path = Path(self._backend_socket_path)
        try:
            if socket_path.exists() and stat.S_ISSOCK(socket_path.stat().st_mode):
                socket_path.unlink()
        except OSError as exc:
            self.get_logger().warning(f"Failed to remove backend socket {socket_path}: {exc}")

    def serve_forever(self) -> None:
        if self._ipc_server is None:
            raise RuntimeError("Backend IPC server not initialized")
        self._ipc_server.serve_forever()

    def _build_handler(self):
        node = self

        class MissionBackendRequestHandler(socketserver.StreamRequestHandler):

            def handle(self) -> None:
                try:
                    request = node._read_raw_ipc_request(self.rfile)
                    response = node._dispatch_ipc_request(request)
                except ValueError as exc:
                    response = node._ipc_error_response(HTTPStatus.BAD_REQUEST, str(exc))
                except Exception as exc:  # noqa: BLE001
                    node.get_logger().warning(f"Backend IPC request failed: {exc}")
                    response = node._ipc_error_response(HTTPStatus.BAD_GATEWAY, str(exc))
                encoded = json.dumps(response, separators=(",", ":")).encode("utf-8") + b"\n"
                try:
                    self.wfile.write(encoded)
                except OSError as exc:
                    if self._is_client_disconnect(exc):
                        node.get_logger().debug(
                            f"Backend IPC client disconnected before response completed: {exc}"
                        )
                        return
                    raise

            @staticmethod
            def _is_client_disconnect(exc: OSError) -> bool:
                return isinstance(exc, (BrokenPipeError, ConnectionResetError, socket.timeout)) or (
                    exc.errno in {errno.EPIPE, errno.ECONNRESET, errno.ECONNABORTED}
                )

        return MissionBackendRequestHandler

    def _read_raw_ipc_request(self, reader) -> dict[str, Any]:
        raw_line = reader.readline(1024 * 1024 + 1)
        if len(raw_line) > 1024 * 1024:
            raise ValueError("Backend IPC request exceeded maximum size")
        if not raw_line:
            raise ValueError("Backend IPC request was empty")
        try:
            decoded = json.loads(raw_line.decode("utf-8").strip())
        except UnicodeDecodeError as exc:
            raise ValueError(f"Backend IPC request was not valid UTF-8: {exc}") from exc
        except json.JSONDecodeError as exc:
            raise ValueError(f"Invalid backend IPC JSON request: {exc}") from exc
        if not isinstance(decoded, dict):
            raise ValueError("Backend IPC request must be a JSON object")
        return decoded

    def _dispatch_ipc_request(self, request: dict[str, Any]) -> dict[str, Any]:
        raw_action = request.get("action")
        if not isinstance(raw_action, str) or not raw_action.strip():
            return self._ipc_error_response(HTTPStatus.BAD_REQUEST, "action is required")
        action = raw_action.strip().upper()

        payload = request.get("payload", {})
        if payload is None:
            payload = {}
        if not isinstance(payload, dict):
            return self._ipc_error_response(HTTPStatus.BAD_REQUEST, "payload must be an object")

        try:
            if action == "GET_STATUS":
                return self._ipc_success_response(self.status_snapshot())
            if action == "LIST_MISSIONS":
                return self._ipc_backend_response(self.list_executable_missions())
            if action == "DOWNLOAD_MISSION":
                mission_id = str(payload.get("mission_id", "")).strip()
                if not mission_id:
                    return self._ipc_error_response(HTTPStatus.BAD_REQUEST, "mission_id is required")
                mission_path = self.mission_file_path(mission_id)
                if not mission_path.exists() or not mission_path.is_file():
                    return self._ipc_error_response(HTTPStatus.NOT_FOUND, "Mission file not found")
                return self._ipc_success_response(
                    {
                        "filename": mission_path.name,
                        "content_type": "application/json; charset=utf-8",
                        "body": mission_path.read_text(encoding="utf-8"),
                    }
                )
            if action in {"EXECUTE_MISSION", "START_SINGLE_MISSION"}:
                mission_id = str(payload.get("mission_id", "")).strip()
                if not mission_id:
                    return self._ipc_error_response(HTTPStatus.BAD_REQUEST, "mission_id is required")
                return self._ipc_backend_response(self.execute_manual_mission(mission_id, payload))
            if action == "START_TELEOP":
                return self._ipc_backend_response(self.start_teleop(payload))
            if action == "STOP_TELEOP":
                return self._ipc_backend_response(self.stop_teleop(payload))
            if action == "SEND_TELEOP_COMMAND":
                return self._ipc_backend_response(self.send_teleop_command(payload))
            if action == "SET_TELEOP_LIGHTS":
                return self._ipc_backend_response(self.set_teleop_lights(payload))
            if action == "UPLOAD_VDA5050_MISSION":
                return self._ipc_backend_response(self.upload_vda5050_mission(payload))
            if action in {"IMPORT_VDA5050_PACKAGE", "APPLY_VDA5050_ORDER"}:
                return self._ipc_backend_response(self.upload_vda5050_mission(payload))
            if action == "VALIDATE_VDA5050_PACKAGE":
                return self._ipc_backend_response(self.validate_vda5050_package(payload))
            if action == "APPLY_VDA5050_ZONESET":
                return self._ipc_backend_response(self.apply_vda5050_zoneset(payload))
            if action == "GET_VDA5050_STATE_SNAPSHOT":
                return self._ipc_backend_response(self.vda5050_state_snapshot())
            if action in {"STOP_MISSION", "STOP"}:
                return self._ipc_backend_response(self.stop_active_mission(payload))
            if action == "REINITIALIZE_SYSTEM":
                return self._ipc_backend_response(self.request_reinitialize(payload))
            if action in {"TRIGGER_SAFETY_STOP", "PAUSE"}:
                return self._ipc_backend_response(self.trigger_safety_stop(payload))
            if action in {"CLEAR_SAFETY_STOP", "RESUME"}:
                return self._ipc_backend_response(self.clear_safety_stop(payload))
            if action == "GET_SCHEDULE":
                return self._ipc_backend_response(self.schedule_snapshot(str(payload.get("week", ""))))
            if action == "SAVE_SCHEDULE_ENTRY":
                return self._ipc_backend_response(self.save_planned_schedule_entry(payload))
            if action == "DELETE_SCHEDULE_ENTRY":
                return self._ipc_backend_response(self.delete_planned_schedule_entry(payload))
            if action == "GET_MAP_DATA":
                return self._ipc_backend_response(self.map_snapshot())
            if action == "LIST_MAPS":
                return self._ipc_backend_response(self.maps_snapshot())
            if action == "BUILD_GAUSSIAN_SPLAT":
                return self._ipc_backend_response(self.build_gaussian_splat(payload))
            if action == "PAUSE_GAUSSIAN_SPLAT":
                return self._ipc_backend_response(self.pause_gaussian_splat(payload))
            if action == "RESUME_GAUSSIAN_SPLAT":
                return self._ipc_backend_response(self.resume_gaussian_splat(payload))
            if action == "GET_GAUSSIAN_SPLAT_STATUS":
                return self._ipc_backend_response(self.gaussian_splat_status())
            if action == "SAVE_MAP":
                return self._ipc_backend_response(self.save_map(payload))
            if action == "DELETE_MAP":
                return self._ipc_backend_response(self.delete_map(payload))
            if action == "GET_RECORD_MAP":
                return self._ipc_backend_response(self.record_map_snapshot())
            if action == "START_RECORD_MAP":
                return self._ipc_backend_response(self.start_record_map(payload))
            if action == "STOP_RECORD_MAP":
                return self._ipc_backend_response(self.stop_record_map(payload))
            if action == "SAVE_RECORDED_MISSION":
                return self._ipc_backend_response(self.create_recorded_mission(payload))
            return self._ipc_error_response(HTTPStatus.NOT_FOUND, f"Unknown backend action: {action}")
        except ValueError as exc:
            return self._ipc_error_response(HTTPStatus.BAD_REQUEST, str(exc))
        except FileNotFoundError as exc:
            return self._ipc_error_response(HTTPStatus.NOT_FOUND, str(exc))
        except Exception as exc:  # noqa: BLE001
            return self._ipc_error_response(HTTPStatus.BAD_GATEWAY, str(exc))

    def _ipc_backend_response(self, payload: dict[str, Any]) -> dict[str, Any]:
        status = HTTPStatus.OK if payload.get("success", True) else HTTPStatus.BAD_GATEWAY
        return self._ipc_response(status, payload)

    def _ipc_success_response(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._ipc_response(HTTPStatus.OK, payload)

    def _ipc_error_response(self, status: HTTPStatus, message: str) -> dict[str, Any]:
        return self._ipc_response(status, {"success": False, "message": message, "error": message})

    def _ipc_response(self, status: HTTPStatus, payload: dict[str, Any]) -> dict[str, Any]:
        response = dict(payload)
        response.setdefault("success", status.value < 400)
        response.setdefault("message", "")
        if not response.get("success", False):
            detail = str(response.get("message") or response.get("error") or status.phrase)
            response["message"] = detail
            response["error"] = detail
        response["status_code"] = int(status)
        return response

    def _handle_fsm_state(self, message: FSMState) -> None:
        with self._state_lock:
            self._latest_fsm_state = {
                "stamp": self._time_to_dict(message.stamp),
                "current_state": message.current_state,
                "current_profile": int(message.current_profile),
            }
            if self._display_fsm_state is None:
                self._display_fsm_state = message.current_state
            if self._display_fsm_profile is None:
                self._display_fsm_profile = int(message.current_profile)

    def _handle_fsm_status(self, message: FSMStatus) -> None:
        should_clear_recent_logs = False
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
            current_state = str(message.current_state).strip().upper()
            transition_status = str(message.transition_status).strip().upper()
            current_profile = int(message.current_profile)
            target_profile = int(message.transitioning_to_profile)
            if transition_status == "TRANSITIONING":
                self._display_transition_active = True
                self._refresh_scheduled_transition_progress_locked()
                if not self._display_transition_progress:
                    self._display_transition_progress = "Transition in progress"
            else:
                if self._display_fsm_state is None:
                    self._display_fsm_state = message.current_state
                if self._display_fsm_profile is None:
                    self._display_fsm_profile = current_profile
                if not self._display_transition_active:
                    self._display_fsm_state = message.current_state
                    self._display_fsm_profile = current_profile
                    self._display_transition_progress = ""
                    self._clear_scheduled_transition_progress_locked()
            if transition_status == "TRANSITIONING" and target_profile == current_profile:
                self._display_transition_progress = "Transition in progress"
            if current_state != "RUNNING":
                self._last_cleared_running_profile = None
            elif (
                transition_status == "STABLE" and
                self._last_cleared_running_profile != current_profile
            ):
                self._last_cleared_running_profile = current_profile
                should_clear_recent_logs = True
        if should_clear_recent_logs:
            self._clear_recent_logs()

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
        completion = _parse_transition_completion_message(message.msg)
        started_command = _parse_started_command_from_log(message.msg)
        progress = _transition_progress_message_from_log(message.msg)
        with self._state_lock:
            if started_command and self._display_transition_active:
                self._handle_started_command_progress_locked(started_command)
            if progress and self._display_transition_active:
                self._display_transition_progress = progress
            if completion is not None:
                completed_state, completed_profile = completion
                self._display_fsm_state = completed_state
                self._display_fsm_profile = completed_profile
                self._display_transition_active = False
                self._display_transition_progress = ""
                self._clear_scheduled_transition_progress_locked()
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

    def _clear_recent_logs(self) -> None:
        with self._state_lock:
            self._recent_logs.clear()

    def list_executable_missions(self) -> dict[str, Any]:
        response = self._call_service(
            self._list_missions_client,
            ListExecutableMissions.Request(),
            timeout_sec=5.0,
            service_name=self._list_missions_service,
        )
        running_profile_defaults = _load_running_profile_default_overrides()
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
                    "profile_default_overrides": {
                        **MISSION_LAYER_OVERRIDE_FALLBACKS,
                        **running_profile_defaults.get(int(running_profile_id), {}),
                    },
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
        request.requester = str(payload.get("requester", self.get_name()))
        request.priority = int(payload.get("priority", 200))
        request.force = bool(payload.get("force", False))
        request.record_rosbag = bool(payload.get("record_rosbag", False))
        layer_overrides = dict(payload.get("layer_overrides", {}) or {})
        if _coerce_bool_value(layer_overrides.get("use_gaussian")) is True:
            layer_overrides["use_amr_sweeper_usb_cameras"] = True
            layer_overrides["use_amr_sweeper_depth_camera"] = True
            layer_overrides["use_amr_sweeper_localization"] = True
            layer_overrides["use_amr_sweeper_mapping"] = True
        request.layer_overrides_json = json.dumps(layer_overrides)
        request.reason = str(payload.get("reason", "manual mission requested from HTTP UI"))

        response = self._call_service(
            self._execute_mission_client,
            request,
            timeout_sec=15.0,
            service_name=self._execute_mission_service,
        )
        if bool(response.success):
            _write_execution_context_preferences(
                response.execution_context_file,
                layer_overrides,
                {
                    "record_rosbag": payload.get("record_rosbag"),
                },
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

    @staticmethod
    def _decode_vda5050_package_payload(payload: dict[str, Any]) -> dict[str, Any]:
        if "mission_json" in payload:
            document = json.loads(str(payload.get("mission_json", "")))
        else:
            document = dict(payload)
        if not isinstance(document, dict):
            raise RuntimeError("VDA5050 package payload must be a JSON object")
        order = document.get("order", document)
        if not isinstance(order, dict):
            raise RuntimeError("VDA5050 package requires an order object")
        if not str(order.get("version", "")).startswith("3."):
            raise RuntimeError("Only VDA5050 major version 3 packages are supported")
        if any(key in order for key in ("missionReference", "missionGeometries", "coveragePathEdgeIds")):
            raise RuntimeError("VDA5050 order contains non-compliant custom mission fields")
        if "map_georeference" not in document:
            raise RuntimeError("VDA5050 package requires map_georeference")
        return document

    def validate_vda5050_package(self, payload: dict[str, Any]) -> dict[str, Any]:
        try:
            document = self._decode_vda5050_package_payload(payload)
            order = document.get("order", document)
            return {
                "success": True,
                "message": "VDA5050 package is structurally acceptable for backend import",
                "order_id": str(order.get("orderId", "")),
                "version": str(order.get("version", "")),
                "has_zone_set": isinstance(document.get("zoneSet"), dict),
            }
        except Exception as exc:
            return {"success": False, "message": str(exc)}

    def apply_vda5050_zoneset(self, payload: dict[str, Any]) -> dict[str, Any]:
        mission_id = str(payload.get("mission_id", "")).strip()
        zone_set = payload.get("zoneSet")
        if not mission_id:
            return {"success": False, "message": "mission_id is required"}
        if not isinstance(zone_set, dict):
            return {"success": False, "message": "zoneSet object is required"}
        mission_file = self.mission_file_path(mission_id)
        if not mission_file.exists():
            return {"success": False, "message": "Mission order.json not found"}
        package = {
            "order": json.loads(mission_file.read_text(encoding="utf-8")),
            "zoneSet": zone_set,
            "map_georeference": json.loads(
                (mission_file.parent / "map_georeference.json").read_text(encoding="utf-8")
            ),
        }
        return self.upload_vda5050_mission(
            {
                "mission_id": mission_id,
                "mission_json": json.dumps(package),
                "overwrite_existing": True,
            }
        )

    def vda5050_state_snapshot(self) -> dict[str, Any]:
        status = self.status_snapshot()
        missions = self.list_executable_missions()
        return {
            "success": True,
            "message": "VDA5050 bridge state snapshot",
            "status": status,
            "missions": missions.get("missions", []),
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

    def start_teleop(self, payload: dict[str, Any]) -> dict[str, Any]:
        request_payload = dict(payload)
        mode = str(request_payload.pop("mode", "teleop")).strip().lower()
        layer_overrides = dict(request_payload.get("layer_overrides", {}))
        layer_overrides["use_joy_node"] = False
        layer_overrides["use_amr_sweeper_usb_cameras"] = False
        layer_overrides["use_gaussian"] = False
        if mode == "record_map":
            request_payload["record_rosbag"] = True
            layer_overrides.update(
                {
                    "use_amr_sweeper_usb_cameras": True,
                    "use_amr_sweeper_depth_camera": True,
                    "use_amr_sweeper_localization": True,
                    "use_amr_sweeper_mapping": True,
                    "use_gaussian": True,
                }
            )
        request_payload["layer_overrides"] = layer_overrides
        request_payload.setdefault(
            "reason",
            "web record map requested from HTTP UI"
            if mode == "record_map"
            else "web teleop requested from HTTP UI",
        )
        request_payload.setdefault("priority", 200)
        request_payload.setdefault("requester", "frontend_teleop")
        return self.execute_manual_mission(
            "RecordMap" if mode == "record_map" else TELEOP_MISSION_ID,
            request_payload,
        )

    def stop_teleop(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._publish_zero_teleop_command()
        request_payload = dict(payload)
        active_execution = self._discover_active_execution() or {}
        active_mission_id = str(active_execution.get("mission_id", ""))
        if active_mission_id in {TELEOP_MISSION_ID, "RecordMap"}:
            request_payload.setdefault("mission_id", active_mission_id)
        else:
            request_payload.setdefault("mission_id", TELEOP_MISSION_ID)
        request_payload.setdefault("reason", "web teleop stop requested from HTTP UI")
        request_payload.setdefault("outcome", "completed")
        request_payload.setdefault("requester", "frontend_teleop")
        request_payload.setdefault("request_idling", True)
        return self.stop_active_mission(request_payload)

    def send_teleop_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        left_x = self._normalized_axis(payload.get("left_x", 0.0))
        left_y = self._normalized_axis(payload.get("left_y", 0.0))
        right_x = self._normalized_axis(payload.get("right_x", 0.0))
        right_y = self._normalized_axis(payload.get("right_y", 0.0))
        control_mode = self._teleop_control_mode(payload.get("control_mode", "one_stick"))
        wheel_scale = self._normalized_speed_scale(payload.get("wheel_scale", TELEOP_DEFAULT_SPEED_SCALE))
        tool_scale = self._normalized_speed_scale(payload.get("tool_scale", TELEOP_DEFAULT_SPEED_SCALE))
        non_zero = any(abs(value) > 0.0 for value in (left_x, left_y, right_x, right_y))

        ready, reason = self._teleop_command_ready()
        if non_zero and not ready:
            self._publish_zero_teleop_command()
            return {
                "success": False,
                "message": f"Teleop command rejected: {reason}",
                "ready": False,
            }

        drive_command = Twist()
        drive_scale = wheel_scale if control_mode == "two_stick" else 1.0
        drive_command.linear.x = left_y * TELEOP_DRIVE_LINEAR_SCALE * drive_scale
        drive_command.angular.z = -left_x * TELEOP_DRIVE_ANGULAR_SCALE * drive_scale
        tool_command = Twist()
        if control_mode == "two_stick":
            tool_command.linear.x = right_y * TELEOP_TOOL_LINEAR_SCALE * tool_scale
            tool_command.angular.z = right_x * TELEOP_TOOL_ANGULAR_SCALE * tool_scale
        self._publish_teleop_mode(control_mode, wheel_scale, tool_scale)
        self._teleop_drive_publisher.publish(drive_command)
        self._teleop_tool_publisher.publish(tool_command)
        return {
            "success": True,
            "message": "Teleop command published",
            "ready": ready,
        }

    def set_teleop_lights(self, payload: dict[str, Any]) -> dict[str, Any]:
        enabled = bool(payload.get("enabled", False))
        priority = self._byte_value(payload.get("priority", 100), "priority")
        brightness = self._byte_value(payload.get("brightness", 255), "brightness")
        requests = [
            ("front_left", 255 if enabled else 0, 255 if enabled else 0, 255 if enabled else 0, brightness if enabled else 0, priority),
            ("front_right", 255 if enabled else 0, 255 if enabled else 0, 255 if enabled else 0, brightness if enabled else 0, priority),
            ("rear_left", 255 if enabled else 0, 0, 0, brightness if enabled else 0, priority),
            ("rear_right", 255 if enabled else 0, 0, 0, brightness if enabled else 0, priority),
        ]
        results = []
        success = True
        for module, red, green, blue, module_brightness, module_priority in requests:
            result = self.set_led(module, red, green, blue, module_brightness, module_priority)
            results.append(result)
            success = success and bool(result.get("success", False))
        if success:
            self._led_lights_enabled = enabled
        return {
            "success": success,
            "message": "Lights updated" if success else "One or more LED modules failed to update",
            "enabled": enabled,
            "modules": results,
        }

    def set_led(
        self,
        module: str,
        red: int,
        green: int,
        blue: int,
        brightness: int,
        priority: int,
    ) -> dict[str, Any]:
        if module not in LED_MODULE_COMMAND_IDS:
            return {"success": False, "module": module, "message": f"Unknown LED module: {module}"}
        try:
            red = self._byte_value(red, "red")
            green = self._byte_value(green, "green")
            blue = self._byte_value(blue, "blue")
            brightness = self._byte_value(brightness, "brightness")
            priority = self._byte_value(priority, "priority")
        except ValueError as exc:
            return {"success": False, "module": module, "message": str(exc)}

        # Temporary firmware workaround: base-ID byte 0 values 0x00..0x06 collide
        # with firmware-update opcodes. RGB565 conversion makes 0..7 equivalent red.
        wire_red = max(red, 7)
        payload = bytes([wire_red, green, blue, brightness, priority])
        can_id = LED_MODULE_COMMAND_IDS[module]
        try:
            self._send_classic_can_frame(can_id, payload)
        except OSError as exc:
            return {
                "success": False,
                "module": module,
                "can_id": f"0x{can_id:03X}",
                "message": f"SocketCAN write failed on {self._led_can_interface}: {exc}",
            }
        return {
            "success": True,
            "module": module,
            "can_id": f"0x{can_id:03X}",
            "wire_payload_hex": payload.hex().upper(),
            "message": "LED command sent",
        }

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

    def build_gaussian_splat(self, payload: dict[str, Any]) -> dict[str, Any]:
        payload = self._gaussian_splat_build_payload(payload)
        request = BuildGaussianSplat.Request()
        request.mission_id = str(payload.get("mission_id", ""))
        request.mission_execution_directory = str(payload.get("mission_execution_directory", ""))
        request.gaussian_manifest_file = str(
            payload.get("gaussian_manifest_file")
            or self._latest_gaussian_capture_manifest_file()
            or ""
        )
        request.force = bool(payload.get("force", False))
        try:
            request.tile_size_meters = float(payload.get("tile_size_meters", 0.0) or 0.0)
        except (TypeError, ValueError):
            request.tile_size_meters = 0.0
        try:
            request.max_iterations_per_tile = int(payload.get("max_iterations_per_tile", 0) or 0)
        except (TypeError, ValueError):
            request.max_iterations_per_tile = 0

        response = self._call_service(
            self._build_gaussian_splat_client,
            request,
            timeout_sec=20.0,
            service_name=self._build_gaussian_splat_service,
        )
        if bool(response.success):
            map_id = str(payload.get("map_id", ""))
            if map_id:
                self._link_map_gaussian_splat_manifest(map_id, response.artifact_manifest_file)
            else:
                self._link_latest_gaussian_splat_manifest(response.artifact_manifest_file)
        return {
            "success": bool(response.success),
            "message": response.message,
            "artifact_manifest_file": response.artifact_manifest_file,
            "tile_count": int(response.tile_count),
            "completed_tile_count": int(response.completed_tile_count),
        }

    def _gaussian_splat_build_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        request_payload = dict(payload)
        map_id = self._sanitize_map_id(str(request_payload.get("map_id", "")))
        if map_id:
            maps_directory = _resolve_path(self._maps_directory)
            map_directory = (maps_directory / map_id).resolve()
            maps_root = maps_directory.resolve()
            if maps_root not in map_directory.parents or not map_directory.exists():
                raise RuntimeError(f"Map '{map_id}' was not found")
            metadata_file = map_directory / "map.json"
            metadata = json.loads(metadata_file.read_text(encoding="utf-8")) if metadata_file.exists() else {}
            manifest = Path(str(metadata.get("gaussian_manifest_file", "")))
            if not manifest.exists():
                fallback = map_directory / "gaussian" / "manifest.json"
                manifest = fallback if fallback.exists() else manifest
            request_payload["map_id"] = map_id
            request_payload["mission_id"] = str(request_payload.get("mission_id") or map_id)
            request_payload["mission_execution_directory"] = str(map_directory)
            request_payload["gaussian_manifest_file"] = str(manifest)
            return request_payload

        if not request_payload.get("gaussian_manifest_file"):
            request_payload["gaussian_manifest_file"] = self._latest_gaussian_capture_manifest_file()
        return request_payload

    def pause_gaussian_splat(self, payload: dict[str, Any]) -> dict[str, Any]:
        request = PauseGaussianSplatBuild.Request()
        request.build_id = str(payload.get("build_id", ""))
        request.mode = str(payload.get("mode", "user_pause") or "user_pause")
        response = self._call_service(
            self._pause_gaussian_splat_build_client,
            request,
            timeout_sec=65.0,
            service_name=self._pause_gaussian_splat_build_service,
        )
        return {"success": bool(response.success), "message": response.message}

    def resume_gaussian_splat(self, payload: dict[str, Any]) -> dict[str, Any]:
        request = ResumeGaussianSplatBuild.Request()
        request.build_id = str(payload.get("build_id", ""))
        try:
            request.additional_iterations_per_tile = int(
                payload.get("additional_iterations_per_tile", 0) or 0
            )
        except (TypeError, ValueError):
            request.additional_iterations_per_tile = 0
        request.auto_stop_enabled = bool(payload.get("auto_stop_enabled", False))
        response = self._call_service(
            self._resume_gaussian_splat_build_client,
            request,
            timeout_sec=20.0,
            service_name=self._resume_gaussian_splat_build_service,
        )
        return {"success": bool(response.success), "message": response.message}

    def gaussian_splat_status(self) -> dict[str, Any]:
        response = self._call_service(
            self._gaussian_splat_status_client,
            Trigger.Request(),
            timeout_sec=5.0,
            service_name=self._gaussian_splat_status_service,
        )
        try:
            status = json.loads(response.message)
        except Exception:
            status = {"message": response.message}
        return {
            "success": bool(response.success),
            "status": status,
        }

    def stop_active_mission(self, payload: dict[str, Any]) -> dict[str, Any]:
        request = EndMission.Request()
        request.mission_id = str(payload.get("mission_id", ""))
        request.reason = str(payload.get("reason", "mission stop requested from HTTP UI"))
        request.outcome = str(payload.get("outcome", "aborted"))
        request.requester = str(payload.get("requester", self.get_name()))
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
        request.requester = str(payload.get("requester", self.get_name()))
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
        if bool(response.accepted):
            self._clear_recent_logs()
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
        remaining_delay = self._safety_clear_delay_remaining_seconds()
        if remaining_delay > 0.0:
            return {
                "success": False,
                "message": f"Safety stop cannot be cleared for another {remaining_delay:.1f} s",
            }
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

    def trigger_safety_stop(self, payload: dict[str, Any]) -> dict[str, Any]:
        message = SafetyStop()
        message.stamp = self.get_clock().now().to_msg()
        message.sender = str(payload.get("sender", self.get_name())).strip() or self.get_name()
        message.reason = str(payload.get("reason", "safety stop requested from HTTP UI")).strip()
        self._safety_stop_publisher.publish(message)
        return {
            "success": True,
            "message": "Safety stop requested",
        }

    def status_snapshot(self) -> dict[str, Any]:
        with self._state_lock:
            self._refresh_scheduled_transition_progress_locked()
            fsm_state = dict(self._latest_fsm_state) if self._latest_fsm_state is not None else None
            fsm_status = dict(self._latest_fsm_status) if self._latest_fsm_status is not None else None
            navsat = dict(self._latest_navsat) if self._latest_navsat is not None else None
            battery = dict(self._latest_battery) if self._latest_battery is not None else None
            safety_status = dict(self._latest_safety_status) if self._latest_safety_status is not None else None
            recent_logs = list(self._recent_logs)
            display_fsm = {
                "current_state": self._display_fsm_state,
                "current_profile": self._display_fsm_profile,
                "transition_active": self._display_transition_active,
                "transition_progress": self._display_transition_progress,
            }

        active_execution = self._discover_active_execution()
        safety_clear_remaining_sec = self._safety_clear_delay_remaining_seconds(safety_status)
        if safety_status is None:
            safety_status = {}
        safety_status["clear_available_in_sec"] = safety_clear_remaining_sec
        safety_status["can_clear"] = safety_clear_remaining_sec <= 0.0 and bool(safety_status.get("latched"))
        robot_now = datetime.now().astimezone()
        return {
            "success": True,
            "site_title": self._site_title,
            "public_base_url": self._public_base_url,
            "robot_clock": {
                "iso": robot_now.isoformat(),
                "local_time": robot_now.strftime("%Y-%m-%d %H:%M:%S"),
                "timezone": robot_now.tzname() or "",
                "utc_offset": robot_now.strftime("%z"),
                "unix_sec": int(robot_now.timestamp()),
            },
            "fsm_state": fsm_state,
            "fsm_status": fsm_status,
            "fsm_display": display_fsm,
            "position": navsat,
            "battery": battery,
            "safety_stop": safety_status,
            "teleop_lights_enabled": self._led_lights_enabled,
            "active_execution": active_execution,
            "recent_logs": recent_logs,
        }

    def _handle_started_command_progress_locked(self, command: str) -> None:
        if "ros2 launch amr_sweeper_layer_1_hardware_bringup " in command:
            self._clear_scheduled_transition_progress_locked()
            self._display_transition_progress = "Starting Hardware"
            return
        if "ros2 launch amr_sweeper_layer_2_controllers_bringup " in command:
            self._clear_scheduled_transition_progress_locked()
            self._display_transition_progress = "Starting Controllers"
            return
        if "ros2 launch amr_sweeper_layer_3_navigation_bringup " in command:
            self._scheduled_transition_progress_steps = _layer3_transition_progress_steps_from_command(command)
            self._scheduled_transition_progress_started_at = time.monotonic()
            self._refresh_scheduled_transition_progress_locked()

    def _refresh_scheduled_transition_progress_locked(self) -> None:
        if (
            not self._display_transition_active or
            not self._scheduled_transition_progress_steps or
            self._scheduled_transition_progress_started_at is None
        ):
            return
        elapsed_sec = max(0.0, time.monotonic() - self._scheduled_transition_progress_started_at)
        latest_label: str | None = None
        for offset_sec, label in self._scheduled_transition_progress_steps:
            if elapsed_sec >= offset_sec:
                latest_label = label
            else:
                break
        if latest_label:
            self._display_transition_progress = latest_label

    def _clear_scheduled_transition_progress_locked(self) -> None:
        self._scheduled_transition_progress_steps = []
        self._scheduled_transition_progress_started_at = None

    def _safety_clear_delay_remaining_seconds(self, safety_status: dict[str, Any] | None = None) -> float:
        if safety_status is None:
            with self._state_lock:
                safety_status = dict(self._latest_safety_status) if self._latest_safety_status is not None else None
        if not safety_status or not bool(safety_status.get("latched")):
            return 0.0

        latest_event_sec = 0.0
        causes = safety_status.get("causes")
        if isinstance(causes, list):
            for cause in causes:
                if not isinstance(cause, dict):
                    continue
                stamp = cause.get("stamp")
                if not isinstance(stamp, dict):
                    continue
                try:
                    sec = float(stamp.get("sec", 0))
                    nanosec = float(stamp.get("nanosec", 0))
                except (TypeError, ValueError):
                    continue
                latest_event_sec = max(latest_event_sec, sec + (nanosec / 1_000_000_000.0))

        if latest_event_sec <= 0.0:
            return 0.0

        now_sec = self.get_clock().now().nanoseconds / 1_000_000_000.0
        remaining = self._safety_clear_min_delay_sec - (now_sec - latest_event_sec)
        return max(0.0, remaining)

    def _fsm_reports_running(self) -> bool:
        with self._state_lock:
            fsm_status = dict(self._latest_fsm_status) if self._latest_fsm_status is not None else None
            fsm_state = dict(self._latest_fsm_state) if self._latest_fsm_state is not None else None

        current_state = ""
        if fsm_status is not None:
            current_state = str(fsm_status.get("current_state", "")).strip().upper()
        if not current_state and fsm_state is not None:
            current_state = str(fsm_state.get("current_state", "")).strip().upper()
        return current_state == "RUNNING"

    @staticmethod
    def _normalized_axis(value: Any) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError):
            number = 0.0
        number = max(-1.0, min(1.0, number))
        return 0.0 if abs(number) < TELEOP_INPUT_DEADZONE else number

    @staticmethod
    def _normalized_speed_scale(value: Any) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError):
            number = TELEOP_DEFAULT_SPEED_SCALE
        return max(0.0, min(1.0, number))

    @staticmethod
    def _teleop_control_mode(value: Any) -> str:
        return "two_stick" if str(value).strip() == "two_stick" else "one_stick"

    @staticmethod
    def _byte_value(value: Any, name: str) -> int:
        try:
            number = int(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be an integer in 0..255") from exc
        if number < 0 or number > 255:
            raise ValueError(f"{name} must be in 0..255")
        return number

    def _publish_zero_teleop_command(self) -> None:
        self._teleop_drive_publisher.publish(Twist())
        self._teleop_tool_publisher.publish(Twist())

    def _publish_teleop_mode(self, control_mode: str, wheel_scale: float, tool_scale: float) -> None:
        mode_message = String()
        mode_message.data = control_mode
        wheel_scale_message = Float32()
        wheel_scale_message.data = float(wheel_scale)
        scale_message = Float32()
        scale_message.data = float(tool_scale)
        self._teleop_control_mode_publisher.publish(mode_message)
        self._teleop_wheel_scale_publisher.publish(wheel_scale_message)
        self._teleop_tool_scale_publisher.publish(scale_message)

    def _teleop_command_ready(self) -> tuple[bool, str]:
        with self._state_lock:
            fsm_status = dict(self._latest_fsm_status) if self._latest_fsm_status is not None else None
            fsm_state = dict(self._latest_fsm_state) if self._latest_fsm_state is not None else None

        current_state = ""
        current_profile = None
        transition_status = ""
        if fsm_status is not None:
            current_state = str(fsm_status.get("current_state", "")).strip().upper()
            current_profile = fsm_status.get("current_profile")
            transition_status = str(fsm_status.get("transition_status", "")).strip().upper()
        if not current_state and fsm_state is not None:
            current_state = str(fsm_state.get("current_state", "")).strip().upper()
            current_profile = fsm_state.get("current_profile")
        try:
            current_profile_id = int(current_profile)
        except (TypeError, ValueError):
            current_profile_id = -1

        if current_state != "RUNNING":
            return False, f"FSM state is {current_state or 'unknown'}"
        if current_profile_id not in {TELEOP_PROFILE_ID, 225}:
            return False, f"FSM profile is {current_profile_id if current_profile_id >= 0 else 'unknown'}"
        if transition_status and transition_status != "STABLE":
            return False, f"FSM transition status is {transition_status}"

        active_execution = self._discover_active_execution() or {}
        if str(active_execution.get("mission_id", "")) not in {TELEOP_MISSION_ID, "RecordMap"}:
            return False, "active mission is not Teleop or RecordMap"
        if active_execution.get("active", True) is False:
            return False, "Teleop/RecordMap mission is not active"
        return True, "ready"

    def _send_classic_can_frame(self, can_id: int, payload: bytes) -> None:
        if len(payload) > 8:
            raise OSError("Classic CAN payload cannot exceed 8 bytes")
        if can_id < 0 or can_id > 0x7FF:
            raise OSError("Classic CAN standard identifier must be 11-bit")
        padded_payload = payload.ljust(8, b"\x00")
        frame = struct.pack("=IB3x8s", can_id, len(payload), padded_payload)
        with self._led_can_lock:
            if self._led_can_socket is None:
                can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                can_socket.bind((self._led_can_interface,))
                self._led_can_socket = can_socket
            try:
                sent = self._led_can_socket.send(frame)
            except OSError:
                self._close_led_can_socket_locked()
                raise
            if sent != len(frame):
                self._close_led_can_socket_locked()
                raise OSError(f"incomplete CAN frame write: {sent}/{len(frame)} bytes")

    def _close_led_can_socket_locked(self) -> None:
        if self._led_can_socket is None:
            return
        try:
            self._led_can_socket.close()
        finally:
            self._led_can_socket = None

    def _discover_active_execution(self) -> dict[str, Any] | None:
        if not self._fsm_reports_running():
            return None

        missions_log_directory = _resolve_path(self._missions_log_directory)
        selected: dict[str, Any] | None = None
        selected_run_started_at = ""
        try:
            candidates = _execution_context_candidates(missions_log_directory)
        except Exception as exc:  # noqa: BLE001
            return {"error": f"Failed to scan execution contexts: {exc}", "path": str(missions_log_directory)}

        for context_path in candidates:
            try:
                context = json.loads(context_path.read_text(encoding="utf-8"))
            except Exception:
                continue
            if not isinstance(context, dict):
                continue
            if context.get("actual_end_utc"):
                continue
            runtime_status = str(context.get("runtime_status", "")).strip().lower()
            if runtime_status in {"completed", "aborted"}:
                continue
            run_started_at = str(context.get("run_started_at", ""))
            if selected is None or run_started_at > selected_run_started_at:
                selected = dict(context)
                selected["execution_context_file"] = str(context_path)
                selected_run_started_at = run_started_at

        return selected

    def _discover_planned_schedule_path(self) -> Path | None:
        missions_from_db_directory = _resolve_path(self._missions_from_db_directory)
        candidates = self._archive_conflicting_planned_schedules(missions_from_db_directory)
        return candidates[0] if candidates else None

    @staticmethod
    def _archive_conflicting_planned_schedules(missions_from_db_directory: Path) -> list[Path]:
        if not missions_from_db_directory.exists() or not missions_from_db_directory.is_dir():
            return []

        candidates = sorted(
            missions_from_db_directory.glob("schedule_*.ics"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        if len(candidates) <= 1:
            return candidates

        archive_directory = missions_from_db_directory / "archive"
        archive_directory.mkdir(parents=True, exist_ok=True)
        for source_path in candidates[1:]:
            archived_path = archive_directory / source_path.name
            suffix = 1
            while archived_path.exists():
                archived_path = archive_directory / f"{source_path.stem}_{suffix}{source_path.suffix}"
                suffix += 1
            source_path.rename(archived_path)
        return [candidates[0]]

    def _discover_actual_schedule_path(self) -> Path | None:
        active_execution = self._discover_active_execution() or {}
        actual_schedule_log_path = active_execution.get("actual_schedule_log_path", "")
        if actual_schedule_log_path:
            path = Path(actual_schedule_log_path)
            if path.exists():
                return path

        actual_schedule_log_directory = _resolve_path(self._actual_schedule_log_directory)
        missions_log_directory = _resolve_path(self._missions_log_directory)
        for candidate in (
            missions_log_directory / "log.ics",
            actual_schedule_log_directory / "log.ics",
            actual_schedule_log_directory / "simulation_schedule.ics",
            missions_log_directory / "actual_schedule.ics",
            actual_schedule_log_directory / "actual_schedule.ics",
            missions_log_directory / "simulation_schedule.ics",
        ):
            if candidate.exists():
                return candidate
        return None

    def _robot_timezone_name(self) -> str:
        planned_path = self._discover_planned_schedule_path()
        if planned_path is not None and planned_path.exists():
            try:
                for line in planned_path.read_text(encoding="utf-8").splitlines():
                    if line.startswith("X-WR-TIMEZONE:"):
                        value = line.split(":", 1)[1].strip()
                        if value:
                            return value
            except Exception:
                pass

        local = datetime.now().astimezone()
        tz_name = local.tzname()
        if tz_name:
            try:
                ZoneInfo(tz_name)
                return tz_name
            except Exception:
                pass
        return "UTC"

    def _robot_timezone(self):
        timezone_name = self._robot_timezone_name()
        try:
            return ZoneInfo(timezone_name)
        except Exception:
            return datetime.now().astimezone().tzinfo or timezone.utc

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
    def _parse_ics_datetime(value: str, target_timezone) -> datetime:
        cleaned = value.strip()
        if cleaned.endswith("Z"):
            parsed = datetime.strptime(cleaned, "%Y%m%dT%H%M%SZ").replace(tzinfo=timezone.utc)
            return parsed.astimezone(target_timezone)
        return datetime.strptime(cleaned, "%Y%m%dT%H%M%S").replace(tzinfo=target_timezone)

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

    def _load_schedule_events(self, schedule_path: Path | None) -> tuple[Path | None, str, list[dict[str, Any]]]:
        if schedule_path is None or not schedule_path.exists():
            return None, self._robot_timezone_name(), []

        lines = self._unfold_ics_lines(schedule_path.read_text(encoding="utf-8"))
        calendar_timezone = self._robot_timezone_name()
        events: list[dict[str, Any]] = []
        current: dict[str, Any] | None = None
        for line in lines:
            if line.startswith("X-WR-TIMEZONE:"):
                calendar_timezone = line.split(":", 1)[1].strip() or calendar_timezone
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
            current[key] = _unescape_ics_text(value)
            if raw_key.startswith("DTSTART"):
                current["DTSTART_RAW"] = raw_key
            if raw_key.startswith("DTEND"):
                current["DTEND_RAW"] = raw_key
        return schedule_path, calendar_timezone, events

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
        target_timezone,
    ) -> list[dict[str, Any]]:
        if "DTSTART" not in event:
            return []

        start = self._parse_ics_datetime(event["DTSTART"], target_timezone)
        end = (
            self._parse_ics_datetime(event["DTEND"], target_timezone)
            if "DTEND" in event
            else start + self._parse_ics_duration(event.get("DURATION", "PT0S"))
        )
        duration = end - start
        rrule = self._parse_rrule(event.get("RRULE", ""))
        occurrences: list[dict[str, Any]] = []
        until = (
            self._parse_ics_datetime(rrule["UNTIL"], target_timezone)
            if rrule.get("UNTIL")
            else None
        )

        def append_occurrence(occurrence_start: datetime) -> None:
            if until is not None and occurrence_start > until:
                return
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
                    "record_rosbag": event.get("X-RECORD-ROSBAG", "").strip().upper() == "TRUE",
                    "gaussian_capture": event.get("X-GAUSSIAN-CAPTURE", "").strip().upper() == "TRUE",
                    "robot_id": event.get("X-ROBOT-ID", ""),
                    "start": occurrence_start.isoformat(),
                    "end": occurrence_end.isoformat(),
                    "start_local": occurrence_start.strftime("%Y-%m-%dT%H:%M:%S"),
                    "end_local": occurrence_end.strftime("%Y-%m-%dT%H:%M:%S"),
                    "date": occurrence_start.strftime("%Y-%m-%d"),
                    "end_date": occurrence_end.strftime("%Y-%m-%d"),
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
            while current < range_end and (until is None or current <= until):
                append_occurrence(current)
                current += timedelta(days=1)
            return occurrences

        if freq == "MINUTELY":
            interval_minutes = max(1, int(rrule.get("INTERVAL", "1")))
            step = timedelta(minutes=interval_minutes)
            current = start
            while current + duration <= range_start:
                current += step
            while current < range_end and (until is None or current <= until):
                append_occurrence(current)
                current += step
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
                        occurrence_start = occurrence_day.replace(
                            hour=start.hour,
                            minute=start.minute,
                            second=start.second,
                        )
                        if until is None or occurrence_start <= until:
                            append_occurrence(occurrence_start)
                    if current_month.month == 12:
                        current_month = datetime(current_month.year + 1, 1, 1)
                    else:
                        current_month = datetime(current_month.year, current_month.month + 1, 1)
            return occurrences

        append_occurrence(start)
        return occurrences

    @staticmethod
    def _monthly_setpos_for_date(date_value: datetime) -> int:
        return ((date_value.day - 1) // 7) + 1

    @staticmethod
    def _weekday_code(date_value: datetime) -> str:
        return ["MO", "TU", "WE", "TH", "FR", "SA", "SU"][date_value.weekday()]

    def _planned_entry_from_event(self, event: dict[str, Any], target_timezone) -> dict[str, Any]:
        start = self._parse_ics_datetime(event["DTSTART"], target_timezone)
        end = (
            self._parse_ics_datetime(event["DTEND"], target_timezone)
            if "DTEND" in event
            else start + self._parse_ics_duration(event.get("DURATION", "PT0S"))
        )
        rrule = self._parse_rrule(event.get("RRULE", ""))
        recurrence_type = "none"
        recurrence_label = "One-off"
        if rrule.get("FREQ") == "DAILY":
            recurrence_type = "daily"
            recurrence_label = "Daily"
        elif rrule.get("FREQ") == "MINUTELY":
            interval_minutes = max(1, int(rrule.get("INTERVAL", "1")))
            recurrence_type = "minutely"
            recurrence_label = f"Every {interval_minutes} min"
        elif rrule.get("FREQ") == "MONTHLY" and rrule.get("BYDAY") and rrule.get("BYSETPOS"):
            recurrence_type = "monthly_nth_weekday"
            recurrence_label = f"Monthly on the {rrule['BYSETPOS']}{rrule['BYDAY']}"

        return {
            "uid": event.get("UID", ""),
            "summary": event.get("SUMMARY", ""),
            "description": event.get("DESCRIPTION", ""),
            "schedule_type": event.get("X-SCHEDULE-TYPE", "WORK") or "WORK",
            "mission_id": event.get("X-MISSION-ID", ""),
            "record_rosbag": event.get("X-RECORD-ROSBAG", "").strip().upper() == "TRUE",
            "gaussian_capture": event.get("X-GAUSSIAN-CAPTURE", "").strip().upper() == "TRUE",
            "robot_id": event.get("X-ROBOT-ID", ""),
            "start_local": start.strftime("%Y-%m-%dT%H:%M"),
            "end_local": end.strftime("%Y-%m-%dT%H:%M"),
            "recurrence_type": recurrence_type,
            "recurrence_interval_minutes": (
                max(1, int(rrule.get("INTERVAL", "1"))) if recurrence_type == "minutely" else 1
            ),
            "recurrence_until_local": (
                self._parse_ics_datetime(rrule["UNTIL"], target_timezone).strftime("%Y-%m-%dT%H:%M")
                if rrule.get("UNTIL")
                else ""
            ),
            "recurrence_label": recurrence_label,
        }

    def list_planned_schedule_entries(self) -> dict[str, Any]:
        schedule_path = self._discover_or_create_planned_schedule_path()
        schedule_path, timezone_name, events = self._load_schedule_events(schedule_path)
        target_timezone = ZoneInfo(timezone_name)
        planned_entries = [
            self._planned_entry_from_event(event, target_timezone)
            for event in events
            if event.get("UID")
        ]
        planned_entries.sort(key=lambda item: (item["start_local"], item["summary"], item["uid"]))
        return {
            "success": True,
            "planned_schedule_path": str(schedule_path) if schedule_path is not None else "",
            "robot_timezone": timezone_name,
            "planned_entries": planned_entries,
        }

    def _discover_or_create_planned_schedule_path(self) -> Path:
        existing = self._discover_planned_schedule_path()
        if existing is not None:
            return existing

        missions_from_db_directory = _resolve_path(self._missions_from_db_directory)
        missions_from_db_directory.mkdir(parents=True, exist_ok=True)
        created_name = datetime.now(timezone.utc).strftime("schedule_%Y%m%dT%H%M%SZ.ics")
        created_path = missions_from_db_directory / created_name
        timezone_name = self._robot_timezone_name()
        created_path.write_text(self._empty_schedule_document(timezone_name), encoding="utf-8")
        return created_path

    def _empty_schedule_document(self, timezone_name: str) -> str:
        return "\n".join(
            [
                "BEGIN:VCALENDAR",
                "VERSION:2.0",
                "PRODID:-//amr_sweeper_interface_server//EditableSchedule 0.1//EN",
                "CALSCALE:GREGORIAN",
                "METHOD:PUBLISH",
                "X-WR-CALNAME:AMR-Sweeper Schedule",
                f"X-WR-TIMEZONE:{timezone_name}",
                "",
                "END:VCALENDAR",
                "",
            ]
        )

    def _read_schedule_document_parts(self, schedule_path: Path) -> tuple[list[str], list[dict[str, Any]]]:
        if not schedule_path.exists():
            schedule_path.write_text(self._empty_schedule_document(self._robot_timezone_name()), encoding="utf-8")

        lines = self._unfold_ics_lines(schedule_path.read_text(encoding="utf-8"))
        preamble: list[str] = []
        raw_events: list[dict[str, Any]] = []
        current: dict[str, Any] | None = None
        in_events = False

        for line in lines:
            if line == "BEGIN:VEVENT":
                current = {}
                in_events = True
                continue
            if line == "END:VEVENT":
                if current is not None:
                    raw_events.append(current)
                current = None
                continue
            if current is not None and ":" in line:
                raw_key, value = line.split(":", 1)
                key = raw_key.split(";", 1)[0]
                current[key] = _unescape_ics_text(value)
                if raw_key.startswith("DTSTART"):
                    current["DTSTART_RAW"] = raw_key
                if raw_key.startswith("DTEND"):
                    current["DTEND_RAW"] = raw_key
                continue
            if line == "END:VCALENDAR":
                break
            if not in_events:
                preamble.append(line)

        if not preamble:
            preamble = self._empty_schedule_document(self._robot_timezone_name()).splitlines()[:-2]
        timezone_name = self._robot_timezone_name()
        for line in preamble:
            if line.startswith("X-WR-TIMEZONE:"):
                timezone_name = line.split(":", 1)[1].strip() or timezone_name
                break
        target_timezone = ZoneInfo(timezone_name)
        events = [
            self._planned_entry_from_event(raw_event, target_timezone)
            for raw_event in raw_events
            if raw_event.get("UID") and raw_event.get("DTSTART")
        ]
        return preamble, events

    def _write_schedule_document(self, schedule_path: Path, preamble: list[str], events: list[dict[str, Any]]) -> None:
        timezone_name = self._robot_timezone_name()
        normalized_preamble = [line for line in preamble if line != "END:VCALENDAR"]
        has_timezone = any(line.startswith("X-WR-TIMEZONE:") for line in normalized_preamble)
        if not has_timezone:
            normalized_preamble.append(f"X-WR-TIMEZONE:{timezone_name}")

        document_lines = list(normalized_preamble)
        if document_lines and document_lines[-1] != "":
            document_lines.append("")
        for event in events:
            document_lines.extend(self._serialize_schedule_event(event, timezone_name))
            document_lines.append("")
        document_lines.append("END:VCALENDAR")
        document_lines.append("")
        schedule_path.write_text("\n".join(document_lines), encoding="utf-8")

    def _serialize_schedule_event(self, event: dict[str, Any], timezone_name: str) -> list[str]:
        target_timezone = ZoneInfo(timezone_name)
        start = self._parse_local_schedule_datetime(str(event.get("start_local", "")), target_timezone)
        end = self._parse_local_schedule_datetime(str(event.get("end_local", "")), target_timezone)
        lines = [
            "BEGIN:VEVENT",
            f"UID:{_escape_ics_text(event['uid'])}",
            f"DTSTAMP:{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}",
            f"SUMMARY:{_escape_ics_text(event.get('summary', 'Scheduled window'))}",
            f"DESCRIPTION:{_escape_ics_text(event.get('description', ''))}",
            f"DTSTART:{start.astimezone(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}",
            f"DTEND:{end.astimezone(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}",
        ]
        recurrence_type = str(event.get("recurrence_type", "none"))
        if recurrence_type == "daily":
            lines.append("RRULE:FREQ=DAILY")
        elif recurrence_type == "minutely":
            interval_minutes = max(1, int(event.get("recurrence_interval_minutes", 1)))
            rrule = f"RRULE:FREQ=MINUTELY;INTERVAL={interval_minutes}"
            recurrence_until_local = str(event.get("recurrence_until_local", "")).strip()
            if recurrence_until_local:
                until = self._parse_local_schedule_datetime(recurrence_until_local, target_timezone)
                rrule += f";UNTIL={until.astimezone(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
            lines.append(rrule)
        elif recurrence_type == "monthly_nth_weekday":
            lines.append(
                f"RRULE:FREQ=MONTHLY;BYDAY={self._weekday_code(start)};BYSETPOS={self._monthly_setpos_for_date(start)}"
            )
        if event.get("robot_id"):
            lines.append(f"X-ROBOT-ID:{_escape_ics_text(event['robot_id'])}")
        if event.get("schedule_type"):
            lines.append(f"X-SCHEDULE-TYPE:{_escape_ics_text(event['schedule_type'])}")
        if event.get("mission_id"):
            lines.append(f"X-MISSION-ID:{_escape_ics_text(event['mission_id'])}")
        lines.append(f"X-RECORD-ROSBAG:{'TRUE' if event.get('record_rosbag') else 'FALSE'}")
        lines.append(f"X-GAUSSIAN-CAPTURE:{'TRUE' if event.get('gaussian_capture') else 'FALSE'}")
        lines.append("END:VEVENT")
        return lines

    def _parse_local_schedule_datetime(self, value: str, target_timezone) -> datetime:
        cleaned = value.strip()
        parsed = datetime.strptime(cleaned, "%Y-%m-%dT%H:%M")
        return parsed.replace(tzinfo=target_timezone)

    def save_planned_schedule_entry(self, payload: dict[str, Any]) -> dict[str, Any]:
        schedule_path = self._discover_or_create_planned_schedule_path()
        preamble, events = self._read_schedule_document_parts(schedule_path)
        entry_uid = str(payload.get("uid", "")).strip() or f"{uuid4()}@amr_sweeper_interface_server"
        entry = {
            "uid": entry_uid,
            "summary": str(payload.get("summary", "")).strip() or "Scheduled window",
            "description": str(payload.get("description", "")).strip(),
            "schedule_type": str(payload.get("schedule_type", "WORK")).strip() or "WORK",
            "mission_id": str(payload.get("mission_id", "")).strip(),
            "robot_id": str(payload.get("robot_id", "")).strip(),
            "start_local": str(payload.get("start_local", "")).strip(),
            "end_local": str(payload.get("end_local", "")).strip(),
            "recurrence_type": str(payload.get("recurrence_type", "none")).strip() or "none",
            "recurrence_interval_minutes": 1,
            "recurrence_until_local": str(payload.get("recurrence_until_local", "")).strip(),
            "record_rosbag": _coerce_bool_value(payload.get("record_rosbag", False)) or False,
            "gaussian_capture": _coerce_bool_value(payload.get("gaussian_capture", False)) or False,
        }
        if not entry["start_local"] or not entry["end_local"]:
            raise RuntimeError("start_local and end_local are required")

        target_timezone = self._robot_timezone()
        start = self._parse_local_schedule_datetime(entry["start_local"], target_timezone)
        end = self._parse_local_schedule_datetime(entry["end_local"], target_timezone)
        if end <= start:
            raise RuntimeError("end_local must be after start_local")
        if entry["recurrence_type"] == "minutely":
            try:
                entry["recurrence_interval_minutes"] = max(
                    1,
                    int(payload.get("recurrence_interval_minutes", 1)),
                )
            except (TypeError, ValueError) as exc:
                raise RuntimeError("recurrence_interval_minutes must be a positive integer") from exc
            duration_minutes = payload.get("continuous_duration_minutes")
            if duration_minutes not in (None, "") and not entry["recurrence_until_local"]:
                try:
                    continuous_duration = max(1, int(duration_minutes))
                except (TypeError, ValueError) as exc:
                    raise RuntimeError("continuous_duration_minutes must be a positive integer") from exc
                entry["recurrence_until_local"] = (start + timedelta(minutes=continuous_duration)).strftime(
                    "%Y-%m-%dT%H:%M"
                )
            if entry["recurrence_until_local"]:
                recurrence_until = self._parse_local_schedule_datetime(
                    entry["recurrence_until_local"],
                    target_timezone,
                )
                if recurrence_until < start:
                    raise RuntimeError("recurrence_until_local must be at or after start_local")

        updated = False
        for index, event in enumerate(events):
            if event.get("UID") == entry_uid:
                events[index] = {"UID": entry_uid, **entry}
                updated = True
                break
        if not updated:
            events.append({"UID": entry_uid, **entry})

        self._write_schedule_document(schedule_path, preamble, events)
        return {"success": True, "message": "Schedule entry saved", "uid": entry_uid}

    def delete_planned_schedule_entry(self, payload: dict[str, Any]) -> dict[str, Any]:
        schedule_path = self._discover_or_create_planned_schedule_path()
        preamble, events = self._read_schedule_document_parts(schedule_path)
        entry_uid = str(payload.get("uid", "")).strip()
        if not entry_uid:
            raise RuntimeError("uid is required")
        filtered_events = [event for event in events if event.get("UID") != entry_uid]
        if len(filtered_events) == len(events):
            raise RuntimeError(f"Schedule entry '{entry_uid}' was not found")
        self._write_schedule_document(schedule_path, preamble, filtered_events)
        return {"success": True, "message": "Schedule entry deleted", "uid": entry_uid}

    def schedule_snapshot(self, week: str) -> dict[str, Any]:
        target_timezone = self._robot_timezone()
        timezone_name = self._robot_timezone_name()
        if week:
            selected_year, selected_week = week.split("-W", 1)
            week_start = datetime.fromisocalendar(int(selected_year), int(selected_week), 1).replace(tzinfo=target_timezone)
        else:
            now = datetime.now(target_timezone)
            iso_year, iso_week, _ = now.isocalendar()
            week_start = datetime.fromisocalendar(iso_year, iso_week, 1).replace(tzinfo=target_timezone)

        week_end = week_start + timedelta(days=7)
        planned_path, planned_timezone_name, planned_events = self._load_schedule_events(self._discover_planned_schedule_path())
        actual_path, _actual_timezone_name, actual_events = self._load_schedule_events(self._discover_actual_schedule_path())
        planned_occurrences: list[dict[str, Any]] = []
        actual_occurrences: list[dict[str, Any]] = []
        for event in planned_events:
            planned_timezone = ZoneInfo(planned_timezone_name)
            for occurrence in self._expand_event_occurrences(event, week_start, week_end, planned_timezone):
                occurrence["source"] = "planned"
                planned_occurrences.append(occurrence)
        for event in actual_events:
            for occurrence in self._expand_event_occurrences(event, week_start, week_end, target_timezone):
                occurrence["source"] = "actual"
                actual_occurrences.append(occurrence)
        planned_occurrences.sort(key=lambda item: item["start"])
        actual_occurrences.sort(key=lambda item: item["start"])

        iso_year, iso_week, _ = week_start.isocalendar()
        planned_entries = self.list_planned_schedule_entries()["planned_entries"]
        robot_now = datetime.now(target_timezone)

        return {
            "success": True,
            "planned_schedule_path": str(planned_path) if planned_path is not None else "",
            "actual_schedule_path": str(actual_path) if actual_path is not None else "",
            "robot_timezone": timezone_name,
            "robot_clock": {
                "iso": robot_now.isoformat(),
                "local_time": robot_now.strftime("%Y-%m-%d %H:%M:%S"),
                "timezone": robot_now.tzname() or "",
                "utc_offset": robot_now.strftime("%z"),
                "unix_sec": int(robot_now.timestamp()),
            },
            "week": f"{iso_year:04d}-W{iso_week:02d}",
            "week_number": int(iso_week),
            "week_label": f"Week {iso_week:02d} · {week_start.strftime('%d %b')} - {(week_end - timedelta(days=1)).strftime('%d %b %Y')}",
            "week_start": week_start.strftime("%Y-%m-%d"),
            "week_end": (week_end - timedelta(days=1)).strftime("%Y-%m-%d"),
            "planned_events": planned_occurrences,
            "planned_entries": planned_entries,
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

        active_execution = self._discover_active_execution() or {}
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
                gaussian_manifest_path = Path(str(latest_metadata.get("gaussian_manifest_file", "")))
                gaussian_splat_manifest_path = Path(str(latest_metadata.get("gaussian_splat_manifest_file", "")))
                if route_path:
                    latest_route_geojson = self._load_geojson_feature_collection(route_path)
                if navsat_path:
                    latest_navsat_geojson = self._load_geojson_feature_collection(navsat_path)
                if gaussian_manifest_path.exists():
                    latest_metadata["gaussian_manifest"] = json.loads(
                        gaussian_manifest_path.read_text(encoding="utf-8")
                    )
                if gaussian_splat_manifest_path.exists():
                    latest_metadata["gaussian_splat_manifest"] = json.loads(
                        gaussian_splat_manifest_path.read_text(encoding="utf-8")
                    )
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

    def _latest_gaussian_capture_manifest_file(self) -> str:
        latest_directory = _resolve_path(self._missions_log_directory) / "latest_recorded_map"
        latest_metadata_file = latest_directory / "latest_recorded_map.json"
        if latest_metadata_file.exists():
            try:
                latest_metadata = json.loads(latest_metadata_file.read_text(encoding="utf-8"))
                manifest = Path(str(latest_metadata.get("gaussian_manifest_file", "")))
                if manifest.exists():
                    return str(manifest)
            except Exception:
                pass

        candidates = sorted(
            _resolve_path(self._missions_log_directory).rglob("gaussian/manifest.json"),
            key=lambda path: path.stat().st_mtime,
        )
        return str(candidates[-1]) if candidates else ""

    def _link_latest_gaussian_splat_manifest(self, artifact_manifest_file: str) -> None:
        manifest = Path(str(artifact_manifest_file))
        if not manifest.exists():
            return
        latest_directory = _resolve_path(self._missions_log_directory) / "latest_recorded_map"
        latest_metadata_file = latest_directory / "latest_recorded_map.json"
        if not latest_metadata_file.exists():
            return
        try:
            latest_metadata = json.loads(latest_metadata_file.read_text(encoding="utf-8"))
            latest_metadata["gaussian_splat_manifest_file"] = str(manifest)
            latest_metadata_file.write_text(json.dumps(latest_metadata, indent=2), encoding="utf-8")
        except Exception:
            return

    def _link_map_gaussian_splat_manifest(self, map_id: str, artifact_manifest_file: str) -> None:
        manifest = Path(str(artifact_manifest_file))
        if not manifest.exists():
            return
        map_directory = _resolve_path(self._maps_directory) / self._sanitize_map_id(map_id)
        metadata_file = map_directory / "map.json"
        if not metadata_file.exists():
            return
        try:
            metadata = json.loads(metadata_file.read_text(encoding="utf-8"))
            metadata["gaussian_splat_manifest_file"] = str(manifest)
            metadata["updated_at"] = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
            metadata_file.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        except Exception:
            return

    @staticmethod
    def _route_geojson_from_vda5050(document: dict[str, Any], mission_id: str) -> dict[str, Any] | None:
        nodes_by_sequence = {
            int(node["sequenceId"]): node
            for node in document.get("nodes", [])
            if "sequenceId" in node and "nodePosition" in node
        }
        edges_by_sequence = {
            int(edge["sequenceId"]): edge
            for edge in document.get("edges", [])
            if "sequenceId" in edge
        }
        if not nodes_by_sequence:
            return None

        coordinates: list[list[float]] = []

        def append_node(sequence_id: int) -> None:
            node_position = nodes_by_sequence[sequence_id].get("nodePosition", {})
            point = [float(node_position.get("x", 0.0)), float(node_position.get("y", 0.0))]
            if not coordinates or coordinates[-1] != point:
                coordinates.append(point)

        append_node(0)
        for sequence_id in range(1, max(nodes_by_sequence.keys()), 2):
            edge = edges_by_sequence.get(sequence_id, {})
            trajectory = edge.get("trajectory", {})
            for control_point in trajectory.get("controlPoints", []) if isinstance(trajectory, dict) else []:
                point = [float(control_point.get("x", 0.0)), float(control_point.get("y", 0.0))]
                if not coordinates or coordinates[-1] != point:
                    coordinates.append(point)
            if sequence_id + 1 in nodes_by_sequence:
                append_node(sequence_id + 1)

        if len(coordinates) < 2:
            return None

        return {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {"name": mission_id, "source": "vda5050_order", "coordinate_frame": "map"},
                    "geometry": {"type": "LineString", "coordinates": coordinates},
                }
            ],
        }

    def map_snapshot(self) -> dict[str, Any]:
        with self._state_lock:
            navsat = dict(self._latest_navsat) if self._latest_navsat is not None else None

        mission_directories = [*_existing_paths([_resolve_path(self._missions_from_db_directory)]), *self._builtin_mission_directories()]
        missions_log_directory = _resolve_path(self._missions_log_directory)
        missions: list[dict[str, Any]] = []
        mission_files: list[Path] = []
        for missions_directory in mission_directories:
            for mission_file in missions_directory.rglob("*.json"):
                if mission_file.name in {"zoneSet.json", "map_georeference.json"}:
                    continue
                mission_files.append(mission_file)
        seen_paths: set[Path] = set()
        for mission_file in sorted(mission_files):
            if mission_file in seen_paths:
                continue
            seen_paths.add(mission_file)
            mission_id = mission_file.parent.name if mission_file.name == "order.json" else mission_file.stem
            document = self._load_geojson_feature_collection(mission_file)
            route_geojson = None
            route_path = mission_file.parent / f"{mission_id}_path.geojson"
            if not route_path.exists():
                route_path = missions_log_directory / mission_id / f"{mission_id}_path_planned.geojson"
            if route_path.exists():
                route_geojson = self._load_geojson_feature_collection(route_path)
            elif document is not None:
                route_geojson = self._route_geojson_from_vda5050(document, mission_id)
            missions.append(
                {
                    "mission_id": mission_id,
                    "mission_file": str(mission_file),
                    "route_geojson": route_geojson,
                    "route_available": route_geojson is not None,
                }
            )

        active_execution = self._discover_active_execution() or {}
        active_route = None
        active_route_path = active_execution.get("mission_route_file", "")
        if active_route_path:
            active_route = self._load_geojson_feature_collection(Path(active_route_path))

        return {
            "success": True,
            "missions": missions,
            "active_execution": active_execution,
            "active_route_geojson": active_route,
            "current_position": navsat,
        }

    def maps_snapshot(self) -> dict[str, Any]:
        maps_directory = _resolve_path(self._maps_directory)
        maps_directory.mkdir(parents=True, exist_ok=True)
        maps: list[dict[str, Any]] = []
        for metadata_file in sorted(maps_directory.glob("*/map.json")):
            try:
                metadata = json.loads(metadata_file.read_text(encoding="utf-8"))
            except Exception as exc:  # noqa: BLE001
                metadata = {
                    "map_id": metadata_file.parent.name,
                    "name": metadata_file.parent.name,
                    "error": str(exc),
                }
            maps.append(self._map_payload_from_metadata(metadata_file.parent, metadata))

        latest_snapshot = self.record_map_snapshot()
        return {
            "success": True,
            "maps": maps,
            "latest_recorded_map": latest_snapshot.get("latest_recorded_map"),
            "latest_route_geojson": latest_snapshot.get("latest_route_geojson"),
            "latest_navsat_geojson": latest_snapshot.get("latest_navsat_geojson"),
            "active_recording": latest_snapshot.get("active_recording", False),
            "active_navsat_geojson": latest_snapshot.get("active_navsat_geojson"),
            "current_position": latest_snapshot.get("current_position"),
            "patterns": latest_snapshot.get("patterns", ["zigzag", "random", "spiral"]),
            "default_pattern": latest_snapshot.get("default_pattern", "zigzag"),
        }

    def save_map(self, payload: dict[str, Any]) -> dict[str, Any]:
        map_id = self._sanitize_map_id(str(payload.get("map_id") or payload.get("name") or ""))
        if not map_id:
            return {"success": False, "message": "map_id or name is required"}
        maps_directory = _resolve_path(self._maps_directory)
        map_directory = maps_directory / map_id
        overwrite = bool(payload.get("overwrite_existing", True))
        if map_directory.exists() and not overwrite:
            return {"success": False, "message": f"Map '{map_id}' already exists"}
        map_directory.mkdir(parents=True, exist_ok=True)

        existing_metadata_file = map_directory / "map.json"
        existing_metadata: dict[str, Any] = {}
        if existing_metadata_file.exists():
            try:
                existing_metadata = json.loads(existing_metadata_file.read_text(encoding="utf-8"))
            except Exception:
                existing_metadata = {}

        source = str(payload.get("source", "latest_recorded_map")).strip()
        metadata = {
            **existing_metadata,
            "map_id": map_id,
            "name": str(payload.get("name") or existing_metadata.get("name") or map_id),
            "description": str(payload.get("description") or existing_metadata.get("description") or ""),
            "updated_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "sweep_pattern": str(payload.get("sweep_pattern") or existing_metadata.get("sweep_pattern") or "zigzag"),
            "start_position": payload.get("start_position", existing_metadata.get("start_position")),
            "end_position": payload.get("end_position", existing_metadata.get("end_position")),
            "layer_visibility": payload.get("layer_visibility", existing_metadata.get("layer_visibility", {})),
        }

        if source == "latest_recorded_map":
            self._copy_latest_recorded_map_into_map_directory(map_directory, metadata)

        metadata_file = map_directory / "map.json"
        metadata_file.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        return {
            "success": True,
            "message": f"Map '{map_id}' saved",
            "map": self._map_payload_from_metadata(map_directory, metadata),
        }

    def delete_map(self, payload: dict[str, Any]) -> dict[str, Any]:
        map_id = self._sanitize_map_id(str(payload.get("map_id", "")))
        if not map_id:
            return {"success": False, "message": "map_id is required"}
        maps_directory = _resolve_path(self._maps_directory)
        map_directory = (maps_directory / map_id).resolve()
        maps_root = maps_directory.resolve()
        if maps_root not in map_directory.parents:
            return {"success": False, "message": "Refusing to delete outside maps directory"}
        if not map_directory.exists():
            return {"success": False, "message": f"Map '{map_id}' was not found"}
        shutil.rmtree(map_directory)
        return {"success": True, "message": f"Map '{map_id}' deleted", "map_id": map_id}

    def _copy_latest_recorded_map_into_map_directory(
        self,
        map_directory: Path,
        metadata: dict[str, Any],
    ) -> None:
        latest_directory = _resolve_path(self._missions_log_directory) / "latest_recorded_map"
        latest_metadata_file = latest_directory / "latest_recorded_map.json"
        if not latest_metadata_file.exists():
            raise RuntimeError("No latest recorded map is available to save")
        latest_metadata = json.loads(latest_metadata_file.read_text(encoding="utf-8"))
        copy_specs = {
            "recorded_work_area_route_file": "boundary.geojson",
            "recorded_work_area_navsat_file": "boundary_navsat.geojson",
            "recorded_work_area_static_costmap_yaml": "static_costmap.yaml",
            "recorded_work_area_static_costmap_image": "static_costmap.pgm",
        }
        for key, filename in copy_specs.items():
            source_path = Path(str(latest_metadata.get(key, "")))
            if source_path.exists() and source_path.is_file():
                destination = map_directory / filename
                shutil.copyfile(source_path, destination)
                metadata[key] = str(destination)
        gaussian_manifest = Path(str(latest_metadata.get("gaussian_manifest_file", "")))
        if gaussian_manifest.exists() and gaussian_manifest.is_file():
            gaussian_directory = map_directory / "gaussian"
            if gaussian_directory.exists():
                shutil.rmtree(gaussian_directory)
            shutil.copytree(gaussian_manifest.parent, gaussian_directory)
            metadata["gaussian_manifest_file"] = str(gaussian_directory / "manifest.json")
        gaussian_splat_manifest = Path(str(latest_metadata.get("gaussian_splat_manifest_file", "")))
        if gaussian_splat_manifest.exists() and gaussian_splat_manifest.is_file():
            gaussian_splat_directory = map_directory / "gaussian_splat"
            if gaussian_splat_directory.exists():
                shutil.rmtree(gaussian_splat_directory)
            shutil.copytree(gaussian_splat_manifest.parent, gaussian_splat_directory)
            metadata["gaussian_splat_manifest_file"] = str(
                gaussian_splat_directory / "gaussian_splat_manifest.json"
            )
        metadata["source_latest_recorded_map_file"] = str(latest_metadata_file)
        metadata["source_mission_id"] = latest_metadata.get("mission_id", "")
        metadata["source_run_started_at"] = latest_metadata.get("run_started_at", "")
        metadata["recorded_obstacle_count"] = latest_metadata.get("recorded_obstacle_count", 0)
        metadata["recorded_obstacle_points"] = latest_metadata.get("recorded_obstacle_points", [])
        metadata["geo_transform"] = latest_metadata.get("geo_transform", {})
        metadata.setdefault("gaussian_manifest_file", latest_metadata.get("gaussian_manifest_file", ""))
        metadata.setdefault(
            "gaussian_splat_manifest_file",
            latest_metadata.get("gaussian_splat_manifest_file", ""),
        )

    def _map_payload_from_metadata(self, map_directory: Path, metadata: dict[str, Any]) -> dict[str, Any]:
        payload = dict(metadata)
        payload.setdefault("map_id", map_directory.name)
        payload.setdefault("name", payload["map_id"])
        payload["directory"] = str(map_directory)
        route_path = Path(str(payload.get("recorded_work_area_route_file", "")))
        navsat_path = Path(str(payload.get("recorded_work_area_navsat_file", "")))
        zone_set_path = map_directory / "zoneSet.json"
        gaussian_manifest_path = Path(str(payload.get("gaussian_manifest_file", "")))
        gaussian_splat_manifest_path = Path(str(payload.get("gaussian_splat_manifest_file", "")))
        if route_path.exists():
            payload["route_geojson"] = self._load_geojson_feature_collection(route_path)
        if navsat_path.exists():
            payload["navsat_geojson"] = self._load_geojson_feature_collection(navsat_path)
        if zone_set_path.exists():
            try:
                payload["zoneSet"] = json.loads(zone_set_path.read_text(encoding="utf-8"))
            except Exception as exc:  # noqa: BLE001
                payload["zoneSet_error"] = str(exc)
        if gaussian_manifest_path.exists():
            try:
                payload["gaussian_manifest"] = json.loads(
                    gaussian_manifest_path.read_text(encoding="utf-8")
                )
            except Exception as exc:  # noqa: BLE001
                payload["gaussian_error"] = str(exc)
        if gaussian_splat_manifest_path.exists():
            try:
                payload["gaussian_splat_manifest"] = json.loads(
                    gaussian_splat_manifest_path.read_text(encoding="utf-8")
                )
            except Exception as exc:  # noqa: BLE001
                payload["gaussian_splat_error"] = str(exc)
        return payload

    @staticmethod
    def _sanitize_map_id(value: str) -> str:
        cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
        return cleaned.strip("._-")

    def mission_file_path(self, mission_id: str) -> Path:
        if not mission_id:
            raise RuntimeError("mission_id is required")

        candidates: list[Path] = []
        for missions_directory in [*_existing_paths([_resolve_path(self._missions_from_db_directory)]), *self._builtin_mission_directories()]:
            candidates.append(missions_directory / mission_id / "order.json")
            candidates.append(missions_directory / f"{mission_id}.json")
            candidates.extend(missions_directory.rglob(f"{mission_id}.json"))

        for candidate in candidates:
            if candidate.exists() and candidate.is_file():
                return candidate

        raise RuntimeError(f"Mission file for '{mission_id}' was not found")

    def _builtin_mission_directories(self) -> list[Path]:
        workspace_root = Path.cwd()
        candidates = [
            workspace_root / "src" / "layer_3_navigation" / "amr_sweeper_navigation" / "missions",
            workspace_root / "install" / "amr_sweeper_navigation" / "share" / "amr_sweeper_navigation" / "missions",
        ]
        existing: list[Path] = []
        seen: set[Path] = set()
        for candidate in candidates:
            if candidate.exists():
                resolved = candidate.resolve()
                if resolved not in seen:
                    seen.add(resolved)
                    existing.append(candidate)
        return existing

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


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = MissionBackendNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    server_thread: threading.Thread | None = None

    try:
        node.start_ipc_server()
        server_thread = threading.Thread(target=node.serve_forever, name="interface_backend_jsonl", daemon=True)
        server_thread.start()
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"Mission backend startup failed: {exc}")
        return 1
    finally:
        try:
            node.stop_ipc_server()
        except RuntimeError:
            pass
        if server_thread is not None:
            server_thread.join(timeout=2.0)
        executor.shutdown()
        try:
            executor.remove_node(node)
        except (RuntimeError, ValueError):
            pass
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError, AttributeError):
            pass
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
