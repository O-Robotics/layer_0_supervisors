#!/usr/bin/env python3

from __future__ import annotations

import json
import errno
import re
import threading
import time
import urllib.parse
import calendar
from datetime import datetime, timedelta, timezone
from collections import deque
from html import escape
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from uuid import uuid4
from zoneinfo import ZoneInfo

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from amr_sweeper_fsm.msg import FSMState, FSMStatus
from amr_sweeper_fsm.srv import RequestState
from amr_sweeper_mission_executor.srv import (
    CreateRecordedMission,
    EndMission,
    ExecuteMission,
    ListExecutableMissions,
    UploadVda5050Mission,
)
from amr_sweeper_safety_msgs.msg import SafetyStop
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Log
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger


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
    "auto_start_mission": True,
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




class MissionBackendNode(Node):
    def __init__(self, node_name: str = "backend_node") -> None:
        super().__init__(node_name)

        self._http_host = self.declare_parameter("http_host", "0.0.0.0").value
        self._http_port = int(self.declare_parameter("http_port", 8080).value)
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
            "missions/simulations",
        ).value
        self._missions_from_db_directory = self.declare_parameter(
            "missions_from_db_directory",
            "missions/database",
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
        safety_stop_qos = QoSProfile(depth=10)
        safety_stop_qos.reliability = ReliabilityPolicy.RELIABLE
        safety_stop_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._safety_stop_publisher = self.create_publisher(
            SafetyStop,
            self._safety_stop_topic,
            safety_stop_qos,
        )

        self._state_lock = threading.Lock()
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

        self._http_server: ThreadingHTTPServer | None = None

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
                payload.get("layer_overrides", {}),
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
                    "record_rosbag": event.get("X-RECORD-ROSBAG", "").strip().upper() == "TRUE",
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
            while current < range_end:
                append_occurrence(current)
                current += timedelta(days=1)
            return occurrences

        if freq == "MINUTELY":
            interval_minutes = max(1, int(rrule.get("INTERVAL", "1")))
            step = timedelta(minutes=interval_minutes)
            current = start
            while current + duration <= range_start:
                current += step
            while current < range_end:
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
            "robot_id": event.get("X-ROBOT-ID", ""),
            "start_local": start.strftime("%Y-%m-%dT%H:%M"),
            "end_local": end.strftime("%Y-%m-%dT%H:%M"),
            "recurrence_type": recurrence_type,
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
        }
        if not entry["start_local"] or not entry["end_local"]:
            raise RuntimeError("start_local and end_local are required")

        target_timezone = self._robot_timezone()
        start = self._parse_local_schedule_datetime(entry["start_local"], target_timezone)
        end = self._parse_local_schedule_datetime(entry["end_local"], target_timezone)
        if end <= start:
            raise RuntimeError("end_local must be after start_local")

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
        with self._state_lock:
            navsat = dict(self._latest_navsat) if self._latest_navsat is not None else None

        mission_directories = [*_existing_paths([_resolve_path(self._missions_from_db_directory)]), *self._builtin_mission_directories()]
        missions_log_directory = _resolve_path(self._missions_log_directory)
        missions: list[dict[str, Any]] = []
        mission_files: list[Path] = []
        for missions_directory in mission_directories:
            mission_files.extend(missions_directory.rglob("*.json"))
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

    def mission_file_path(self, mission_id: str) -> Path:
        if not mission_id:
            raise RuntimeError("mission_id is required")

        candidates: list[Path] = []
        for missions_directory in [*_existing_paths([_resolve_path(self._missions_from_db_directory)]), *self._builtin_mission_directories()]:
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

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"Mission backend startup failed: {exc}")
        return 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
