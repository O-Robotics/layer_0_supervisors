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
            "src/missions_log",
        ).value
        self._missions_from_db_directory = self.declare_parameter(
            "missions_from_db_directory",
            "src/missions_from_db",
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
        self._battery_topic = self.declare_parameter(
            "battery_topic", "battery/battery_state").value
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
        request.requester = str(payload.get("requester", self.get_name()))
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

        active_execution = self._discover_active_execution()
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

    def _discover_active_execution(self) -> dict[str, Any] | None:
        missions_log_directory = _resolve_path(self._missions_log_directory)
        selected: dict[str, Any] | None = None
        selected_run_started_at = ""
        try:
            candidates = missions_log_directory.rglob("execution_context.json")
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
        candidates = sorted(
            missions_from_db_directory.glob("schedule_*.ics"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        return candidates[0] if candidates else None

    def _discover_actual_schedule_path(self) -> Path | None:
        active_execution = self._discover_active_execution() or {}
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
