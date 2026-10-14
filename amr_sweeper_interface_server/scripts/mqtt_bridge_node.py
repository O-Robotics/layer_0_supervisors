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

"""VDA5050 MQTT bridge for the layer-0 operator backend.

The bridge intentionally keeps all robot-side decisions behind the backend IPC
socket owned by ``backend_node.py``. MQTT callbacks only validate/routinely
translate VDA5050 messages, then ask the backend to upload, execute, stop, or
summarize state through the same local control surface used by the HTTP UI.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import socket
import ssl
import time
from typing import Any

import rclpy
from rclpy.node import Node


VDA_ACTION_TO_BACKEND_ACTION = {
    "cancelOrder": "STOP_MISSION",
    "startPause": "PAUSE",
    "stopPause": "RESUME",
}


@dataclass(frozen=True)
class Vda5050Topics:
    interface_name: str
    version: str
    manufacturer: str
    serial_number: str

    @property
    def major_version(self) -> str:
        return self.version.split(".", 1)[0]

    def topic(self, subtopic: str) -> str:
        return (
            f"{self.interface_name}/v{self.major_version}/"
            f"{self.manufacturer}/{self.serial_number}/{subtopic.strip('/')}"
        )

    @property
    def order(self) -> str:
        return self.topic("order")

    @property
    def instant_actions(self) -> str:
        return self.topic("instantActions")

    @property
    def zone_set(self) -> str:
        return self.topic("zoneSet")

    @property
    def responses(self) -> str:
        return self.topic("responses")

    @property
    def state(self) -> str:
        return self.topic("state")

    @property
    def connection(self) -> str:
        return self.topic("connection")

    @property
    def factsheet(self) -> str:
        return self.topic("factsheet")

    @property
    def subscribe_topics(self) -> tuple[str, ...]:
        return (self.order, self.instant_actions, self.zone_set, self.responses)


def read_secret_file(path: str) -> str:
    if not path:
        return ""
    secret_path = Path(path)
    if not secret_path.exists():
        return ""
    return secret_path.read_text(encoding="utf-8").strip()


def coerce_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def vda_timestamp() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def normalize_backend_request_for_mqtt(
    topic: str,
    payload: dict[str, Any],
    topics: Vda5050Topics,
    overwrite_existing: bool,
    auto_execute_orders: bool,
) -> tuple[str, dict[str, Any]]:
    del auto_execute_orders
    if topic == topics.order:
        mission_id = str(payload.get("orderId", "")).strip()
        if not mission_id:
            raise ValueError("VDA5050 order requires orderId")
        package = payload if "order" in payload else {"order": payload}
        backend_payload = {
            "mission_id": mission_id,
            "mission_json": json.dumps(package, separators=(",", ":")),
            "overwrite_existing": overwrite_existing,
            "requester": "vda5050_mqtt_bridge",
            "reason": "VDA5050 order received over MQTT",
        }
        return "APPLY_VDA5050_ORDER", backend_payload

    if topic == topics.zone_set:
        zone_set = payload.get("zoneSet")
        if not isinstance(zone_set, dict):
            zone_set = {}
        mission_id = str(
            payload.get("mission_id")
            or payload.get("missionId")
            or payload.get("orderId")
            or zone_set.get("mapId", "")
        ).strip()
        return "APPLY_VDA5050_ZONESET", {"mission_id": mission_id, "zoneSet": payload}

    if topic == topics.instant_actions:
        actions = payload.get("actions", [])
        if not isinstance(actions, list) or not actions:
            raise ValueError("instantActions requires a non-empty actions array")
        action_type = str(actions[0].get("actionType", "")).strip()
        backend_action = VDA_ACTION_TO_BACKEND_ACTION.get(action_type)
        if backend_action is None:
            raise ValueError(f"Unsupported VDA5050 instant action: {action_type}")
        return backend_action, {
            "requester": "vda5050_mqtt_bridge",
            "reason": f"VDA5050 instantAction {action_type} received over MQTT",
            "force": True,
        }

    if topic == topics.responses:
        return "GET_STATUS", {}

    raise ValueError(f"Unexpected VDA5050 topic: {topic}")


class BackendIpcClient:
    def __init__(self, socket_path: str, timeout_sec: float = 5.0) -> None:
        self.socket_path = socket_path
        self.timeout_sec = timeout_sec

    def request(self, action: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        request = {"action": action, "payload": payload or {}}
        encoded = (json.dumps(request, separators=(",", ":")) + "\n").encode("utf-8")
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(self.timeout_sec)
            connection.connect(self.socket_path)
            connection.sendall(encoded)
            connection.shutdown(socket.SHUT_WR)
            raw_response = self._read_line(connection)
        decoded = json.loads(raw_response.decode("utf-8").strip())
        if not isinstance(decoded, dict):
            raise RuntimeError("Backend IPC response must be a JSON object")
        return decoded

    def _read_line(self, connection: socket.socket) -> bytes:
        chunks = []
        while True:
            chunk = connection.recv(4096)
            if not chunk:
                break
            chunks.append(chunk)
            if b"\n" in chunk:
                break
            if sum(len(part) for part in chunks) > 1024 * 1024:
                raise RuntimeError("Backend IPC response exceeded maximum size")
        raw = b"".join(chunks).split(b"\n", 1)[0]
        if not raw:
            raise RuntimeError("Backend IPC response was empty")
        return raw


class Vda5050MqttBridgeNode(Node):

    def __init__(self) -> None:
        super().__init__("vda5050_mqtt_bridge_node")
        self.declare_parameter("backend_socket_path", "/tmp/amr_sweeper_interface_backend.sock")
        self.declare_parameter("backend_timeout_seconds", 5.0)
        self.declare_parameter("mqtt_host", "")
        self.declare_parameter("mqtt_port", 8883)
        self.declare_parameter("mqtt_client_id", "")
        self.declare_parameter("mqtt_username", "")
        self.declare_parameter("mqtt_password_file", "")
        self.declare_parameter("mqtt_tls", True)
        self.declare_parameter("mqtt_ca_file", "")
        self.declare_parameter("mqtt_cert_file", "")
        self.declare_parameter("mqtt_key_file", "")
        self.declare_parameter("mqtt_keepalive_seconds", 30)
        self.declare_parameter("interface_name", "vda5050")
        self.declare_parameter("vda5050_version", "3.0.0")
        self.declare_parameter("manufacturer", "O-Robotics")
        self.declare_parameter("serial_number", "amr_sweeper")
        self.declare_parameter("publish_period_seconds", 2.0)
        self.declare_parameter("auto_execute_orders", False)
        self.declare_parameter("overwrite_existing_orders", True)

        self.backend = BackendIpcClient(
            str(self.get_parameter("backend_socket_path").value),
            float(self.get_parameter("backend_timeout_seconds").value),
        )
        self.topics = Vda5050Topics(
            str(self.get_parameter("interface_name").value),
            str(self.get_parameter("vda5050_version").value),
            str(self.get_parameter("manufacturer").value),
            str(self.get_parameter("serial_number").value),
        )
        self.auto_execute_orders = coerce_bool(self.get_parameter("auto_execute_orders").value)
        self.overwrite_existing_orders = coerce_bool(
            self.get_parameter("overwrite_existing_orders").value
        )
        self._client = None
        self._header_ids = {"connection": 0, "factsheet": 0, "state": 0}
        self._last_backend_result: dict[str, Any] | None = None
        self._last_order_id = ""
        self._last_order_update_id = 0

        self._start_mqtt()
        period = max(0.2, float(self.get_parameter("publish_period_seconds").value))
        self._timer = self.create_timer(period, self._publish_periodic_messages)
        self.get_logger().info(
            "VDA5050 MQTT bridge connected to %s:%s for %s/%s",
            self.get_parameter("mqtt_host").value,
            self.get_parameter("mqtt_port").value,
            self.topics.manufacturer,
            self.topics.serial_number,
        )

    def _start_mqtt(self) -> None:
        mqtt_host = str(self.get_parameter("mqtt_host").value).strip()
        if not mqtt_host:
            raise RuntimeError("mqtt_host is required when launching VDA5050 MQTT bridge")
        try:
            import paho.mqtt.client as mqtt
        except ImportError as exc:
            raise RuntimeError("python3-paho-mqtt is required for VDA5050 MQTT bridge") from exc

        client_id = str(self.get_parameter("mqtt_client_id").value).strip()
        if not client_id:
            client_id = f"amr-sweeper-vda5050-{self.topics.serial_number}"
        self._client = mqtt.Client(client_id=client_id)
        username = str(self.get_parameter("mqtt_username").value).strip()
        if username:
            password = read_secret_file(
                str(self.get_parameter("mqtt_password_file").value).strip()
            )
            self._client.username_pw_set(username, password or None)
        if coerce_bool(self.get_parameter("mqtt_tls").value):
            self._client.tls_set(
                ca_certs=str(self.get_parameter("mqtt_ca_file").value).strip() or None,
                certfile=str(self.get_parameter("mqtt_cert_file").value).strip() or None,
                keyfile=str(self.get_parameter("mqtt_key_file").value).strip() or None,
                cert_reqs=ssl.CERT_REQUIRED,
            )
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        self._client.will_set(
            self.topics.connection,
            json.dumps(self._connection_message("CONNECTION_BROKEN"), separators=(",", ":")),
            qos=1,
            retain=True,
        )
        self._client.connect(
            mqtt_host,
            int(self.get_parameter("mqtt_port").value),
            int(self.get_parameter("mqtt_keepalive_seconds").value),
        )
        self._client.loop_start()

    def _on_connect(self, client, _userdata, _flags, rc) -> None:
        self.get_logger().info("mqtt_connected rc=%s", rc)
        for topic in self.topics.subscribe_topics:
            client.subscribe(topic, qos=1)
        self._publish_connection("ONLINE")
        self._publish_factsheet()

    def _on_disconnect(self, _client, _userdata, rc) -> None:
        self.get_logger().warning("mqtt_disconnected rc=%s", rc)

    def _on_message(self, _client, _userdata, msg) -> None:
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("VDA5050 MQTT payload must be a JSON object")
            action, backend_payload = normalize_backend_request_for_mqtt(
                msg.topic,
                payload,
                self.topics,
                self.overwrite_existing_orders,
                self.auto_execute_orders,
            )
            if msg.topic == self.topics.order:
                self._last_order_id = str(payload.get("orderId", "")).strip()
                self._last_order_update_id = int(payload.get("orderUpdateId", 0))
            result = self.backend.request(action, backend_payload)
            if (
                action == "APPLY_VDA5050_ORDER"
                and self.auto_execute_orders
                and bool(result.get("success", False))
            ):
                execute_payload = {
                    "mission_id": backend_payload["mission_id"],
                    "requester": "vda5050_mqtt_bridge",
                    "reason": "VDA5050 order imported and auto-executed from MQTT",
                    "priority": 200,
                    "force": False,
                }
                result = self.backend.request("EXECUTE_MISSION", execute_payload)
            self._last_backend_result = result
            self.get_logger().info(
                "vda5050_mqtt_action topic=%s action=%s success=%s message=%s",
                msg.topic,
                action,
                result.get("success"),
                result.get("message", ""),
            )
        except Exception as exc:  # noqa: BLE001
            self._last_backend_result = {"success": False, "message": str(exc)}
            self.get_logger().warning(
                "vda5050_mqtt_message_failed topic=%s error=%s",
                msg.topic,
                exc,
            )
        self._publish_state()

    def _publish_periodic_messages(self) -> None:
        self._publish_connection("ONLINE")
        self._publish_state()

    def _publish_connection(self, connection_state: str) -> None:
        self._publish_json(
            self.topics.connection,
            self._connection_message(connection_state),
            retain=True,
        )

    def _publish_factsheet(self) -> None:
        self._publish_json(
            self.topics.factsheet,
            self._factsheet_message(),
            retain=True,
        )

    def _publish_state(self) -> None:
        try:
            snapshot = self.backend.request("GET_VDA5050_STATE_SNAPSHOT", {})
        except Exception as exc:  # noqa: BLE001
            snapshot = {"success": False, "message": str(exc)}
        self._publish_json(self.topics.state, self._state_message(snapshot), retain=False)

    def _publish_json(self, topic: str, payload: dict[str, Any], retain: bool = False) -> None:
        if self._client is None:
            return
        self._client.publish(
            topic,
            json.dumps(payload, separators=(",", ":")),
            qos=1,
            retain=retain,
        )

    def _header(self, topic_key: str) -> dict[str, Any]:
        self._header_ids[topic_key] += 1
        return {
            "headerId": self._header_ids[topic_key],
            "timestamp": vda_timestamp(),
            "version": self.topics.version,
            "manufacturer": self.topics.manufacturer,
            "serialNumber": self.topics.serial_number,
        }

    def _connection_message(self, connection_state: str) -> dict[str, Any]:
        return {**self._header("connection"), "connectionState": connection_state}

    def _factsheet_message(self) -> dict[str, Any]:
        return {
            **self._header("factsheet"),
            "typeSpecification": {
                "seriesName": "AMR Sweeper",
                "seriesDescription": "AMR Sweeper layer-0 VDA5050 bridge",
                "mobileRobotKinematics": "DIFFERENTIAL",
                "mobileRobotClass": "CARRIER",
                "maximumLoadMass": 0.0,
                "localizationTypes": ["GNSS", "ODOM"],
                "navigationTypes": ["FREELY_NAVIGATING"],
            },
            "physicalParameters": {
                "minimumSpeed": 0.0,
                "maximumSpeed": 1.0,
                "maximumAcceleration": 0.5,
                "maximumDeceleration": 0.5,
                "minimumHeight": 0.0,
                "maximumHeight": 1.5,
                "width": 0.8,
                "length": 1.2,
            },
            "protocolLimits": {
                "maximumStringLengths": {},
                "maximumArrayLengths": {},
                "timing": {
                    "minimumOrderInterval": 0.0,
                    "minimumStateInterval": 0.2,
                },
            },
            "protocolFeatures": {
                "optionalParameters": [],
                "mobileRobotActions": [
                    {
                        "actionType": "startPause",
                        "actionDescription": "Trigger safety pause",
                        "actionScopes": ["INSTANT"],
                        "pauseAllowed": False,
                        "cancelAllowed": False,
                    },
                    {
                        "actionType": "stopPause",
                        "actionDescription": "Clear safety pause",
                        "actionScopes": ["INSTANT"],
                        "pauseAllowed": False,
                        "cancelAllowed": False,
                    },
                    {
                        "actionType": "cancelOrder",
                        "actionDescription": "Stop active mission",
                        "actionScopes": ["INSTANT"],
                        "pauseAllowed": False,
                        "cancelAllowed": False,
                    },
                ],
            },
            "mobileRobotGeometry": {},
            "loadSpecification": {},
        }

    def _state_message(self, snapshot: dict[str, Any]) -> dict[str, Any]:
        status = snapshot.get("status", {}) if isinstance(snapshot.get("status"), dict) else {}
        fsm_status = (
            status.get("fsm_status", {})
            if isinstance(status.get("fsm_status"), dict) else {}
        )
        battery = status.get("battery", {}) if isinstance(status.get("battery"), dict) else {}
        safety_stop = (
            status.get("safety_stop", {})
            if isinstance(status.get("safety_stop"), dict) else {}
        )
        active_execution = (
            status.get("active_execution", {})
            if isinstance(status.get("active_execution"), dict) else {}
        )
        backend_ok = bool(snapshot.get("success", False))
        last_ok = True if self._last_backend_result is None else bool(
            self._last_backend_result.get("success", False)
        )
        errors = []
        if not backend_ok:
            errors.append(_vda_error("backend_snapshot_failed", snapshot.get("message", "")))
        if not last_ok and self._last_backend_result is not None:
            errors.append(
                _vda_error(
                    "backend_action_failed",
                    self._last_backend_result.get("message", ""),
                )
            )

        order_id = str(active_execution.get("mission_id") or self._last_order_id)
        driving = str(fsm_status.get("current_state", "")).upper() == "RUNNING"
        paused = bool(safety_stop.get("latched", False))
        battery_percentage = battery.get("percentage")
        if battery_percentage is None:
            battery_charge = 0.0
        else:
            battery_charge = max(0.0, min(100.0, float(battery_percentage) * 100.0))
        return {
            **self._header("state"),
            "orderId": order_id,
            "orderUpdateId": self._last_order_update_id,
            "lastNodeId": "",
            "lastNodeSequenceId": 0,
            "nodeStates": [],
            "edgeStates": [],
            "agvPosition": {"positionInitialized": False},
            "velocity": {},
            "loads": [],
            "driving": driving,
            "paused": paused,
            "newBaseRequest": False,
            "distanceSinceLastNode": 0.0,
            "operatingMode": "AUTOMATIC",
            "actionStates": [],
            "instantActionStates": [],
            "powerSupply": {
                "stateOfCharge": battery_charge,
                "charging": False,
            },
            "errors": errors,
            "information": [
                {
                    "infoType": "layer0State",
                    "infoLevel": "INFO",
                    "infoDescription": str(fsm_status.get("last_message", "")),
                    "infoReferences": [],
                }
            ],
            "safetyState": {
                "activeEmergencyStop": "REMOTE" if paused else "NONE",
                "fieldViolation": False,
            },
        }

    def destroy_node(self):
        if self._client is not None:
            try:
                self._publish_connection("OFFLINE")
                self._client.loop_stop()
                self._client.disconnect()
            finally:
                self._client = None
        return super().destroy_node()


def _vda_error(error_type: str, description: Any) -> dict[str, Any]:
    return {
        "errorType": error_type,
        "errorLevel": "WARNING",
        "errorDescription": str(description or error_type),
        "errorReferences": [],
    }


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = Vda5050MqttBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError, AttributeError):
            pass
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
