#!/usr/bin/env python3
#
# Copyright 2026 O-Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from __future__ import annotations

import importlib.machinery
import importlib.util
import sys
from pathlib import Path
import types
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "mqtt_bridge_node.py"


def _load_bridge_module():
    rclpy_module = types.ModuleType("rclpy")
    rclpy_module.init = lambda args=None: None
    rclpy_module.spin = lambda node: None
    rclpy_module.try_shutdown = lambda: None
    node_module = types.ModuleType("rclpy.node")
    node_module.Node = type("Node", (), {})
    sys.modules["rclpy"] = rclpy_module
    sys.modules["rclpy.node"] = node_module

    loader = importlib.machinery.SourceFileLoader("mqtt_bridge_node", str(SCRIPT))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    sys.modules[loader.name] = module
    loader.exec_module(module)
    return module


bridge = _load_bridge_module()


class MqttBridgeNodeTests(unittest.TestCase):

    def setUp(self) -> None:
        self.topics = bridge.Vda5050Topics("vda5050", "3.0.0", "O-Robotics", "amr_sweeper")

    def test_topics_follow_vda5050_v3_pattern(self) -> None:
        self.assertEqual(
            self.topics.order,
            "vda5050/v3/O-Robotics/amr_sweeper/order",
        )
        self.assertIn(
            "vda5050/v3/O-Robotics/amr_sweeper/instantActions",
            self.topics.subscribe_topics,
        )

    def test_order_routes_to_backend_import(self) -> None:
        action, payload = bridge.normalize_backend_request_for_mqtt(
            self.topics.order,
            {"version": "3.0.0", "orderId": "mission-1", "orderUpdateId": 0},
            self.topics,
            overwrite_existing=True,
            auto_execute_orders=True,
        )

        self.assertEqual(action, "APPLY_VDA5050_ORDER")
        self.assertEqual(payload["mission_id"], "mission-1")
        self.assertTrue(payload["overwrite_existing"])
        self.assertIn("\"orderId\":\"mission-1\"", payload["mission_json"])

    def test_instant_action_routes_to_stop(self) -> None:
        action, payload = bridge.normalize_backend_request_for_mqtt(
            self.topics.instant_actions,
            {"actions": [{"actionType": "cancelOrder"}]},
            self.topics,
            overwrite_existing=True,
            auto_execute_orders=False,
        )

        self.assertEqual(action, "STOP_MISSION")
        self.assertEqual(payload["requester"], "vda5050_mqtt_bridge")

    def test_unsupported_instant_action_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            bridge.normalize_backend_request_for_mqtt(
                self.topics.instant_actions,
                {"actions": [{"actionType": "blink"}]},
                self.topics,
                overwrite_existing=True,
                auto_execute_orders=False,
            )

    def test_string_booleans_are_coerced(self) -> None:
        self.assertTrue(bridge.coerce_bool("true"))
        self.assertTrue(bridge.coerce_bool("1"))
        self.assertFalse(bridge.coerce_bool("false"))
        self.assertFalse(bridge.coerce_bool(""))


if __name__ == "__main__":
    unittest.main()
