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

import importlib.machinery
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "gaussian_splat_builder_node"


class _StubNode:

    pass


class _StubExecutor:

    def add_node(self, _node):
        pass

    def spin(self):
        pass

    def shutdown(self):
        pass


def _install_ros_stubs() -> None:
    fsm_msg = types.ModuleType("amr_sweeper_fsm.msg")
    fsm_msg.FSMState = type("FSMState", (), {})
    fsm_msg.FSMStatus = type("FSMStatus", (), {})
    mission_builder_srv = types.ModuleType("amr_sweeper_mission_builder.srv")
    for name in ("BuildGaussianSplat", "PauseGaussianSplatBuild", "ResumeGaussianSplatBuild"):
        setattr(mission_builder_srv, name, type(name, (), {}))
    rclpy_module = types.ModuleType("rclpy")
    rclpy_module.ok = lambda: False
    rclpy_module.init = lambda: None
    rclpy_module.shutdown = lambda: None
    executors_module = types.ModuleType("rclpy.executors")
    executors_module.MultiThreadedExecutor = _StubExecutor
    node_module = types.ModuleType("rclpy.node")
    node_module.Node = _StubNode
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {})
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = type("Trigger", (), {})

    sys.modules.setdefault("amr_sweeper_fsm", types.ModuleType("amr_sweeper_fsm"))
    sys.modules["amr_sweeper_fsm.msg"] = fsm_msg
    sys.modules.setdefault(
        "amr_sweeper_mission_builder",
        types.ModuleType("amr_sweeper_mission_builder"),
    )
    sys.modules["amr_sweeper_mission_builder.srv"] = mission_builder_srv
    sys.modules["rclpy"] = rclpy_module
    sys.modules["rclpy.executors"] = executors_module
    sys.modules["rclpy.node"] = node_module
    sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs"))
    sys.modules["std_msgs.msg"] = std_msgs_msg
    sys.modules.setdefault("std_srvs", types.ModuleType("std_srvs"))
    sys.modules["std_srvs.srv"] = std_srvs_srv


def _load_builder_module():
    _install_ros_stubs()
    loader = importlib.machinery.SourceFileLoader("gaussian_splat_builder_node", str(SCRIPT))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


builder = _load_builder_module()


def _write_capture_manifest(path: Path, capture_count: int = 1) -> None:
    captures = [
        {
            "odometry_pose": {
                "position": {"x": float(index), "y": 0.0, "z": 0.0},
            },
            "cameras": [],
        }
        for index in range(capture_count)
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                "representation": "synchronized_gaussian_capture_dataset",
                "captures": captures,
            }
        ),
        encoding="utf-8",
    )


class GaussianSplatBuilderPathTest(unittest.TestCase):

    def test_validates_explicit_capture_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest = Path(temp_dir) / "run" / "gaussian" / "manifest.json"
            _write_capture_manifest(manifest)

            document = builder._validate_capture_manifest(manifest)

            self.assertEqual(document["representation"], "synchronized_gaussian_capture_dataset")

    def test_rejects_empty_capture_manifest_with_clear_message(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest = Path(temp_dir) / "run" / "gaussian" / "manifest.json"
            manifest.parent.mkdir(parents=True, exist_ok=True)
            manifest.write_text("", encoding="utf-8")

            with self.assertRaisesRegex(RuntimeError, "Gaussian capture manifest is empty"):
                builder._validate_capture_manifest(manifest)

    def test_output_directory_is_next_to_run_gaussian_directory(self) -> None:
        manifest = Path("/tmp/example_run/gaussian/manifest.json")

        output = builder._splat_output_directory_for_capture_manifest(manifest, "gaussian_splat")

        self.assertEqual(output, Path("/tmp/example_run/gaussian_splat"))

    def test_nonstandard_manifest_does_not_escape_to_filesystem_root(self) -> None:
        manifest = Path("/tmp/manifest.json")

        output = builder._splat_output_directory_for_capture_manifest(manifest, "gaussian_splat")

        self.assertEqual(output, Path("/tmp/gaussian_splat"))

    def test_mission_id_search_includes_simulation_root(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_root = Path(temp_dir)
            logs_root = temp_root / "logs"
            simulations_root = temp_root / "simulations"
            maps_root = temp_root / "maps"
            manifest = simulations_root / "demo" / "20260904T120000Z" / "gaussian" / "manifest.json"
            _write_capture_manifest(manifest)

            node = builder.GaussianSplatBuilderNode.__new__(builder.GaussianSplatBuilderNode)
            node._missions_log_directory = logs_root
            node._simulations_directory = simulations_root
            node._maps_directory = maps_root

            request = types.SimpleNamespace(
                gaussian_manifest_file="",
                mission_execution_directory="",
                mission_id="demo",
            )

            self.assertEqual(node._resolve_capture_manifest(request), manifest.resolve())


if __name__ == "__main__":
    unittest.main()
