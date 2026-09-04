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


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "frontend_http_node.py"
BACKEND_SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "backend_node.py"


class _StubNode:

    pass


class _StubExecutor:

    def add_node(self, _node):
        pass

    def spin(self):
        pass

    def shutdown(self):
        pass


def _stub_message_package(package: str, names: tuple[str, ...]) -> None:
    parent = types.ModuleType(package)
    msg = types.ModuleType(f"{package}.msg")
    for name in names:
        setattr(msg, name, type(name, (), {}))
    sys.modules[package] = parent
    sys.modules[f"{package}.msg"] = msg


def _stub_service_package(package: str, names: tuple[str, ...]) -> None:
    parent = types.ModuleType(package)
    srv = types.ModuleType(f"{package}.srv")
    for name in names:
        setattr(srv, name, type(name, (), {}))
    sys.modules[package] = parent
    sys.modules[f"{package}.srv"] = srv


def _install_backend_stubs() -> None:
    ament_packages = types.ModuleType("ament_index_python.packages")
    ament_packages.get_package_share_directory = lambda _name: ""
    ament_packages.PackageNotFoundError = RuntimeError
    sys.modules.setdefault("ament_index_python", types.ModuleType("ament_index_python"))
    sys.modules["ament_index_python.packages"] = ament_packages

    _stub_message_package("amr_sweeper_fsm", ("FSMState", "FSMStatus"))
    _stub_service_package("amr_sweeper_fsm", ("RequestState",))
    _stub_service_package(
        "amr_sweeper_mission_builder",
        ("BuildGaussianSplat", "PauseGaussianSplatBuild", "ResumeGaussianSplatBuild"),
    )
    _stub_service_package(
        "amr_sweeper_mission_executor",
        (
            "CreateRecordedMission",
            "EndMission",
            "ExecuteMission",
            "ListExecutableMissions",
            "UploadVda5050Mission",
        ),
    )
    _stub_message_package("amr_sweeper_safety_msgs", ("SafetyStop",))
    _stub_message_package("geometry_msgs", ("Twist",))
    _stub_message_package("rcl_interfaces", ("Log",))
    _stub_message_package("sensor_msgs", ("BatteryState", "NavSatFix"))
    _stub_message_package("std_msgs", ("Float32", "String"))
    _stub_service_package("std_srvs", ("Trigger",))

    rclpy_module = types.ModuleType("rclpy")
    rclpy_module.ok = lambda: False
    rclpy_module.init = lambda: None
    rclpy_module.shutdown = lambda: None
    executors_module = types.ModuleType("rclpy.executors")
    executors_module.MultiThreadedExecutor = _StubExecutor
    node_module = types.ModuleType("rclpy.node")
    node_module.Node = _StubNode
    qos_module = types.ModuleType("rclpy.qos")
    qos_module.DurabilityPolicy = type("DurabilityPolicy", (), {})
    qos_module.QoSProfile = type("QoSProfile", (), {})
    qos_module.ReliabilityPolicy = type("ReliabilityPolicy", (), {})
    sys.modules["rclpy"] = rclpy_module
    sys.modules["rclpy.executors"] = executors_module
    sys.modules["rclpy.node"] = node_module
    sys.modules["rclpy.qos"] = qos_module


def _load_frontend_module():
    loader = importlib.machinery.SourceFileLoader("frontend_http_node", str(SCRIPT))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


frontend = _load_frontend_module()


def _load_backend_module():
    _install_backend_stubs()
    loader = importlib.machinery.SourceFileLoader("backend_node", str(BACKEND_SCRIPT))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


backend = _load_backend_module()


class GaussianSplatArtifactRootsTest(unittest.TestCase):

    def test_simulation_gaussian_splat_artifacts_are_allowed(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            workspace = Path(temp_dir)
            roots = frontend.gaussian_splat_artifact_allowed_roots(workspace)
            simulation_artifact = (
                workspace
                / "missions"
                / "simulations"
                / "demo"
                / "20260904T120000Z"
                / "gaussian_splat"
                / "gaussian_splat_manifest.json"
            ).resolve()

            self.assertTrue(any(root == simulation_artifact or root in simulation_artifact.parents for root in roots))

    def test_unrelated_paths_are_not_allowed(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            workspace = Path(temp_dir) / "workspace"
            outside = Path("/var/lib/amr_sweeper/gaussian_splat_manifest.json").resolve()
            roots = frontend.gaussian_splat_artifact_allowed_roots(workspace)

            self.assertFalse(any(root == outside or root in outside.parents for root in roots))


class SavedMapGaussianFlowTest(unittest.TestCase):

    def test_saving_named_map_rewrites_gaussian_manifest_to_saved_map_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            workspace = Path(temp_dir)
            latest = workspace / "missions" / "logs" / "latest_recorded_map"
            latest_gaussian = latest / "gaussian"
            latest_gaussian.mkdir(parents=True)
            (latest_gaussian / "frames").mkdir()
            source_manifest = latest_gaussian / "manifest.json"
            source_manifest.write_text(
                json.dumps(
                    {
                        "representation": "synchronized_gaussian_capture_dataset",
                        "output_directory": str(latest_gaussian),
                        "gaussian_manifest_file": str(source_manifest),
                        "captures": [],
                    }
                ),
                encoding="utf-8",
            )
            latest_metadata_file = latest / "latest_recorded_map.json"
            latest_metadata_file.write_text(
                json.dumps({"gaussian_manifest_file": str(source_manifest)}),
                encoding="utf-8",
            )
            map_directory = workspace / "missions" / "maps" / "My_Map"
            metadata = {}
            node = backend.MissionBackendNode.__new__(backend.MissionBackendNode)
            node._missions_log_directory = str(workspace / "missions" / "logs")

            node._copy_latest_recorded_map_into_map_directory(map_directory, metadata)

            saved_manifest = map_directory / "gaussian" / "manifest.json"
            document = json.loads(saved_manifest.read_text(encoding="utf-8"))
            self.assertEqual(document["output_directory"], str(map_directory / "gaussian"))
            self.assertEqual(document["gaussian_manifest_file"], str(saved_manifest))
            self.assertEqual(metadata["gaussian_manifest_file"], str(saved_manifest))

    def test_saved_map_build_uses_saved_manifest_not_payload_or_latest_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            workspace = Path(temp_dir)
            maps_root = workspace / "missions" / "maps"
            map_directory = maps_root / "Named_Map"
            saved_manifest = map_directory / "gaussian" / "manifest.json"
            saved_manifest.parent.mkdir(parents=True)
            saved_manifest.write_text(
                json.dumps(
                    {
                        "representation": "synchronized_gaussian_capture_dataset",
                        "captures": [],
                    }
                ),
                encoding="utf-8",
            )
            (map_directory / "map.json").write_text(
                json.dumps({"map_id": "Named_Map", "name": "Named Map"}),
                encoding="utf-8",
            )
            node = backend.MissionBackendNode.__new__(backend.MissionBackendNode)
            node._maps_directory = str(maps_root)
            node._missions_log_directory = str(workspace / "missions" / "logs")
            node._simulations_directory = str(workspace / "missions" / "simulations")

            payload = node._gaussian_splat_build_payload(
                {
                    "map_id": "Named Map",
                    "gaussian_manifest_file": "/tmp/wrong/manifest.json",
                }
            )

            self.assertEqual(payload["map_id"], "Named_Map")
            self.assertEqual(payload["mission_execution_directory"], str(map_directory))
            self.assertEqual(payload["gaussian_manifest_file"], str(saved_manifest))

    def test_save_as_from_selected_saved_map_copies_selected_source(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            workspace = Path(temp_dir)
            maps_root = workspace / "missions" / "maps"
            source_directory = maps_root / "Test4"
            source_gaussian = source_directory / "gaussian"
            source_gaussian.mkdir(parents=True)
            source_manifest = source_gaussian / "manifest.json"
            source_manifest.write_text(
                json.dumps(
                    {
                        "representation": "synchronized_gaussian_capture_dataset",
                        "output_directory": "/tmp/old/source/gaussian",
                        "gaussian_manifest_file": "/tmp/old/source/gaussian/manifest.json",
                        "captures": [],
                    }
                ),
                encoding="utf-8",
            )
            (source_directory / "map.json").write_text(
                json.dumps(
                    {
                        "map_id": "Test4",
                        "name": "Test4",
                        "gaussian_manifest_file": str(source_manifest),
                    }
                ),
                encoding="utf-8",
            )
            node = backend.MissionBackendNode.__new__(backend.MissionBackendNode)
            node._maps_directory = str(maps_root)
            node._missions_log_directory = str(workspace / "missions" / "logs")
            node._simulations_directory = str(workspace / "missions" / "simulations")

            response = node.save_map(
                {
                    "map_id": "Test4 Copy",
                    "name": "Test4 Copy",
                    "source": "saved_map",
                    "source_map_id": "Test4",
                    "overwrite_existing": True,
                }
            )

            copied_directory = maps_root / "Test4_Copy"
            copied_manifest = copied_directory / "gaussian" / "manifest.json"
            document = json.loads(copied_manifest.read_text(encoding="utf-8"))
            self.assertTrue(response["success"])
            self.assertEqual(response["map"]["map_id"], "Test4_Copy")
            self.assertEqual(document["output_directory"], str(copied_directory / "gaussian"))
            self.assertEqual(document["gaussian_manifest_file"], str(copied_manifest))

    def test_save_latest_map_with_empty_gaussian_manifest_still_saves_map(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            workspace = Path(temp_dir)
            latest = workspace / "missions" / "logs" / "latest_recorded_map"
            latest_gaussian = latest / "gaussian"
            latest_gaussian.mkdir(parents=True)
            source_manifest = latest_gaussian / "manifest.json"
            source_manifest.write_text("", encoding="utf-8")
            route_file = latest / "latest_recorded_map_route.geojson"
            navsat_file = latest / "latest_recorded_map_navsat.geojson"
            costmap_yaml = latest / "latest_recorded_map_static_costmap.yaml"
            costmap_image = latest / "latest_recorded_map_static_costmap.pgm"
            route_file.write_text('{"type":"FeatureCollection","features":[]}', encoding="utf-8")
            navsat_file.write_text('{"type":"FeatureCollection","features":[]}', encoding="utf-8")
            costmap_yaml.write_text("image: latest_recorded_map_static_costmap.pgm\nresolution: 0.05\n", encoding="utf-8")
            costmap_image.write_text("P2\n1 1\n255\n0\n", encoding="utf-8")
            latest_metadata_file = latest / "latest_recorded_map.json"
            latest_metadata_file.write_text(
                json.dumps(
                    {
                        "recorded_work_area_route_file": str(route_file),
                        "recorded_work_area_navsat_file": str(navsat_file),
                        "recorded_work_area_static_costmap_yaml": str(costmap_yaml),
                        "recorded_work_area_static_costmap_image": str(costmap_image),
                        "gaussian_manifest_file": str(source_manifest),
                    }
                ),
                encoding="utf-8",
            )
            node = backend.MissionBackendNode.__new__(backend.MissionBackendNode)
            node._maps_directory = str(workspace / "missions" / "maps")
            node._missions_log_directory = str(workspace / "missions" / "logs")
            node._simulations_directory = str(workspace / "missions" / "simulations")

            response = node.save_map(
                {
                    "map_id": "Empty Gaussian",
                    "name": "Empty Gaussian",
                    "source": "latest_recorded_map",
                    "overwrite_existing": True,
                }
            )

            metadata_file = workspace / "missions" / "maps" / "Empty_Gaussian" / "map.json"
            metadata = json.loads(metadata_file.read_text(encoding="utf-8"))
            self.assertTrue(response["success"])
            self.assertEqual(response["map"]["map_id"], "Empty_Gaussian")
            self.assertTrue((metadata_file.parent / "Empty_Gaussian_boundary.geojson").is_file())
            self.assertTrue((metadata_file.parent / "Empty_Gaussian_boundary_navsat.geojson").is_file())
            self.assertTrue((metadata_file.parent / "Empty_Gaussian_static_costmap.yaml").is_file())
            self.assertTrue((metadata_file.parent / "Empty_Gaussian_static_costmap.pgm").is_file())
            self.assertEqual(
                metadata["recorded_work_area_route_file"],
                str(metadata_file.parent / "Empty_Gaussian_boundary.geojson"),
            )
            self.assertIn(
                "image: Empty_Gaussian_static_costmap.pgm",
                (metadata_file.parent / "Empty_Gaussian_static_costmap.yaml").read_text(encoding="utf-8"),
            )
            self.assertIn("Could not update saved Gaussian capture manifest", metadata["gaussian_error"])
            self.assertNotIn("gaussian_manifest_file", metadata)

    def test_save_as_from_saved_map_with_empty_gaussian_manifest_still_saves_map(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            workspace = Path(temp_dir)
            maps_root = workspace / "missions" / "maps"
            source_directory = maps_root / "Test4"
            source_gaussian = source_directory / "gaussian"
            source_gaussian.mkdir(parents=True)
            source_manifest = source_gaussian / "manifest.json"
            source_manifest.write_text("", encoding="utf-8")
            source_route = source_directory / "Test4_boundary.geojson"
            source_navsat = source_directory / "Test4_boundary_navsat.geojson"
            source_costmap_yaml = source_directory / "Test4_static_costmap.yaml"
            source_costmap_image = source_directory / "Test4_static_costmap.pgm"
            source_route.write_text('{"type":"FeatureCollection","features":[]}', encoding="utf-8")
            source_navsat.write_text('{"type":"FeatureCollection","features":[]}', encoding="utf-8")
            source_costmap_yaml.write_text("image: Test4_static_costmap.pgm\nresolution: 0.05\n", encoding="utf-8")
            source_costmap_image.write_text("P2\n1 1\n255\n0\n", encoding="utf-8")
            (source_directory / "map.json").write_text(
                json.dumps(
                    {
                        "map_id": "Test4",
                        "name": "Test4",
                        "recorded_work_area_route_file": str(source_route),
                        "recorded_work_area_navsat_file": str(source_navsat),
                        "recorded_work_area_static_costmap_yaml": str(source_costmap_yaml),
                        "recorded_work_area_static_costmap_image": str(source_costmap_image),
                        "gaussian_manifest_file": str(source_manifest),
                    }
                ),
                encoding="utf-8",
            )
            node = backend.MissionBackendNode.__new__(backend.MissionBackendNode)
            node._maps_directory = str(maps_root)
            node._missions_log_directory = str(workspace / "missions" / "logs")
            node._simulations_directory = str(workspace / "missions" / "simulations")

            response = node.save_map(
                {
                    "map_id": "Test4 Copy",
                    "name": "Test4 Copy",
                    "source": "saved_map",
                    "source_map_id": "Test4",
                    "overwrite_existing": True,
                }
            )

            metadata_file = maps_root / "Test4_Copy" / "map.json"
            metadata = json.loads(metadata_file.read_text(encoding="utf-8"))
            self.assertTrue(response["success"])
            self.assertEqual(response["map"]["map_id"], "Test4_Copy")
            self.assertTrue((metadata_file.parent / "Test4_Copy_boundary.geojson").is_file())
            self.assertTrue((metadata_file.parent / "Test4_Copy_boundary_navsat.geojson").is_file())
            self.assertTrue((metadata_file.parent / "Test4_Copy_static_costmap.yaml").is_file())
            self.assertTrue((metadata_file.parent / "Test4_Copy_static_costmap.pgm").is_file())
            self.assertFalse((metadata_file.parent / "Test4_boundary.geojson").exists())
            self.assertEqual(
                metadata["recorded_work_area_route_file"],
                str(metadata_file.parent / "Test4_Copy_boundary.geojson"),
            )
            self.assertIn(
                "image: Test4_Copy_static_costmap.pgm",
                (metadata_file.parent / "Test4_Copy_static_costmap.yaml").read_text(encoding="utf-8"),
            )
            self.assertIn("Could not update saved Gaussian capture manifest", metadata["gaussian_error"])
            self.assertNotIn("gaussian_manifest_file", metadata)


if __name__ == "__main__":
    unittest.main()
