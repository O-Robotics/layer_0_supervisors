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


class _UnavailableClient:

    def service_is_ready(self):
        return False


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


class UnifiedMissionPageTest(unittest.TestCase):

    def test_missions_page_contains_full_screen_shell_and_no_map_nav(self) -> None:
        renderer = frontend.MissionFrontendRenderer()
        renderer._site_title = "AMR Sweeper"

        html = renderer.render_map_html()

        self.assertIn("id=\"mission-app\"", html)
        self.assertIn("class=\"mission-app\"", html)
        self.assertIn("id=\"nav-drawer\"", html)
        self.assertIn("id=\"mission-drawer\"", html)
        self.assertIn("id=\"mission-bottom-sheet\"", html)
        self.assertIn("id=\"mission-action-menu\"", html)
        self.assertIn("Mission Selection", html)
        self.assertIn("Map Edit", html)
        self.assertIn("Path Edit", html)
        self.assertIn("id=\"fit-mission-button\"", html)
        self.assertIn("id=\"start-confirmation-sheet\"", html)
        self.assertIn("id=\"new-mission-sheet\"", html)
        self.assertIn("id=\"open-map-edit-button\"", html)
        self.assertIn("id=\"open-path-edit-button\"", html)
        self.assertIn("id=\"rename-mission-button\"", html)
        self.assertIn("defaultMissionIds", html)
        self.assertIn("protectedMissionIds", html)
        self.assertIn("missionIsSelectable", html)
        self.assertIn("data-layer-mode=\"3d\"", html)
        self.assertIn("data-layer=\"work_areas\"", html)
        self.assertIn("data-layer=\"no_go\"", html)
        self.assertIn("data-layer=\"transit\"", html)
        self.assertIn("data-layer=\"station\"", html)
        self.assertIn("id=\"area-edit-toolbar\"", html)
        self.assertIn("id=\"open-area-edit-button\"", html)
        self.assertIn("Mission settings", html)
        self.assertIn("function updateLayerPanelMode", html)
        self.assertNotIn("mapLayerControl.style.display = 'none'", html)
        self.assertNotIn("id=\"mission-list\"", html)
        self.assertNotIn("href=\"/map\"", html)

    def test_deprecated_missions_renderer_uses_canonical_page(self) -> None:
        renderer = frontend.MissionFrontendRenderer()
        renderer._site_title = "AMR Sweeper"

        self.assertEqual(renderer.render_missions_html(), renderer.render_map_html())

    def test_frontend_removes_duplicate_safety_card_id_and_calendar_alerts(self) -> None:
        renderer = frontend.MissionFrontendRenderer()
        renderer._site_title = "AMR Sweeper"

        dashboard_html = renderer.render_index_html()
        calendar_html = renderer.render_calendar_html()

        self.assertNotIn('id="safety-card"', dashboard_html)
        self.assertIn('id="system-status-card"', dashboard_html)
        self.assertIn('id="safety-control-panel"', dashboard_html)
        self.assertEqual(dashboard_html.count('id="drawer-live-status"'), 1)
        self.assertEqual(dashboard_html.count('id="drawer-live-dot"'), 1)
        self.assertNotIn("window.alert", calendar_html)
        self.assertIn("function setBanner", calendar_html)

    def test_teleop_uses_full_viewport_control_shell(self) -> None:
        renderer = frontend.MissionFrontendRenderer()
        renderer._site_title = "AMR Sweeper"

        html = renderer.render_teleop_html()

        self.assertIn('class="app-topbar"', html)
        self.assertIn('id="nav-drawer"', html)
        self.assertIn('id="teleop-topbar-stop"', html)
        self.assertIn('id="teleop-options-dock"', html)
        self.assertIn('id="teleop-options-list"', html)
        self.assertIn('aria-controls="teleop-options-list"', html)
        self.assertIn('.teleop-options-dock {\n      position: relative;', html)
        self.assertNotIn('.teleop-options-dock {\n      position: fixed;', html)
        self.assertIn('id="lights-button"', html)
        self.assertIn('id="camera-button"', html)
        self.assertIn('id="two-stick-button"', html)
        self.assertIn('id="speed-readout"', html)
        self.assertIn("function showSpeedReadout", html)
        self.assertIn("function hideSpeedReadout", html)
        self.assertIn("const scaleRect = scale.shell.getBoundingClientRect()", html)
        self.assertIn("speedReadout.style.left", html)
        self.assertIn("speedReadout.style.top", html)
        self.assertIn("function relaxStickToZero", html)
        self.assertIn("const durationMs = 250 * distance", html)
        self.assertIn("requestAnimationFrame(step)", html)
        self.assertIn("streamCommand();", html)
        self.assertIn("let teleopStopPending = false", html)
        self.assertIn("teleopStopPending ? 'Stopping'", html)
        self.assertIn("Drive Speed", html)
        self.assertIn("Tool Speed", html)
        self.assertNotIn("<h2>Drive</h2>", html)
        self.assertNotIn("<h2>Tools</h2>", html)
        self.assertNotIn('#teleop-topbar-stop {\n        grid-column: 1 / -1', html)
        self.assertNotIn('height: 100dvh;\n      overflow: hidden;', html)
        self.assertIn("RECORDING MAP", html)
        self.assertNotIn("FSM:", html)
        self.assertNotIn('href="/map"', html)

    def test_primary_pages_use_shared_topbar_with_fsm_battery_and_drawer_connection(self) -> None:
        renderer = frontend.MissionFrontendRenderer()
        renderer._site_title = "AMR Sweeper"

        pages = [
            renderer.render_index_html(),
            renderer.render_calendar_html(),
            renderer.render_map_html(),
            renderer.render_teleop_html(),
            renderer.render_developer_html(),
        ]

        for html in pages:
            self.assertIn('data-shared-topbar="true"', html)
            self.assertNotIn('id="topbar-connection"', html)
            self.assertIn('id="topbar-page-state"', html)
            self.assertIn('id="topbar-battery-button"', html)
            self.assertLess(
                html.index('id="topbar-battery-button"'),
                html.index('id="topbar-page-state"'),
            )
            self.assertIn('id="drawer-live-status"', html)
            self.assertIn('id="drawer-live-dot"', html)
            self.assertIn('id="battery-popover"', html)
            self.assertIn("function updateTopbarFromStatus", html)
            self.assertIn("function formatRuntime", html)
            self.assertIn("markTopbarDisconnected", html)

    def test_calendar_defaults_to_compact_expandable_overview_before_editor(self) -> None:
        renderer = frontend.MissionFrontendRenderer()
        renderer._site_title = "AMR Sweeper"

        html = renderer.render_calendar_html()

        self.assertIn('id="calendar-card"', html)
        self.assertIn('id="compact-calendar-grid"', html)
        self.assertIn('class="calendar-detail"', html)
        self.assertIn('id="calendar-expand-button"', html)
        self.assertIn("function renderCompactCalendar", html)
        self.assertIn("function setCalendarExpanded", html)
        self.assertLess(html.index('id="calendar-card"'), html.index('class="editor-layout"'))

    def test_mission_bottom_sheet_has_explicit_expand_collapse_affordance(self) -> None:
        renderer = frontend.MissionFrontendRenderer()
        renderer._site_title = "AMR Sweeper"

        html = renderer.render_map_html()

        self.assertIn('id="sheet-summary-toggle"', html)
        self.assertIn('id="sheet-chevron"', html)
        self.assertIn('aria-expanded="false"', html)
        self.assertIn("function toggleMissionSheet", html)
        self.assertIn("summaryToggle.setAttribute('aria-expanded'", html)

    def test_developer_uses_tabs_and_defines_accent(self) -> None:
        renderer = frontend.MissionFrontendRenderer()
        renderer._site_title = "AMR Sweeper"

        html = renderer.render_developer_html()

        self.assertIn("--accent: #fdca0f", html)
        self.assertIn("developer-panel-logs", html)
        self.assertIn("developer-panel-profile", html)
        self.assertIn("developer-panel-raw", html)
        self.assertIn("Copy JSON", html)

    def test_primary_pages_use_drawer_navigation_without_map_link(self) -> None:
        renderer = frontend.MissionFrontendRenderer()
        renderer._site_title = "AMR Sweeper"

        pages = [
            renderer.render_index_html(),
            renderer.render_calendar_html(),
            renderer.render_map_html(),
            renderer.render_teleop_html(),
            renderer.render_developer_html(),
        ]

        for html in pages:
            self.assertIn('id="nav-drawer"', html)
            self.assertIn('id="open-nav-button"', html)
            self.assertNotIn('href="/map"', html)

    def test_map_route_is_not_rendered_or_redirected(self) -> None:
        source = SCRIPT.read_text(encoding="utf-8")

        self.assertNotIn('parsed.path == "/map"', source)


class OptionalServiceFallbackTest(unittest.TestCase):

    def test_list_missions_returns_fast_local_fallback_when_service_not_ready(self) -> None:
        node = backend.MissionBackendNode.__new__(backend.MissionBackendNode)
        node._list_missions_client = _UnavailableClient()
        node._list_missions_service = "list_executable_missions"

        response = node.list_executable_missions()

        self.assertTrue(response["success"])
        self.assertFalse(response["service_ready"])
        self.assertEqual(response["missions"], [])

    def test_gaussian_status_returns_fast_unavailable_state_when_service_not_ready(self) -> None:
        node = backend.MissionBackendNode.__new__(backend.MissionBackendNode)
        node._gaussian_splat_status_client = _UnavailableClient()
        node._gaussian_splat_status_service = "get_gaussian_splat_status"

        response = node.gaussian_splat_status()

        self.assertTrue(response["success"])
        self.assertFalse(response["service_ready"])
        self.assertEqual(response["status"]["state"], "unavailable")


class SemanticMissionZoneTest(unittest.TestCase):

    def _node_with_saved_mission(self, workspace: Path, mission_id: str = "North_Lawn"):
        mission_directory = workspace / "missions" / "logs" / mission_id
        (mission_directory / "_2D_map").mkdir(parents=True)
        (mission_directory / "_Path").mkdir(parents=True)
        (mission_directory / "mission.json").write_text(
            json.dumps({"mission_id": mission_id, "map_id": mission_id, "name": mission_id}),
            encoding="utf-8",
        )
        node = backend.MissionBackendNode.__new__(backend.MissionBackendNode)
        node._missions_log_directory = str(workspace / "missions" / "logs")
        return node, mission_directory

    def test_create_update_delete_semantic_zone_persists_zone_set(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            node, mission_directory = self._node_with_saved_mission(Path(temp_dir))

            create_response = node.create_mission_zone(
                {
                    "mission_id": "North_Lawn",
                    "zone_id": "north_bed",
                    "name": "North bed",
                    "type": "NO_GO",
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [[[12.0, 55.0], [12.1, 55.0], [12.1, 55.1], [12.0, 55.1]]],
                    },
                }
            )
            update_response = node.update_mission_zone(
                {
                    "mission_id": "North_Lawn",
                    "zone_id": "north_bed",
                    "name": "North bed updated",
                    "type": "NO_GO",
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [[[12.0, 55.0], [12.2, 55.0], [12.2, 55.1], [12.0, 55.1]]],
                    },
                }
            )
            delete_response = node.delete_mission_zone({"mission_id": "North_Lawn", "zone_id": "north_bed"})

            self.assertTrue(create_response["success"])
            self.assertTrue(update_response["success"])
            self.assertTrue(delete_response["success"])
            zone_set = json.loads((mission_directory / "_2D_map" / "zoneSet.json").read_text(encoding="utf-8"))
            self.assertEqual(zone_set["zoneSet"]["zones"], [])
            self.assertTrue((mission_directory / "_Path" / "zoneSet.json").is_file())

    def test_set_station_and_default_mission_settings_are_returned(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            node, mission_directory = self._node_with_saved_mission(Path(temp_dir))

            response = node.set_mission_station(
                {
                    "mission_id": "North_Lawn",
                    "station": {"name": "Charger", "position": {"x": 12.0, "y": 55.0}, "heading": 1.57},
                }
            )

            metadata = json.loads((mission_directory / "mission.json").read_text(encoding="utf-8"))
            self.assertTrue(response["success"])
            self.assertEqual(metadata["station"]["name"], "Charger")
            self.assertEqual(response["map"]["station"]["position"]["x"], 12.0)
            self.assertEqual(response["map"]["mission_settings"]["robot_speed"], "standard")

    def test_invalid_semantic_polygon_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            node, _mission_directory = self._node_with_saved_mission(Path(temp_dir))

            with self.assertRaises(ValueError):
                node.create_mission_zone(
                    {
                        "mission_id": "North_Lawn",
                        "zone_id": "bad",
                        "type": "WORK_AREA",
                        "geometry": {"type": "Polygon", "coordinates": [[[0, 0], [1, 1]]]},
                    }
                )


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
            map_directory = workspace / "missions" / "logs" / "My_Map"
            metadata = {}
            node = backend.MissionBackendNode.__new__(backend.MissionBackendNode)
            node._missions_log_directory = str(workspace / "missions" / "logs")

            node._copy_latest_recorded_map_into_map_directory(map_directory, metadata)

            saved_manifest = map_directory / "_3D_map" / "gaussian" / "manifest.json"
            document = json.loads(saved_manifest.read_text(encoding="utf-8"))
            self.assertEqual(document["output_directory"], str(map_directory / "_3D_map" / "gaussian"))
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

            copied_directory = workspace / "missions" / "logs" / "Test4_Copy"
            copied_manifest = copied_directory / "_3D_map" / "gaussian" / "manifest.json"
            document = json.loads(copied_manifest.read_text(encoding="utf-8"))
            self.assertTrue(response["success"])
            self.assertEqual(response["map"]["map_id"], "Test4_Copy")
            self.assertEqual(document["output_directory"], str(copied_directory / "_3D_map" / "gaussian"))
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

            metadata_file = workspace / "missions" / "logs" / "Empty_Gaussian" / "mission.json"
            metadata = json.loads(metadata_file.read_text(encoding="utf-8"))
            self.assertTrue(response["success"])
            self.assertEqual(response["map"]["map_id"], "Empty_Gaussian")
            self.assertTrue((metadata_file.parent / "_Path" / "Empty_Gaussian_boundary.geojson").is_file())
            self.assertTrue((metadata_file.parent / "_Path" / "Empty_Gaussian_boundary_navsat.geojson").is_file())
            self.assertTrue((metadata_file.parent / "_2D_map" / "Empty_Gaussian_static_costmap.yaml").is_file())
            self.assertTrue((metadata_file.parent / "_2D_map" / "Empty_Gaussian_static_costmap.pgm").is_file())
            self.assertEqual(
                metadata["recorded_work_area_route_file"],
                str(metadata_file.parent / "_Path" / "Empty_Gaussian_boundary.geojson"),
            )
            self.assertIn(
                "image: Empty_Gaussian_static_costmap.pgm",
                (metadata_file.parent / "_2D_map" / "Empty_Gaussian_static_costmap.yaml").read_text(encoding="utf-8"),
            )
            self.assertIn("Could not update saved Gaussian capture manifest", metadata["gaussian_error"])
            self.assertNotIn("gaussian_manifest_file", metadata)

    def test_save_record_map_as_mission_copies_canonical_bundle_folders(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            workspace = Path(temp_dir)
            record_map = workspace / "missions" / "logs" / "RecordMap"
            (record_map / "_2D_map").mkdir(parents=True)
            (record_map / "_3D_map" / "gaussian").mkdir(parents=True)
            (record_map / "_Path").mkdir(parents=True)
            route_file = record_map / "_Path" / "RecordMap_boundary.geojson"
            navsat_file = record_map / "_Path" / "RecordMap_boundary_navsat.geojson"
            costmap_yaml = record_map / "_2D_map" / "RecordMap_static_costmap.yaml"
            costmap_image = record_map / "_2D_map" / "RecordMap_static_costmap.pgm"
            gaussian_manifest = record_map / "_3D_map" / "gaussian" / "manifest.json"
            route_file.write_text('{"type":"FeatureCollection","features":[]}', encoding="utf-8")
            navsat_file.write_text('{"type":"FeatureCollection","features":[]}', encoding="utf-8")
            costmap_yaml.write_text("image: RecordMap_static_costmap.pgm\nresolution: 0.05\n", encoding="utf-8")
            costmap_image.write_text("P2\n1 1\n255\n0\n", encoding="utf-8")
            gaussian_manifest.write_text(
                json.dumps(
                    {
                        "output_directory": str(gaussian_manifest.parent),
                        "gaussian_manifest_file": str(gaussian_manifest),
                        "captures": [],
                    }
                ),
                encoding="utf-8",
            )
            (record_map / "mission.json").write_text(
                json.dumps(
                    {
                        "mission_id": "RecordMap",
                        "recorded_work_area_route_file": str(route_file),
                        "recorded_work_area_navsat_file": str(navsat_file),
                        "recorded_work_area_static_costmap_yaml": str(costmap_yaml),
                        "recorded_work_area_static_costmap_image": str(costmap_image),
                        "gaussian_manifest_file": str(gaussian_manifest),
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
                    "map_id": "North Lawn",
                    "name": "North Lawn",
                    "source": "latest_recorded_map",
                    "overwrite_existing": True,
                }
            )

            mission_directory = workspace / "missions" / "logs" / "North_Lawn"
            metadata = json.loads((mission_directory / "mission.json").read_text(encoding="utf-8"))
            self.assertTrue(response["success"])
            self.assertTrue((mission_directory / "_2D_map").is_dir())
            self.assertTrue((mission_directory / "_3D_map").is_dir())
            self.assertTrue((mission_directory / "_Path").is_dir())
            self.assertEqual(
                metadata["recorded_work_area_route_file"],
                str(mission_directory / "_Path" / "North_Lawn_boundary.geojson"),
            )
            self.assertEqual(
                metadata["gaussian_manifest_file"],
                str(mission_directory / "_3D_map" / "gaussian" / "manifest.json"),
            )

    def test_mission_run_history_reads_timestamped_context_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            workspace = Path(temp_dir)
            run_directory = workspace / "missions" / "logs" / "North_Lawn" / "20260906T120000Z"
            run_directory.mkdir(parents=True)
            actual_path = run_directory / "North_Lawn_20260906T120000Z_path_actual.geojson"
            actual_navsat = run_directory / "North_Lawn_20260906T120000Z_path_navsat.geojson"
            actual_path.write_text('{"type":"FeatureCollection","features":[]}', encoding="utf-8")
            actual_navsat.write_text('{"type":"FeatureCollection","features":[]}', encoding="utf-8")
            (run_directory / "North_Lawn_20260906T120000Z_context.json").write_text(
                json.dumps(
                    {
                        "mission_id": "North_Lawn",
                        "run_started_at": "20260906T120000Z",
                        "actual_path_file": str(actual_path),
                        "actual_path_navsat_file": str(actual_navsat),
                        "actual_path_length_meters": 42.0,
                    }
                ),
                encoding="utf-8",
            )
            node = backend.MissionBackendNode.__new__(backend.MissionBackendNode)
            node._missions_log_directory = str(workspace / "missions" / "logs")

            runs = node._mission_run_history("North Lawn")

            self.assertEqual(len(runs), 1)
            self.assertEqual(runs[0]["run_started_at"], "20260906T120000Z")
            self.assertEqual(runs[0]["actual_path_length_meters"], 42.0)
            self.assertEqual(runs[0]["actual_path_navsat_geojson"]["type"], "FeatureCollection")

    def test_rename_saved_mission_rewrites_metadata_costmap_and_gaussian_manifests(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            workspace = Path(temp_dir)
            mission_directory = workspace / "missions" / "logs" / "Old_Mission"
            (mission_directory / "_2D_map").mkdir(parents=True)
            (mission_directory / "_3D_map" / "gaussian").mkdir(parents=True)
            (mission_directory / "_3D_map" / "gaussian_splat").mkdir(parents=True)
            (mission_directory / "_Path").mkdir(parents=True)
            costmap_yaml = mission_directory / "_2D_map" / "Old_Mission_static_costmap.yaml"
            costmap_image = mission_directory / "_2D_map" / "Old_Mission_static_costmap.pgm"
            route_file = mission_directory / "_Path" / "Old_Mission_boundary.geojson"
            gaussian_manifest = mission_directory / "_3D_map" / "gaussian" / "manifest.json"
            splat_manifest = mission_directory / "_3D_map" / "gaussian_splat" / "gaussian_splat_manifest.json"
            costmap_yaml.write_text("image: Old_Mission_static_costmap.pgm\nresolution: 0.05\n", encoding="utf-8")
            costmap_image.write_text("P2\n1 1\n255\n0\n", encoding="utf-8")
            route_file.write_text('{"type":"FeatureCollection","features":[]}', encoding="utf-8")
            gaussian_manifest.write_text(
                json.dumps(
                    {
                        "output_directory": str(gaussian_manifest.parent),
                        "gaussian_manifest_file": str(gaussian_manifest),
                        "captures": [],
                    }
                ),
                encoding="utf-8",
            )
            splat_manifest.write_text(
                json.dumps(
                    {
                        "artifact_directory": str(splat_manifest.parent),
                        "source_capture_manifest_file": str(gaussian_manifest),
                        "tiles": [],
                    }
                ),
                encoding="utf-8",
            )
            (mission_directory / "mission.json").write_text(
                json.dumps(
                    {
                        "map_id": "Old_Mission",
                        "mission_id": "Old_Mission",
                        "name": "Old_Mission",
                        "recorded_work_area_route_file": str(route_file),
                        "recorded_work_area_static_costmap_yaml": str(costmap_yaml),
                        "recorded_work_area_static_costmap_image": str(costmap_image),
                        "gaussian_manifest_file": str(gaussian_manifest),
                        "gaussian_splat_manifest_file": str(splat_manifest),
                    }
                ),
                encoding="utf-8",
            )
            node = backend.MissionBackendNode.__new__(backend.MissionBackendNode)
            node._missions_log_directory = str(workspace / "missions" / "logs")

            response = node.rename_mission({"mission_id": "Old Mission", "new_mission_id": "New Mission"})

            renamed_directory = workspace / "missions" / "logs" / "New_Mission"
            metadata = json.loads((renamed_directory / "mission.json").read_text(encoding="utf-8"))
            renamed_gaussian_manifest = renamed_directory / "_3D_map" / "gaussian" / "manifest.json"
            renamed_splat_manifest = renamed_directory / "_3D_map" / "gaussian_splat" / "gaussian_splat_manifest.json"
            gaussian_document = json.loads(renamed_gaussian_manifest.read_text(encoding="utf-8"))
            splat_document = json.loads(renamed_splat_manifest.read_text(encoding="utf-8"))
            self.assertTrue(response["success"])
            self.assertFalse((workspace / "missions" / "logs" / "Old_Mission").exists())
            self.assertEqual(metadata["map_id"], "New_Mission")
            self.assertEqual(metadata["mission_id"], "New_Mission")
            self.assertEqual(
                metadata["recorded_work_area_route_file"],
                str(renamed_directory / "_Path" / "New_Mission_boundary.geojson"),
            )
            self.assertIn(
                "image: New_Mission_static_costmap.pgm",
                (renamed_directory / "_2D_map" / "New_Mission_static_costmap.yaml").read_text(encoding="utf-8"),
            )
            self.assertEqual(gaussian_document["output_directory"], str(renamed_gaussian_manifest.parent))
            self.assertEqual(splat_document["artifact_directory"], str(renamed_splat_manifest.parent))
            self.assertEqual(splat_document["source_capture_manifest_file"], str(renamed_gaussian_manifest))

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

            metadata_file = workspace / "missions" / "logs" / "Test4_Copy" / "mission.json"
            metadata = json.loads(metadata_file.read_text(encoding="utf-8"))
            self.assertTrue(response["success"])
            self.assertEqual(response["map"]["map_id"], "Test4_Copy")
            self.assertTrue((metadata_file.parent / "_Path" / "Test4_Copy_boundary.geojson").is_file())
            self.assertTrue((metadata_file.parent / "_Path" / "Test4_Copy_boundary_navsat.geojson").is_file())
            self.assertTrue((metadata_file.parent / "_2D_map" / "Test4_Copy_static_costmap.yaml").is_file())
            self.assertTrue((metadata_file.parent / "_2D_map" / "Test4_Copy_static_costmap.pgm").is_file())
            self.assertFalse((metadata_file.parent / "_Path" / "Test4_boundary.geojson").exists())
            self.assertEqual(
                metadata["recorded_work_area_route_file"],
                str(metadata_file.parent / "_Path" / "Test4_Copy_boundary.geojson"),
            )
            self.assertIn(
                "image: Test4_Copy_static_costmap.pgm",
                (metadata_file.parent / "_2D_map" / "Test4_Copy_static_costmap.yaml").read_text(encoding="utf-8"),
            )
            self.assertIn("Could not update saved Gaussian capture manifest", metadata["gaussian_error"])
            self.assertNotIn("gaussian_manifest_file", metadata)


if __name__ == "__main__":
    unittest.main()
