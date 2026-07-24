import copy
import json
import math
import os
import random
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_GZ_SIM_SIGTERM_TIMEOUT_SEC = "15.0"
_GZ_SIM_SIGKILL_TIMEOUT_SEC = "5.0"


def _normalize_namespace(namespace: str) -> str:
    return namespace.strip().strip("/")


def _absolute_topic(namespace: str, topic_suffix: str) -> str:
    suffix = topic_suffix.strip().lstrip("/")
    normalized_namespace = _normalize_namespace(namespace)
    if normalized_namespace:
        return f"/{normalized_namespace}/{suffix}"
    return f"/{suffix}"


def _child_namespace(namespace: str, child: str) -> str:
    normalized_namespace = _normalize_namespace(namespace)
    normalized_child = child.strip().strip("/")
    if normalized_namespace:
        return f"{normalized_namespace}/{normalized_child}"
    return normalized_child


def _deep_merge_dict(base: dict, override: dict) -> dict:
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge_dict(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _resolve_simulation_profile(config: dict, profile_name: str) -> tuple[str, dict]:
    simulation_config = config["simulation"]
    default_profile = simulation_config.get("default_profile", "empty1")
    selected_profile = (profile_name or default_profile).strip() or default_profile
    profiles = simulation_config.get("profiles", {})
    if selected_profile not in profiles:
        available_profiles = ", ".join(sorted(profiles))
        raise ValueError(
            f"Unknown simulation profile '{selected_profile}'. "
            f"Available profiles: {available_profiles}"
        )

    defaults = {
        key: value
        for key, value in simulation_config.items()
        if key not in {"default_profile", "profiles"}
    }
    resolved = _deep_merge_dict(defaults, profiles[selected_profile])
    resolved.setdefault("world_name", selected_profile)
    resolved.setdefault("world_file", f"{selected_profile}.sdf")
    return selected_profile, resolved


def _resolve_world_path(simulation_pkg: str, world_file: str) -> str:
    if os.path.isabs(world_file):
        return world_file
    return os.path.join(simulation_pkg, "worlds", world_file)


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _point_in_polygon(point: tuple[float, float], polygon: list[tuple[float, float]]) -> bool:
    x, y = point
    inside = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        denominator = yj - yi
        if abs(denominator) < 1.0e-12:
            denominator = 1.0e-12
        if ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / denominator + xi
        ):
            inside = not inside
        j = i
    return inside


def _node_sequence_from_edges(
    edge_ids: list[str],
    edges_by_id: dict[str, dict],
    close_loop: bool,
) -> list[str]:
    sequence: list[str] = []
    for edge_id in edge_ids:
        edge = edges_by_id[edge_id]
        start_node_id = edge["startNodeId"]
        end_node_id = edge["endNodeId"]
        if not sequence:
            sequence.append(start_node_id)
        elif sequence[-1] != start_node_id:
            sequence.append(start_node_id)
        sequence.append(end_node_id)
    if close_loop and len(sequence) > 1 and sequence[0] == sequence[-1]:
        sequence.pop()
    return sequence


def _load_mission_polygon(mission_path: str) -> tuple[list[tuple[float, float]], str]:
    with open(mission_path, "r", encoding="utf-8") as mission_file:
        mission = json.load(mission_file)

    reference = mission.get("missionReference", {})
    coordinate_frame = str(reference.get("coordinateFrame", "")).strip().lower()
    frame = "local" if coordinate_frame in {"local", "odom"} else "wgs84"

    nodes_by_id = {
        node["nodeId"]: node["nodePosition"]
        for node in mission.get("nodes", [])
    }
    edges_by_id = {
        edge["edgeId"]: edge
        for edge in mission.get("edges", [])
    }
    working_zones = mission.get("missionGeometries", {}).get("workingZones", [])
    if not working_zones:
        raise ValueError(f"Mission {mission_path} has no working zones")

    edge_ids = working_zones[0].get("edgeIds", [])
    node_ids = _node_sequence_from_edges(edge_ids, edges_by_id, close_loop=True)
    polygon = [
        (float(nodes_by_id[node_id]["x"]), float(nodes_by_id[node_id]["y"]))
        for node_id in node_ids
    ]
    if len(polygon) < 3:
        raise ValueError(f"Mission {mission_path} working zone has fewer than three vertices")
    return polygon, frame


def _invert_affine_xy_to_wgs84(point: tuple[float, float], georeference: dict) -> tuple[float, float]:
    lon, lat = point
    lon_coefficients = georeference["longitude_coefficients"]
    lat_coefficients = georeference["latitude_coefficients"]
    a, b, c = [float(value) for value in lon_coefficients]
    d, e, f = [float(value) for value in lat_coefficients]
    determinant = a * e - b * d
    if abs(determinant) < 1.0e-18:
        raise ValueError("Mission georeference affine transform is singular")
    lon_delta = lon - c
    lat_delta = lat - f
    x = (lon_delta * e - b * lat_delta) / determinant
    y = (a * lat_delta - d * lon_delta) / determinant
    return x, y


def _resolve_mission_file_path(simulation_config: dict) -> str:
    configured_path = simulation_config.get("random_spawn_mission_path", "")
    if not configured_path:
        configured_path = os.path.join(
            "missions",
            "database",
            "simulations",
            f"{simulation_config['mission_id']}.json",
        )
    if os.path.isabs(configured_path):
        return configured_path
    return os.path.join(os.getcwd(), configured_path)


def _resolve_spawn_pose(simulation_config: dict) -> dict:
    spawn_pose = copy.deepcopy(simulation_config["spawn_pose"])
    if not _as_bool(simulation_config.get("random_spawn", False)):
        return spawn_pose

    mission_path = _resolve_mission_file_path(simulation_config)
    polygon, frame = _load_mission_polygon(mission_path)
    if frame == "wgs84":
        georeference = simulation_config.get("mission_georeference", {})
        polygon = [
            _invert_affine_xy_to_wgs84(point, georeference)
            for point in polygon
        ]

    min_x = min(point[0] for point in polygon)
    max_x = max(point[0] for point in polygon)
    min_y = min(point[1] for point in polygon)
    max_y = max(point[1] for point in polygon)

    for _ in range(1000):
        candidate = (random.uniform(min_x, max_x), random.uniform(min_y, max_y))
        if _point_in_polygon(candidate, polygon):
            spawn_pose["x"] = candidate[0]
            spawn_pose["y"] = candidate[1]
            spawn_pose["yaw"] = random.uniform(-math.pi, math.pi)
            return spawn_pose

    raise RuntimeError(f"Failed to sample random spawn inside {mission_path}")


def _simulation_resource_paths(simulation_pkg: str, description_share: str) -> list[str]:
    worlds_dir = os.path.join(simulation_pkg, "worlds")
    local_models = os.path.join(worlds_dir, "models")
    candidates = [
        os.path.dirname(simulation_pkg),
        os.path.dirname(description_share),
        worlds_dir,
        local_models,
    ]
    resource_paths = []
    for candidate in candidates:
        if os.path.exists(candidate) and candidate not in resource_paths:
            resource_paths.append(candidate)
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "").strip()
    if existing_resource_path:
        for candidate in existing_resource_path.split(os.pathsep):
            normalized = candidate.strip()
            if normalized and normalized not in resource_paths:
                resource_paths.append(normalized)
    return resource_paths


def _available_model_names(resource_paths: list[str]) -> set[str]:
    model_names = set()
    for resource_path in resource_paths:
        if not os.path.isdir(resource_path):
            continue
        try:
            for entry in os.scandir(resource_path):
                if entry.is_dir():
                    model_names.add(entry.name)
        except OSError:
            continue
    return model_names


def _iter_model_references(element: ET.Element):
    for descendant in element.iter():
        text = (descendant.text or "").strip()
        if text.startswith("model://"):
            yield text


def _missing_model_names(element: ET.Element, available_model_names: set[str]) -> set[str]:
    missing = set()
    for reference in _iter_model_references(element):
        match = re.match(r"model://([^/]+)", reference)
        if match:
            model_name = match.group(1)
            if model_name not in available_model_names:
                missing.add(model_name)
    return missing


def _strip_missing_model_elements(world_sdf: str, available_model_names: set[str]) -> tuple[str, set[str]]:
    root = ET.fromstring(world_sdf)
    parent_map = {child: parent for parent in root.iter() for child in parent}
    removed_model_names = set()

    for element in list(root.iter()):
        if element.tag not in {"include", "model"}:
            continue
        missing_model_names = _missing_model_names(element, available_model_names)
        if not missing_model_names:
            continue
        parent = parent_map.get(element)
        if parent is None:
            continue
        parent.remove(element)
        removed_model_names.update(missing_model_names)

    return ET.tostring(root, encoding="unicode"), removed_model_names


def _strip_incompatible_model_includes(world_sdf: str) -> tuple[str, set[str]]:
    incompatible_model_uris = set()
    root = ET.fromstring(world_sdf)
    parent_map = {child: parent for parent in root.iter() for child in parent}
    removed_model_uris = set()

    for include in list(root.iter("include")):
        uri = include.find("uri")
        if uri is None:
            continue
        model_uri = (uri.text or "").strip()
        if model_uri not in incompatible_model_uris:
            continue
        parent = parent_map.get(include)
        if parent is None:
            continue
        parent.remove(include)
        removed_model_uris.add(model_uri)

    return ET.tostring(root, encoding="unicode"), removed_model_uris


def _strip_legacy_classic_plugins(world_sdf: str) -> tuple[str, int]:
    root = ET.fromstring(world_sdf)
    parent_map = {child: parent for parent in root.iter() for child in parent}
    removed_plugin_count = 0

    for plugin in list(root.iter("plugin")):
        filename = (plugin.get("filename") or "").strip()
        if not filename.startswith("lib") or not filename.endswith(".so"):
            continue
        parent = parent_map.get(plugin)
        if parent is None:
            continue
        parent.remove(plugin)
        removed_plugin_count += 1

    return ET.tostring(root, encoding="unicode"), removed_plugin_count


def _strip_actor_elements(world_sdf: str) -> tuple[str, int]:
    root = ET.fromstring(world_sdf)
    parent_map = {child: parent for parent in root.iter() for child in parent}
    removed_actor_count = 0

    for actor in list(root.iter("actor")):
        parent = parent_map.get(actor)
        if parent is None:
            continue
        parent.remove(actor)
        removed_actor_count += 1

    return ET.tostring(root, encoding="unicode"), removed_actor_count


def _strip_road_elements(world_sdf: str) -> tuple[str, int]:
    root = ET.fromstring(world_sdf)
    parent_map = {child: parent for parent in root.iter() for child in parent}
    removed_road_count = 0

    for road in list(root.iter("road")):
        parent = parent_map.get(road)
        if parent is None:
            continue
        parent.remove(road)
        removed_road_count += 1

    return ET.tostring(root, encoding="unicode"), removed_road_count


def _strip_inline_mesh_collision_elements(world_sdf: str) -> tuple[str, int]:
    root = ET.fromstring(world_sdf)
    parent_map = {child: parent for parent in root.iter() for child in parent}
    removed_collision_count = 0

    for collision in list(root.iter("collision")):
        if collision.find(".//mesh") is None:
            continue
        parent = parent_map.get(collision)
        if parent is None:
            continue
        parent.remove(collision)
        removed_collision_count += 1

    return ET.tostring(root, encoding="unicode"), removed_collision_count


def _set_child_text(element: ET.Element, tag_name: str, value: str) -> None:
    child = element.find(tag_name)
    if child is None:
        child = ET.SubElement(element, tag_name)
    child.text = value


def _upgrade_legacy_materials(world_sdf: str) -> tuple[str, int]:
    asphalt_texture = "model://asphalt_plane/materials/textures/tarmac.png"
    legacy_color_materials = {
        "CitySim/ShinyGrey": "0.55 0.55 0.55 1.0",
        "Gazebo/Residential": "0.42 0.42 0.42 1.0",
    }
    root = ET.fromstring(world_sdf)
    parent_map = {child: parent for parent in root.iter() for child in parent}
    upgraded_material_count = 0

    for script in list(root.iter("script")):
        uri = script.find("uri")
        name = script.find("name")
        if uri is None or name is None:
            continue
        script_uri = (uri.text or "").strip()
        script_name = (name.text or "").strip()
        if (
            script_uri != "file://media/materials/scripts/gazebo.material"
            and script_name not in {"vrc/asphalt", "CitySim/ShinyGrey"}
        ):
            continue
        if script_name not in {"Gazebo/Residential", "vrc/asphalt", "CitySim/ShinyGrey"}:
            continue

        material = parent_map.get(script)
        if material is None:
            continue
        material.remove(script)
        material_color = legacy_color_materials.get(script_name, "0.42 0.42 0.42 1.0")
        _set_child_text(material, "ambient", material_color)
        _set_child_text(material, "diffuse", material_color)
        _set_child_text(material, "specular", "0.12 0.12 0.12 1.0")

        if script_name == "vrc/asphalt":
            pbr = material.find("pbr")
            if pbr is None:
                pbr = ET.SubElement(material, "pbr")
            metal = pbr.find("metal")
            if metal is None:
                metal = ET.SubElement(pbr, "metal")
            _set_child_text(metal, "albedo_map", asphalt_texture)
            _set_child_text(metal, "roughness", "0.85")
            _set_child_text(metal, "metalness", "0.0")
        upgraded_material_count += 1

    for material in root.iter("material"):
        ambient = material.find("ambient")
        diffuse = material.find("diffuse")
        if ambient is None or diffuse is not None:
            continue
        ambient_value = (ambient.text or "").strip()
        if not ambient_value:
            continue
        _set_child_text(material, "diffuse", ambient_value)
        upgraded_material_count += 1

    return ET.tostring(root, encoding="unicode"), upgraded_material_count


def _expand_bridge_template(value: str, world_name: str, entity_name: str) -> str:
    return value.replace("{world_name}", world_name).replace("{entity_name}", entity_name)


def _bridge_config(bridge: dict, namespace: str, world_name: str, entity_name: str) -> dict:
    ros_topic_name = _expand_bridge_template(bridge["ros_topic_name"], world_name, entity_name)
    if not ros_topic_name.startswith("/"):
        ros_topic_name = _absolute_topic(namespace, ros_topic_name)
    gz_topic_name = _expand_bridge_template(
        bridge.get("gz_topic_name", bridge["ros_topic_name"]),
        world_name,
        entity_name,
    )
    if not gz_topic_name.startswith("/"):
        gz_topic_name = "/" + gz_topic_name.lstrip("/")
    return {
        "ros_topic_name": ros_topic_name,
        "gz_topic_name": gz_topic_name,
        "ros_type_name": bridge["ros_type_name"],
        "gz_type_name": _expand_bridge_template(bridge["gz_type_name"], world_name, entity_name),
        "direction": bridge["direction"],
    }


def _render_simulation_robot(namespace: str, entity_name: str) -> str:
    description_pkg = get_package_share_directory("amr_sweeper_description")
    xacro_file = os.path.join(description_pkg, "urdf", "robot", "gazebo.urdf.xacro")
    robot_description = subprocess.check_output(
        [
            "xacro",
            xacro_file,
            f"robot_namespace:={namespace}",
            f"entity_name:={entity_name}",
            "use_simulation:=true",
            "simulation_physics:=true",
            "use_ros2_control:=false",
            "enable_usb_cameras:=false",
            "enable_gnss:=true",
            "enable_imu:=true",
            "enable_depth_camera:=true",
        ],
        text=True,
    )
    urdf_file = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="amr_sweeper_sim_",
        suffix=".urdf",
        delete=False,
        encoding="utf-8",
    )
    with urdf_file:
        urdf_file.write(robot_description)
    return urdf_file.name


def _rename_world(world_sdf: str, world_name: str) -> str:
    def replace_name(match):
        return f"{match.group(1)}{match.group(2)}{world_name}{match.group(4)}"

    world_sdf = re.sub(
        r"(<world\s+name=)(['\"])([^'\"]+)(['\"])",
        replace_name,
        world_sdf,
        count=1,
    )
    world_sdf = re.sub(
        r"(<state\s+world_name=)(['\"])([^'\"]+)(['\"])",
        replace_name,
        world_sdf,
    )
    return world_sdf


def _ensure_gz_sim_system_plugins(world_sdf: str) -> str:
    if "gz-sim-physics-system" in world_sdf:
        return world_sdf

    plugins = """
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>
"""
    return re.sub(r"(<world\b[^>]*>)", r"\1\n" + plugins, world_sdf, count=1)


def _render_world_with_georeference(
    world_path: str,
    world_name: str,
    georeference: dict,
    available_model_names: set[str],
) -> str:
    with open(world_path, "r", encoding="utf-8") as stream:
        world_sdf = stream.read()

    world_sdf = _rename_world(world_sdf, world_name)
    world_sdf, removed_plugin_count = _strip_legacy_classic_plugins(world_sdf)
    if removed_plugin_count:
        print(
            "[amr_sweeper_simulation] Removed legacy Gazebo Classic plugins: "
            f"{removed_plugin_count}"
        )
    world_sdf, removed_actor_count = _strip_actor_elements(world_sdf)
    if removed_actor_count:
        print(
            "[amr_sweeper_simulation] Removed legacy actor animations: "
            f"{removed_actor_count}"
        )
    world_sdf, removed_road_count = _strip_road_elements(world_sdf)
    if removed_road_count:
        print(
            "[amr_sweeper_simulation] Removed Gazebo road collision geometry: "
            f"{removed_road_count}"
        )
    world_sdf, removed_inline_mesh_collision_count = _strip_inline_mesh_collision_elements(
        world_sdf
    )
    if removed_inline_mesh_collision_count:
        print(
            "[amr_sweeper_simulation] Removed inline mesh collision geometry: "
            f"{removed_inline_mesh_collision_count}"
        )
    world_sdf = _ensure_gz_sim_system_plugins(world_sdf)
    world_sdf, removed_model_names = _strip_missing_model_elements(
        world_sdf, available_model_names
    )
    if removed_model_names:
        print(
            "[amr_sweeper_simulation] Removed missing world models: "
            + ", ".join(sorted(removed_model_names))
        )
    world_sdf, removed_incompatible_model_uris = _strip_incompatible_model_includes(
        world_sdf
    )
    if removed_incompatible_model_uris:
        print(
            "[amr_sweeper_simulation] Removed incompatible legacy world models: "
            + ", ".join(sorted(removed_incompatible_model_uris))
        )
    world_sdf, upgraded_material_count = _upgrade_legacy_materials(
        world_sdf
    )
    if upgraded_material_count:
        print(
            "[amr_sweeper_simulation] Upgraded legacy world materials: "
            f"{upgraded_material_count}"
        )

    replacements = {
        "latitude_deg": georeference["latitude_deg"],
        "longitude_deg": georeference["longitude_deg"],
        "elevation": georeference["elevation_m"],
        "heading_deg": georeference["heading_deg"],
    }
    for tag_name, value in replacements.items():
        world_sdf = re.sub(
            rf"(<{tag_name}>)([^<]+)(</{tag_name}>)",
            rf"\g<1>{value}\g<3>",
            world_sdf,
            count=1,
        )

    world_file = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="amr_sweeper_sim_world_",
        suffix=".sdf",
        delete=False,
        encoding="utf-8",
    )
    with world_file:
        world_file.write(world_sdf)
    return world_file.name


def _render_rviz_config(rviz_config_path: str, namespace: str) -> str:
    with open(rviz_config_path, "r", encoding="utf-8") as stream:
        rviz_config = stream.read()

    namespace_prefix = ""
    normalized_namespace = _normalize_namespace(namespace)
    if normalized_namespace:
        namespace_prefix = f"/{normalized_namespace}"
    rviz_config = rviz_config.replace("__NS_PREFIX__", namespace_prefix)

    rendered_rviz_config = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="amr_sweeper_sim_rviz_",
        suffix=".rviz",
        delete=False,
        encoding="utf-8",
    )
    with rendered_rviz_config:
        rendered_rviz_config.write(rviz_config)
    return rendered_rviz_config.name


def _gnss_launch_file() -> str:
    return os.path.join(
        get_package_share_directory("amr_sweeper_gnss"),
        "launch",
        "amr_sweeper_gnss.launch.py",
    )


def _launch_setup(context, *args, **kwargs):
    simulation_pkg = get_package_share_directory("amr_sweeper_simulation")
    config_path = os.path.join(simulation_pkg, "config", "simulation.yaml")
    with open(config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    simulation_config = config["simulation"]
    gnss_config = simulation_config["gnss"]
    namespace = LaunchConfiguration("namespace").perform(context)
    use_ntrip_client = LaunchConfiguration("use_ntrip_client").perform(context)
    launch_gnss_stack = LaunchConfiguration("launch_gnss_stack").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)
    launch_gz_gui = LaunchConfiguration("launch_gz_gui").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context).strip()
    if not rviz_config:
        rviz_config = os.path.join(
            simulation_pkg,
            "rviz",
            "amr_sweeper_simulation.rviz",
        )
    override_timestamps_with_wall_time = (
        LaunchConfiguration("override_timestamps_with_wall_time").perform(context).strip().lower()
        in {"1", "true", "yes", "on"}
    )
    simulation_profile = LaunchConfiguration("simulation_profile").perform(context).strip()
    selected_profile, simulation_config = _resolve_simulation_profile(config, simulation_profile)
    world_name = simulation_config["world_name"]
    entity_name = simulation_config["entity_name"]
    base_world = _resolve_world_path(simulation_pkg, simulation_config["world_file"])
    georeference = simulation_config["georeference"]
    description_share = get_package_share_directory("amr_sweeper_description")
    resource_paths = _simulation_resource_paths(simulation_pkg, description_share)
    world = _render_world_with_georeference(
        base_world,
        world_name,
        georeference,
        _available_model_names(resource_paths),
    )
    robot_urdf = _render_simulation_robot(namespace, entity_name)
    rendered_rviz_config = _render_rviz_config(rviz_config, namespace)

    spawn_pose = _resolve_spawn_pose(simulation_config)
    launch_gz_gui_enabled = launch_gz_gui.strip().lower() in {"1", "true", "yes", "on"}
    gazebo_cmd = ["gz", "sim", "-r", world] if launch_gz_gui_enabled else ["gz", "sim", "-s", "-r", world]
    gazebo_kwargs = {
        "cmd": gazebo_cmd,
        "additional_env": {
            "GZ_SIM_RESOURCE_PATH": os.pathsep.join(resource_paths),
        },
        "output": "screen",
    }
    if launch_gz_gui_enabled:
        # `gz sim` without `-s` uses a Ruby wrapper that forks GUI + server and
        # shuts them down sequentially. Give that wrapper time to complete its
        # own cleanup without launch sending a second signal mid-shutdown.
        gazebo_kwargs["sigterm_timeout"] = _GZ_SIM_SIGTERM_TIMEOUT_SEC
        gazebo_kwargs["sigkill_timeout"] = _GZ_SIM_SIGKILL_TIMEOUT_SEC
    gazebo = ExecuteProcess(**gazebo_kwargs)

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_amr_sweeper",
        output="screen",
        arguments=[
            "-world", world_name,
            "-file", robot_urdf,
            "-name", entity_name,
            "-x", str(spawn_pose["x"]),
            "-y", str(spawn_pose["y"]),
            "-z", str(spawn_pose["z"]),
            "-R", str(spawn_pose["roll"]),
            "-P", str(spawn_pose["pitch"]),
            "-Y", str(spawn_pose["yaw"]),
            "-allow_renaming", "false",
        ],
    )
    bridge_configs = {
        f"bridge_{index}": _bridge_config(bridge, namespace, world_name, entity_name)
        for index, bridge in enumerate(config["bridges"])
    }
    bridge = Node(
        package="amr_sweeper_simulation",
        executable="ros_gz_bridge_wrapper.py",
        name="gz_bridge",
        output="screen",
        parameters=[{
            "override_timestamps_with_wall_time": override_timestamps_with_wall_time,
            "expand_gz_topic_names": False,
            "bridge_names": sorted(bridge_configs.keys()),
            "bridges": bridge_configs,
        }],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    world_pose_relay = Node(
        package="amr_sweeper_simulation",
        executable="world_pose_to_sim_pose.py",
        name="world_pose_to_sim_pose",
        output="screen",
        parameters=[{
            "input_topic": _absolute_topic("", f"world/{world_name}/pose/info"),
            "output_topic": _absolute_topic(namespace, "simulation/pose/info"),
        }],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rendered_rviz_config],
        parameters=[{"use_sim_time": True}],
    )

    actions = [
        gazebo,
        LogInfo(
            msg=(
                "[amr_sweeper_simulation] Launching simulation profile "
                f"'{selected_profile}' in world '{world_name}'"
            )
        ),
        spawn_robot,
        bridge,
        world_pose_relay,
    ]

    if launch_rviz.strip().lower() in {"1", "true", "yes", "on"}:
        actions.append(rviz)

    if launch_gnss_stack.strip().lower() in {"1", "true", "yes", "on"}:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_gnss_launch_file()),
                launch_arguments={
                    "gnss_namespace": _child_namespace(namespace, "gnss"),
                    "use_simulation": "true",
                    "use_sim_time": "true",
                    "use_ntrip_client": use_ntrip_client,
                    "use_nmea_to_caster": use_ntrip_client,
                    "gnss_frame_id": gnss_config["frame_id"],
                    "fix_topic": "navsat",
                    "robot_pose_topic": _absolute_topic(namespace, "simulation/robot_pose"),
                    "navsat_topic": gnss_config["navsat_topic"],
                    "gpsfix_topic": gnss_config["gpsfix_topic"],
                    "odometry_topic": gnss_config["odometry_topic"],
                    "rtcm_topic": gnss_config.get("rtcm_topic", "ntrip_client/rtcm"),
                    "origin_lat": str(georeference["latitude_deg"]),
                    "origin_lon": str(georeference["longitude_deg"]),
                    "origin_alt": str(georeference["elevation_m"]),
                    "publish_rate_hz": str(gnss_config.get("publish_rate_hz", 5.0)),
                    "noise_correlation_tau_s": str(gnss_config.get("noise_correlation_tau_s", 12.0)),
                    "autonomous_noise_h_m": str(gnss_config.get("autonomous_noise_h_m", 1.25)),
                    "autonomous_noise_v_m": str(gnss_config.get("autonomous_noise_v_m", 2.5)),
                    "dgps_noise_h_m": str(gnss_config.get("dgps_noise_h_m", 0.7)),
                    "dgps_noise_v_m": str(gnss_config.get("dgps_noise_v_m", 1.3)),
                    "rtk_float_noise_h_m": str(gnss_config.get("rtk_float_noise_h_m", 0.3)),
                    "rtk_float_noise_v_m": str(gnss_config.get("rtk_float_noise_v_m", 0.55)),
                    "rtk_fixed_noise_h_m": str(gnss_config.get("rtk_fixed_noise_h_m", 0.12)),
                    "rtk_fixed_noise_v_m": str(gnss_config.get("rtk_fixed_noise_v_m", 0.25)),
                    "min_horizontal_stddev_m": str(gnss_config.get("min_horizontal_stddev_m", 0.5)),
                    "min_vertical_stddev_m": str(gnss_config.get("min_vertical_stddev_m", 1.0)),
                    "horizontal_covariance_scale": str(gnss_config.get("horizontal_covariance_scale", 4.0)),
                    "vertical_covariance_scale": str(gnss_config.get("vertical_covariance_scale", 4.0)),
                    "autonomous_satellites": str(gnss_config.get("autonomous_satellites", 10)),
                    "corrected_satellites": str(gnss_config.get("corrected_satellites", 14)),
                    "correction_timeout_s": str(gnss_config.get("correction_timeout_s", 3.0)),
                    "dgps_warmup_s": str(gnss_config.get("dgps_warmup_s", 2.0)),
                    "rtk_float_warmup_s": str(gnss_config.get("rtk_float_warmup_s", 8.0)),
                    "simulated_ntrip_publish_rate_hz": str(gnss_config.get("simulated_ntrip_publish_rate_hz", 1.0)),
                    "simulated_ntrip_startup_delay_s": str(gnss_config.get("simulated_ntrip_startup_delay_s", 1.5)),
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("use_ntrip_client", default_value="true"),
        DeclareLaunchArgument("launch_gnss_stack", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("launch_gz_gui", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                get_package_share_directory("amr_sweeper_simulation"),
                "rviz",
                "amr_sweeper_simulation.rviz",
            ),
        ),
        DeclareLaunchArgument("simulation_profile", default_value="empty1"),
        DeclareLaunchArgument("enable_gnss", default_value="true"),
        DeclareLaunchArgument("enable_imu", default_value="true"),
        DeclareLaunchArgument("enable_depth_camera", default_value="true"),
        DeclareLaunchArgument("override_timestamps_with_wall_time", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
