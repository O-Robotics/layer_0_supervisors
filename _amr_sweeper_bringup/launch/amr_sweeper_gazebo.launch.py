import os
import re
import subprocess
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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


def _bridge_argument(bridge: dict, namespace: str) -> str:
    ros_topic_name = bridge["ros_topic_name"]
    if not ros_topic_name.startswith("/"):
        ros_topic_name = _absolute_topic(namespace, ros_topic_name)
    direction = bridge["direction"].strip().lower()
    separator = "[" if direction == "gz_to_ros" else "]"
    return f"{ros_topic_name}@{bridge['ros_type_name']}{separator}{bridge['gz_type_name']}"


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


def _render_world_with_georeference(world_path: str, georeference: dict) -> str:
    with open(world_path, "r", encoding="utf-8") as stream:
        world_sdf = stream.read()

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


def _gnss_launch_file() -> str:
    return os.path.join(
        get_package_share_directory("amr_sweeper_gnss"),
        "launch",
        "amr_sweeper_gnss.launch.py",
    )


def _launch_setup(context, *args, **kwargs):
    bringup_pkg = get_package_share_directory("amr_sweeper_bringup")
    config_path = os.path.join(bringup_pkg, "simulation", "config", "simulation.yaml")
    with open(config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    simulation_config = config["simulation"]
    gnss_config = simulation_config["gnss"]
    namespace = LaunchConfiguration("namespace").perform(context)
    use_ntrip_client = LaunchConfiguration("use_ntrip_client").perform(context)
    launch_gnss_stack = LaunchConfiguration("launch_gnss_stack").perform(context)
    override_timestamps_with_wall_time = (
        LaunchConfiguration("override_timestamps_with_wall_time").perform(context).strip().lower()
        in {"1", "true", "yes", "on"}
    )
    world_name = simulation_config["world_name"]
    entity_name = simulation_config["entity_name"]
    base_world = os.path.join(bringup_pkg, "simulation", "worlds", simulation_config["world_file"])
    georeference = simulation_config["georeference"]
    world = _render_world_with_georeference(base_world, georeference)
    description_share = get_package_share_directory("amr_sweeper_description")
    robot_urdf = _render_simulation_robot(namespace, entity_name)

    spawn_pose = simulation_config["spawn_pose"]
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world],
        additional_env={
            "GZ_SIM_RESOURCE_PATH": os.pathsep.join([
                os.path.dirname(bringup_pkg),
                os.path.dirname(description_share),
            ]),
        },
        output="screen",
    )

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

    bridge_arguments = [_bridge_argument(bridge, namespace) for bridge in config["bridges"]]
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{
            "override_timestamps_with_wall_time": override_timestamps_with_wall_time,
            "expand_gz_topic_names": False,
        }],
        arguments=bridge_arguments,
    )

    actions = [gazebo, spawn_robot, bridge]

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
                    "world_name": world_name,
                    "pose_topic": _absolute_topic("", f"world/{world_name}/pose/info"),
                    "navsat_topic": gnss_config["navsat_topic"],
                    "gpsfix_topic": gnss_config["gpsfix_topic"],
                    "odometry_topic": gnss_config["odometry_topic"],
                    "status_marker_topic": gnss_config["status_marker_topic"],
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
        DeclareLaunchArgument("enable_gnss", default_value="true"),
        DeclareLaunchArgument("enable_imu", default_value="true"),
        DeclareLaunchArgument("enable_depth_camera", default_value="true"),
        DeclareLaunchArgument("override_timestamps_with_wall_time", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
