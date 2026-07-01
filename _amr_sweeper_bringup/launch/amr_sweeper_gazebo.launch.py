import os
import subprocess
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
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
    return (
        f"{ros_topic_name}@{bridge['ros_type_name']}"
        f"{separator}{bridge['gz_type_name']}"
    )


def _render_simulation_robot(
    namespace: str,
    entity_name: str,
) -> str:
    description_pkg = get_package_share_directory("amr_sweeper_description")
    xacro_file = os.path.join(
        description_pkg,
        "urdf",
        "robot",
        "gazebo.urdf.xacro",
    )
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


def _launch_setup(context, *args, **kwargs):
    bringup_pkg = get_package_share_directory("amr_sweeper_bringup")
    config_path = os.path.join(
        bringup_pkg,
        "simulation",
        "config",
        "simulation.yaml",
    )
    with open(config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    simulation_config = config["simulation"]
    namespace = LaunchConfiguration("namespace").perform(context)
    world_name = simulation_config["world_name"]
    entity_name = simulation_config["entity_name"]
    world = os.path.join(bringup_pkg, "simulation", "worlds", simulation_config["world_file"])
    description_share = get_package_share_directory("amr_sweeper_description")
    robot_urdf = _render_simulation_robot(
        namespace,
        entity_name,
    )

    spawn_pose = simulation_config["spawn_pose"]
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
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

    bridge_arguments = [
        _bridge_argument(bridge, namespace)
        for bridge in config["bridges"]
    ]
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{
            "override_timestamps_with_wall_time": True,
            "expand_gz_topic_names": False,
        }],
        arguments=bridge_arguments,
    )

    gps_pub = Node(
        package="amr_sweeper_bringup",
        executable="gz_pose_to_gps",
        name="gnss_node",
        namespace=_child_namespace(namespace, "gnss"),
        output="screen",
        parameters=[{
            "world_name": world_name,
            "navsat_topic": _absolute_topic(namespace, simulation_config["gnss"]["navsat_topic"]),
            "gpsfix_topic": _absolute_topic(namespace, simulation_config["gnss"]["gpsfix_topic"]),
            "odometry_topic": _absolute_topic(namespace, simulation_config["gnss"]["odometry_topic"]),
            "status_marker_topic": _absolute_topic(namespace, simulation_config["gnss"]["status_marker_topic"]),
            "frame_id": simulation_config["gnss"]["frame_id"],
            "spike_at_s": -1.0,
        }],
    )

    return [
        gazebo,
        spawn_robot,
        bridge,
        gps_pub,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("enable_gnss", default_value="true"),
        DeclareLaunchArgument("enable_imu", default_value="true"),
        DeclareLaunchArgument("enable_depth_camera", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
