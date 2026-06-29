"""Launch the AMR Sweeper layer 0 supervisor stack from one bringup entrypoint."""

import os
import re
import tempfile
from datetime import datetime, timezone

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def _launch_file(package_name: str, launch_file_name: str):
    return PathJoinSubstitution([
        FindPackageShare(package_name),
        "launch",
        launch_file_name,
    ])


def _resolve_workspace_path(configured_path: str) -> str:
    if os.path.isabs(configured_path):
        return configured_path

    workspace_relative = os.path.join(os.getcwd(), configured_path)
    if os.path.exists(workspace_relative):
        return workspace_relative
    return configured_path


def _load_rosbag_topics(topics_file: str) -> list[str]:
    resolved_topics_file = _resolve_workspace_path(topics_file)
    if not os.path.exists(resolved_topics_file):
        raise FileNotFoundError(f"Rosbag topics file does not exist: {resolved_topics_file}")

    topics: list[str] = []
    with open(resolved_topics_file, encoding="utf-8") as stream:
        for raw_line in stream:
            trimmed = raw_line.strip()
            if not trimmed or trimmed.startswith("#"):
                continue
            if trimmed == "topics:" or trimmed.startswith("topics:"):
                continue
            if not trimmed.startswith("- "):
                continue
            topic = trimmed[2:].strip()
            if topic.startswith("/"):
                topics.append(topic)
    return topics


def _build_rosbag_regex(topics: list[str]) -> str:
    escaped_topics = [re.escape(topic) for topic in topics]
    return "^(" + "|".join(escaped_topics) + ")$"


def _write_runtime_rosbag_qos_overrides(rosbag_output_directory: str) -> str:
    overrides_path = os.path.join(
        rosbag_output_directory,
        "rosbag_runtime_qos_overrides.yaml",
    )
    with open(overrides_path, "w", encoding="utf-8") as stream:
        stream.write(
            "/amr_sweeper/depth_camera/scan:\n"
            "  reliability: best_effort\n"
            "  history: keep_last\n"
            "  depth: 5\n"
        )
    return overrides_path


def _start_bringup_rosbag(context, *args, **kwargs):
    del args, kwargs

    record_rosbag = LaunchConfiguration("record_rosbag").perform(context).strip().lower()
    if record_rosbag != "true":
        return []

    missions_log_directory = _resolve_workspace_path(
        LaunchConfiguration("missions_log_directory").perform(context)
    )
    use_profile = LaunchConfiguration("use_profile").perform(context)
    rosbag_topics_file = LaunchConfiguration("rosbag_topics_file").perform(context)
    mission_id = f"amr_sweeper_bringup_profile_{use_profile}"
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    rosbag_output_directory = os.path.join(
        missions_log_directory,
        mission_id,
        timestamp,
        "artifacts",
        "rosbag",
    )
    os.makedirs(rosbag_output_directory, exist_ok=True)

    topics = _load_rosbag_topics(rosbag_topics_file)
    if not topics:
        return [
            LogInfo(
                msg=(
                    "[amr_sweeper_bringup] record_rosbag requested but no topics were "
                    f"loaded from {rosbag_topics_file}"
                )
            )
        ]

    rosbag_regex = _build_rosbag_regex(topics)
    rosbag_qos_overrides_path = _write_runtime_rosbag_qos_overrides(rosbag_output_directory)
    return [
        LogInfo(msg=f"[amr_sweeper_bringup] Recording rosbag under {rosbag_output_directory}"),
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "--regex",
                rosbag_regex,
                "--qos-profile-overrides-path",
                rosbag_qos_overrides_path,
                "-o",
                rosbag_output_directory,
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    console_output_format = "[{severity}] [{time}] [{name}] : {message}"
    ros_log_dir = tempfile.mkdtemp(prefix="amr_sweeper_bringup_roslog_")
    fsm_override_arg_names = [
        "use_sim_time",
        "use_simulation",
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
        "joy_dev",
        "use_amr_sweeper_visual_odometry",
        "use_amr_sweeper_localization",
        "use_amr_sweeper_mapping",
        "use_amr_sweeper_navigation",
        "auto_start_mission",
    ]
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    state_params_file = LaunchConfiguration("state_params_file")
    test_output_directory = LaunchConfiguration("test_output_directory")
    use_profile = LaunchConfiguration("use_profile")
    use_simulation = LaunchConfiguration("use_simulation")
    tick_period_ms = LaunchConfiguration("tick_period_ms")
    missions_from_db_directory = LaunchConfiguration("missions_from_db_directory")
    missions_log_directory = LaunchConfiguration("missions_log_directory")
    manual_missions_directory = LaunchConfiguration("manual_missions_directory")
    fsm_request_service = LaunchConfiguration("fsm_request_service")
    schedule_ics_path = LaunchConfiguration("schedule_ics_path")
    robot_id = LaunchConfiguration("robot_id")
    safety_stop_topic = LaunchConfiguration("safety_stop_topic")
    teleop_odometry_topic = LaunchConfiguration("teleop_odometry_topic")
    manual_mapping_odometry_topic = LaunchConfiguration("manual_mapping_odometry_topic")
    manual_mission_inactivity_timeout_seconds = LaunchConfiguration("manual_mission_inactivity_timeout_seconds")
    idling_profile_id = LaunchConfiguration("idling_profile_id")
    mission_parser_node_name = LaunchConfiguration("mission_parser_node_name")
    mission_parser_build_service = LaunchConfiguration("mission_parser_build_service")
    default_schedule_filename = LaunchConfiguration("default_schedule_filename")
    use_test = LaunchConfiguration("use_test")
    test_schedule_ics_path = LaunchConfiguration("test_schedule_ics_path")
    record_rosbag = LaunchConfiguration("record_rosbag")
    rosbag_topics_file = LaunchConfiguration("rosbag_topics_file")
    mission_file_extension = LaunchConfiguration("mission_file_extension")
    mission_executor_execute_service = LaunchConfiguration("mission_executor_execute_service")
    mission_executor_prepare_service = LaunchConfiguration("mission_executor_prepare_service")
    trigger_running_on_work_window = LaunchConfiguration("trigger_running_on_work_window")
    launch_scheduler = LaunchConfiguration("launch_scheduler")
    launch_vda5050_parser = LaunchConfiguration("launch_vda5050_parser")
    mission_path = LaunchConfiguration("mission_path")
    auto_build_on_start = LaunchConfiguration("auto_build_on_start")
    watch_for_updates = LaunchConfiguration("watch_for_updates")
    http_host = LaunchConfiguration("http_host")
    http_port = LaunchConfiguration("http_port")
    gnss_topic = LaunchConfiguration("gnss_topic")
    battery_topic = LaunchConfiguration("battery_topic")
    fsm_state_topic = LaunchConfiguration("fsm_state_topic")
    fsm_status_topic = LaunchConfiguration("fsm_status_topic")
    site_title = LaunchConfiguration("site_title")
    public_base_url = LaunchConfiguration("public_base_url")
    fsm_override_args = {name: LaunchConfiguration(name) for name in fsm_override_arg_names}
    default_state_params_file = PathJoinSubstitution([
        FindPackageShare("amr_sweeper_fsm"),
        "config",
        "state_parameters.yaml",
    ])
    default_manual_missions_directory = PathJoinSubstitution([
        FindPackageShare("amr_sweeper_navigation"),
        "missions",
    ])
    effective_schedule_ics_path = PythonExpression([
        '"', test_schedule_ics_path, '" if ("', use_test, '" == "true" and "', schedule_ics_path,
        '" == "") else "', schedule_ics_path, '"',
    ])

    extra_fsm_override_declarations = [
        DeclareLaunchArgument(name, default_value="")
        for name in fsm_override_arg_names
    ]

    fsm_launch_arguments = {
        "namespace": namespace,
        "use_simulation": use_simulation,
        "use_sim_time": use_sim_time,
        "use_profile": use_profile,
        "tick_period_ms": tick_period_ms,
        "state_params_file": state_params_file,
        "use_test": use_test,
        "test_output_directory": test_output_directory,
        "missions_directory": missions_from_db_directory,
        "auto_build_on_start": auto_build_on_start,
        "watch_for_updates": watch_for_updates,
        "trigger_running_on_work_window": trigger_running_on_work_window,
    }
    fsm_launch_arguments.update(fsm_override_args)

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOG_DIR", ros_log_dir),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        SetEnvironmentVariable("RCUTILS_CONSOLE_OUTPUT_FORMAT", console_output_format),
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("use_simulation", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("state_params_file", default_value=default_state_params_file),
        DeclareLaunchArgument("test_output_directory", default_value="src/layer_3_navigation/tests"),
        DeclareLaunchArgument("use_profile", default_value="001"),
        DeclareLaunchArgument("tick_period_ms", default_value="100"),
        DeclareLaunchArgument("missions_from_db_directory", default_value="missions/database"),
        DeclareLaunchArgument("missions_log_directory", default_value="missions/logs"),
        DeclareLaunchArgument("manual_missions_directory", default_value=default_manual_missions_directory),
        DeclareLaunchArgument("fsm_request_service", default_value="request_state"),
        DeclareLaunchArgument("schedule_ics_path", default_value=""),
        DeclareLaunchArgument("robot_id", default_value="RBT-01"),
        DeclareLaunchArgument("safety_stop_topic", default_value="safety_msgs/stop"),
        DeclareLaunchArgument("teleop_odometry_topic", default_value="drive_controller/odom"),
        DeclareLaunchArgument("manual_mapping_odometry_topic", default_value="localization/odometry_fused"),
        DeclareLaunchArgument("manual_mission_inactivity_timeout_seconds", default_value="300.0"),
        DeclareLaunchArgument("idling_profile_id", default_value="101"),
        DeclareLaunchArgument("mission_parser_node_name", default_value="vda5050_parser_node"),
        DeclareLaunchArgument("mission_parser_build_service", default_value="build_current_mission"),
        DeclareLaunchArgument("default_schedule_filename", default_value=""),
        DeclareLaunchArgument("use_test", default_value="false"),
        DeclareLaunchArgument(
            "test_schedule_ics_path",
            default_value="src/layer_0_supervisors/tests/schedule_20260000T000000Z.ics",
        ),
        DeclareLaunchArgument("record_rosbag", default_value="false"),
        DeclareLaunchArgument(
            "rosbag_topics_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("amr_sweeper_mission_executor"), "config", "record_rosbag.yaml"]
            ),
        ),
        DeclareLaunchArgument("mission_file_extension", default_value=".json"),
        DeclareLaunchArgument("mission_executor_execute_service", default_value="execute_mission"),
        DeclareLaunchArgument("mission_executor_prepare_service", default_value="prepare_manual_mission"),
        DeclareLaunchArgument("trigger_running_on_work_window", default_value="true"),
        DeclareLaunchArgument("launch_scheduler", default_value="false"),
        DeclareLaunchArgument("launch_vda5050_parser", default_value="false"),
        DeclareLaunchArgument("mission_path", default_value=""),
        DeclareLaunchArgument("auto_build_on_start", default_value="true"),
        DeclareLaunchArgument("watch_for_updates", default_value="true"),
        DeclareLaunchArgument("http_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("http_port", default_value="8080"),
        DeclareLaunchArgument("gnss_topic", default_value="gnss/navsat"),
        DeclareLaunchArgument("battery_topic", default_value="battery/battery_state"),
        DeclareLaunchArgument("fsm_state_topic", default_value="fsm/supervisor_node/fsm_state"),
        DeclareLaunchArgument("fsm_status_topic", default_value="fsm/supervisor_node/fsm_status"),
        DeclareLaunchArgument("site_title", default_value="AMR Sweeper Mission Control"),
        DeclareLaunchArgument("public_base_url", default_value="http://192.168.2.1:8080"),
        *extra_fsm_override_declarations,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("amr_sweeper_fsm", "amr_sweeper_fsm.launch.py")),
            launch_arguments=fsm_launch_arguments.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("amr_sweeper_mission_executor", "mission_executor.launch.py")),
            launch_arguments={
                "namespace": namespace,
                "use_simulation": use_simulation,
                "missions_directory": missions_from_db_directory,
                "missions_log_directory": missions_log_directory,
                "manual_missions_directory": manual_missions_directory,
                "fsm_request_service": fsm_request_service,
                "schedule_ics_path": effective_schedule_ics_path,
                "robot_id": robot_id,
                "safety_stop_topic": safety_stop_topic,
                "teleop_odometry_topic": teleop_odometry_topic,
                "manual_mapping_odometry_topic": manual_mapping_odometry_topic,
                "rosbag_topics_file": rosbag_topics_file,
                "manual_mission_inactivity_timeout_seconds": manual_mission_inactivity_timeout_seconds,
                "idling_profile_id": idling_profile_id,
                "mission_parser_node_name": mission_parser_node_name,
                "mission_parser_build_service": mission_parser_build_service,
            }.items(),
        ),
        OpaqueFunction(function=_start_bringup_rosbag, condition=IfCondition(record_rosbag)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("amr_sweeper_scheduler", "amr_sweeper_scheduler.launch.py")),
            launch_arguments={
                "namespace": namespace,
                "use_sim_time": use_sim_time,
                "schedule_ics_path": effective_schedule_ics_path,
                "missions_directory": missions_from_db_directory,
                "default_schedule_filename": default_schedule_filename,
                "mission_file_extension": mission_file_extension,
                "mission_executor_execute_service": mission_executor_execute_service,
                "mission_executor_prepare_service": mission_executor_prepare_service,
                "trigger_running_on_work_window": trigger_running_on_work_window,
            }.items(),
            condition=IfCondition(launch_scheduler),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("amr_sweeper_vda5050_parser", "amr_sweeper_vda5050_parser.launch.py")),
            launch_arguments={
                "namespace": namespace,
                "use_sim_time": use_sim_time,
                "mission_path": mission_path,
                "missions_directory": missions_from_db_directory,
                "missions_log_directory": missions_log_directory,
                "auto_build_on_start": auto_build_on_start,
                "watch_for_updates": watch_for_updates,
            }.items(),
            condition=IfCondition(launch_vda5050_parser),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("amr_sweeper_interface_server", "amr_sweeper_interface_server.launch.py")),
            launch_arguments={
                "namespace": namespace,
                "missions_from_db_directory": missions_from_db_directory,
                "missions_log_directory": missions_log_directory,
                "http_host": http_host,
                "http_port": http_port,
                "gnss_topic": gnss_topic,
                "battery_topic": battery_topic,
                "fsm_request_service": fsm_request_service,
                "fsm_state_topic": fsm_state_topic,
                "fsm_status_topic": fsm_status_topic,
                "site_title": site_title,
                "public_base_url": public_base_url,
            }.items(),
        ),
    ])
