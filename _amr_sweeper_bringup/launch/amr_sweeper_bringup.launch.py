"""Launch the AMR Sweeper layer 0 supervisor stack from one bringup entrypoint."""

import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
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


def generate_launch_description():
    console_output_format = "[{severity}] [{time}] [{name}] : {message}"
    ros_log_dir = tempfile.mkdtemp(prefix="amr_sweeper_bringup_roslog_")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    state_params_file = LaunchConfiguration("state_params_file")
    test_output_directory = LaunchConfiguration("test_output_directory")
    use_profile = LaunchConfiguration("use_profile")
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
    default_state_params_file = PathJoinSubstitution([
        FindPackageShare("amr_sweeper_fsm"),
        "config",
        "state_parameters.yaml",
    ])
    default_manual_missions_directory = PathJoinSubstitution([
        FindPackageShare("amr_sweeper_default_missions"),
        "missions",
    ])
    effective_schedule_ics_path = PythonExpression([
        '"', test_schedule_ics_path, '" if ("', use_test, '" == "true" and "', schedule_ics_path,
        '" == "") else "', schedule_ics_path, '"',
    ])

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOG_DIR", ros_log_dir),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        SetEnvironmentVariable("RCUTILS_CONSOLE_OUTPUT_FORMAT", console_output_format),
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
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
        DeclareLaunchArgument("manual_mapping_odometry_topic", default_value="odometry/fused"),
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
        DeclareLaunchArgument("fsm_state_topic", default_value="fsm_state"),
        DeclareLaunchArgument("fsm_status_topic", default_value="fsm_status"),
        DeclareLaunchArgument("site_title", default_value="AMR Sweeper Mission Control"),
        DeclareLaunchArgument("public_base_url", default_value="http://192.168.2.1:8080"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("amr_sweeper_fsm", "amr_sweeper_fsm.launch.py")),
            launch_arguments={
                "namespace": namespace,
                "use_sim_time": use_sim_time,
                "use_profile": use_profile,
                "tick_period_ms": tick_period_ms,
                "state_params_file": state_params_file,
                "use_test": use_test,
                "test_output_directory": test_output_directory,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("amr_sweeper_mission_executor", "mission_executor.launch.py")),
            launch_arguments={
                "namespace": namespace,
                "missions_directory": missions_from_db_directory,
                "missions_log_directory": missions_log_directory,
                "manual_missions_directory": manual_missions_directory,
                "fsm_request_service": fsm_request_service,
                "schedule_ics_path": effective_schedule_ics_path,
                "robot_id": robot_id,
                "safety_stop_topic": safety_stop_topic,
                "teleop_odometry_topic": teleop_odometry_topic,
                "manual_mapping_odometry_topic": manual_mapping_odometry_topic,
                "manual_mission_inactivity_timeout_seconds": manual_mission_inactivity_timeout_seconds,
                "idling_profile_id": idling_profile_id,
                "mission_parser_node_name": mission_parser_node_name,
                "mission_parser_build_service": mission_parser_build_service,
            }.items(),
        ),
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
