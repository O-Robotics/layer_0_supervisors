from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    missions_directory = LaunchConfiguration("missions_directory")
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
    use_http_server = LaunchConfiguration("use_http_server")
    http_host = LaunchConfiguration("http_host")
    http_port = LaunchConfiguration("http_port")
    gnss_topic = LaunchConfiguration("gnss_topic")
    battery_topic = LaunchConfiguration("battery_topic")
    fsm_state_topic = LaunchConfiguration("fsm_state_topic")
    fsm_status_topic = LaunchConfiguration("fsm_status_topic")
    site_title = LaunchConfiguration("site_title")
    public_base_url = LaunchConfiguration("public_base_url")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("missions_directory", default_value="src/missions"),
        DeclareLaunchArgument(
            "manual_missions_directory",
            default_value=PathJoinSubstitution(
                [FindPackageShare("amr_sweeper_default_missions"), "missions"]
            ),
        ),
        DeclareLaunchArgument("fsm_request_service", default_value="request_state"),
        DeclareLaunchArgument("schedule_ics_path", default_value=""),
        DeclareLaunchArgument("robot_id", default_value="RBT-01"),
        DeclareLaunchArgument("safety_stop_topic", default_value="safety_msgs/stop"),
        DeclareLaunchArgument("teleop_odometry_topic", default_value="diff_cont/odom"),
        DeclareLaunchArgument("manual_mapping_odometry_topic", default_value="odometry/fused"),
        DeclareLaunchArgument("manual_mission_inactivity_timeout_seconds", default_value="300.0"),
        DeclareLaunchArgument("idling_profile_id", default_value="100"),
        DeclareLaunchArgument("mission_parser_node_name", default_value="mission_parser_node"),
        DeclareLaunchArgument("mission_parser_build_service", default_value="build_current_mission"),
        DeclareLaunchArgument("use_http_server", default_value="true"),
        DeclareLaunchArgument("http_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("http_port", default_value="8080"),
        DeclareLaunchArgument("gnss_topic", default_value="gnss/navsat"),
        DeclareLaunchArgument("battery_topic", default_value="battery_state"),
        DeclareLaunchArgument("fsm_state_topic", default_value="fsm_state"),
        DeclareLaunchArgument("fsm_status_topic", default_value="fsm_status"),
        DeclareLaunchArgument("site_title", default_value="AMR Sweeper Mission Control"),
        DeclareLaunchArgument("public_base_url", default_value="http://192.168.2.5:8080"),
        Node(
            package="amr_sweeper_mission_executor",
            executable="mission_executor_node",
            name="mission_executor_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "missions_directory": missions_directory,
                "manual_missions_directory": manual_missions_directory,
                "fsm_request_service": fsm_request_service,
                "schedule_ics_path": schedule_ics_path,
                "robot_id": robot_id,
                "safety_stop_topic": safety_stop_topic,
                "teleop_odometry_topic": teleop_odometry_topic,
                "manual_mapping_odometry_topic": manual_mapping_odometry_topic,
                "manual_mission_inactivity_timeout_seconds": manual_mission_inactivity_timeout_seconds,
                "idling_profile_id": idling_profile_id,
                "mission_parser_node_name": mission_parser_node_name,
                "mission_parser_build_service": mission_parser_build_service,
            }],
        ),
        Node(
            package="amr_sweeper_mission_executor",
            executable="mission_web_server.py",
            name="mission_web_server",
            namespace=namespace,
            output="screen",
            condition=IfCondition(use_http_server),
            parameters=[{
                "http_host": http_host,
                "http_port": http_port,
                "site_title": site_title,
                "public_base_url": public_base_url,
                "missions_directory": missions_directory,
                "list_missions_service": "list_executable_missions",
                "execute_mission_service": "execute_mission",
                "upload_vda5050_mission_service": "upload_vda5050_mission",
                "end_mission_service": "end_mission",
                "fsm_state_topic": fsm_state_topic,
                "fsm_status_topic": fsm_status_topic,
                "gnss_topic": gnss_topic,
                "battery_topic": battery_topic,
            }],
        ),
    ])
