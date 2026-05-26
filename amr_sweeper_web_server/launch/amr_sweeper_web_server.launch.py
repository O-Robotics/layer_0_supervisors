from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    missions_log_directory = LaunchConfiguration("missions_log_directory")
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
        DeclareLaunchArgument("missions_log_directory", default_value="src/missions_log"),
        DeclareLaunchArgument("http_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("http_port", default_value="8080"),
        DeclareLaunchArgument("gnss_topic", default_value="gnss/navsat"),
        DeclareLaunchArgument("battery_topic", default_value="battery_state"),
        DeclareLaunchArgument("fsm_state_topic", default_value="fsm_state"),
        DeclareLaunchArgument("fsm_status_topic", default_value="fsm_status"),
        DeclareLaunchArgument("site_title", default_value="AMR Sweeper Mission Control"),
        DeclareLaunchArgument("public_base_url", default_value="http://192.168.2.1:8080"),
        Node(
            package="amr_sweeper_web_server",
            executable="web_server_node.py",
            name="web_server_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "http_host": http_host,
                "http_port": http_port,
                "site_title": site_title,
                "public_base_url": public_base_url,
                "missions_log_directory": missions_log_directory,
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
