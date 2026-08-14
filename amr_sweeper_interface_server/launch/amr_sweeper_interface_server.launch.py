from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    missions_from_db_directory = LaunchConfiguration("missions_from_db_directory")
    missions_log_directory = LaunchConfiguration("missions_log_directory")
    actual_schedule_log_directory = LaunchConfiguration("actual_schedule_log_directory")
    backend_socket_path = LaunchConfiguration("backend_socket_path")
    http_host = LaunchConfiguration("http_host")
    http_port = LaunchConfiguration("http_port")
    gnss_topic = LaunchConfiguration("gnss_topic")
    battery_topic = LaunchConfiguration("battery_topic")
    fsm_state_topic = LaunchConfiguration("fsm_state_topic")
    fsm_status_topic = LaunchConfiguration("fsm_status_topic")
    fsm_request_service = LaunchConfiguration("fsm_request_service")
    rosout_topic = LaunchConfiguration("rosout_topic")
    site_title = LaunchConfiguration("site_title")
    public_base_url = LaunchConfiguration("public_base_url")
    launch_mqtt_bridge = LaunchConfiguration("launch_mqtt_bridge")
    mqtt_host = LaunchConfiguration("mqtt_host")
    mqtt_port = LaunchConfiguration("mqtt_port")
    vda5050_interface_name = LaunchConfiguration("vda5050_interface_name")
    vda5050_version = LaunchConfiguration("vda5050_version")
    vda5050_manufacturer = LaunchConfiguration("vda5050_manufacturer")
    vda5050_serial_number = LaunchConfiguration("vda5050_serial_number")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("missions_from_db_directory", default_value="missions/database"),
        DeclareLaunchArgument("missions_log_directory", default_value="missions/logs"),
        DeclareLaunchArgument("actual_schedule_log_directory", default_value="missions/logs"),
        DeclareLaunchArgument("backend_socket_path", default_value="/tmp/amr_sweeper_interface_backend.sock"),
        DeclareLaunchArgument("http_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("http_port", default_value="8080"),
        DeclareLaunchArgument("gnss_topic", default_value="gnss/navsat"),
        DeclareLaunchArgument("battery_topic", default_value="battery/battery_state"),
        DeclareLaunchArgument("fsm_state_topic", default_value="fsm/supervisor_node/fsm_state"),
        DeclareLaunchArgument("fsm_status_topic", default_value="fsm/supervisor_node/fsm_status"),
        DeclareLaunchArgument("fsm_request_service", default_value="request_state"),
        DeclareLaunchArgument("rosout_topic", default_value="/rosout"),
        DeclareLaunchArgument("site_title", default_value="AMR-Sweeper"),
        DeclareLaunchArgument("public_base_url", default_value="http://192.168.2.1:8080"),
        DeclareLaunchArgument("launch_mqtt_bridge", default_value="false"),
        DeclareLaunchArgument("mqtt_host", default_value=""),
        DeclareLaunchArgument("mqtt_port", default_value="8883"),
        DeclareLaunchArgument("vda5050_interface_name", default_value="vda5050"),
        DeclareLaunchArgument("vda5050_version", default_value="3.0.0"),
        DeclareLaunchArgument("vda5050_manufacturer", default_value="O-Robotics"),
        DeclareLaunchArgument("vda5050_serial_number", default_value="amr_sweeper"),
        Node(
            package="amr_sweeper_interface_server",
            executable="backend_node.py",
            name="backend_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "backend_socket_path": backend_socket_path,
                "site_title": site_title,
                "public_base_url": public_base_url,
                "missions_from_db_directory": missions_from_db_directory,
                "missions_log_directory": missions_log_directory,
                "actual_schedule_log_directory": actual_schedule_log_directory,
                "list_missions_service": "list_executable_missions",
                "execute_mission_service": "execute_mission",
                "upload_vda5050_mission_service": "upload_vda5050_mission",
                "end_mission_service": "end_mission",
                "fsm_request_service": fsm_request_service,
                "fsm_state_topic": fsm_state_topic,
                "fsm_status_topic": fsm_status_topic,
                "rosout_topic": rosout_topic,
                "gnss_topic": gnss_topic,
                "battery_topic": battery_topic,
            }],
        ),
        Node(
            package="amr_sweeper_interface_server",
            executable="frontend_http_node.py",
            name="frontend_http_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "backend_socket_path": backend_socket_path,
                "http_host": http_host,
                "http_port": http_port,
                "site_title": site_title,
                "public_base_url": public_base_url,
            }],
        ),
        Node(
            package="amr_sweeper_interface_server",
            executable="mqtt_bridge_node.py",
            name="mqtt_bridge_node",
            namespace=namespace,
            output="screen",
            condition=IfCondition(launch_mqtt_bridge),
            parameters=[{
                "backend_socket_path": backend_socket_path,
                "mqtt_host": mqtt_host,
                "mqtt_port": mqtt_port,
                "interface_name": vda5050_interface_name,
                "vda5050_version": vda5050_version,
                "manufacturer": vda5050_manufacturer,
                "serial_number": vda5050_serial_number,
            }],
        ),
    ])
