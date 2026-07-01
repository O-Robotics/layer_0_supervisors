from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_simulation = LaunchConfiguration("use_simulation")
    missions_directory = LaunchConfiguration("missions_directory")
    missions_log_directory = LaunchConfiguration("missions_log_directory")
    manual_missions_directory = LaunchConfiguration("manual_missions_directory")
    fsm_request_service = LaunchConfiguration("fsm_request_service")
    schedule_ics_path = LaunchConfiguration("schedule_ics_path")
    robot_id = LaunchConfiguration("robot_id")
    safety_stop_topic = LaunchConfiguration("safety_stop_topic")
    teleop_odometry_topic = LaunchConfiguration("teleop_odometry_topic")
    manual_mapping_odometry_topic = LaunchConfiguration("manual_mapping_odometry_topic")
    record_mission_rosbag = LaunchConfiguration("record_mission_rosbag")
    rosbag_topics_file = LaunchConfiguration("rosbag_topics_file")
    manual_mission_inactivity_timeout_seconds = LaunchConfiguration("manual_mission_inactivity_timeout_seconds")
    idling_profile_id = LaunchConfiguration("idling_profile_id")
    mission_parser_node_name = LaunchConfiguration("mission_parser_node_name")
    mission_parser_build_service = LaunchConfiguration("mission_parser_build_service")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("use_simulation", default_value="false"),
        DeclareLaunchArgument("missions_directory", default_value="missions/database"),
        DeclareLaunchArgument("missions_log_directory", default_value="missions/logs"),
        DeclareLaunchArgument(
            "manual_missions_directory",
            default_value=PathJoinSubstitution(
                [FindPackageShare("amr_sweeper_navigation"), "missions"]
            ),
        ),
        DeclareLaunchArgument("fsm_request_service", default_value="request_state"),
        DeclareLaunchArgument("schedule_ics_path", default_value=""),
        DeclareLaunchArgument("robot_id", default_value="RBT-01"),
        DeclareLaunchArgument("safety_stop_topic", default_value="safety_msgs/stop"),
        DeclareLaunchArgument("teleop_odometry_topic", default_value="drive_controller/odom"),
        DeclareLaunchArgument("manual_mapping_odometry_topic", default_value="localization/odometry_fused"),
        DeclareLaunchArgument("record_mission_rosbag", default_value="false"),
        DeclareLaunchArgument(
            "rosbag_topics_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("amr_sweeper_mission_executor"), "config", "record_mission_rosbag.yaml"]
            ),
        ),
        DeclareLaunchArgument("manual_mission_inactivity_timeout_seconds", default_value="300.0"),
        DeclareLaunchArgument("idling_profile_id", default_value="101"),
        DeclareLaunchArgument("mission_parser_node_name", default_value="vda5050_parser_node"),
        DeclareLaunchArgument("mission_parser_build_service", default_value="build_current_mission"),
        Node(
            package="amr_sweeper_mission_executor",
            executable="mission_executor_node",
            name="mission_executor_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "missions_directory": missions_directory,
                "missions_log_directory": missions_log_directory,
                "manual_missions_directory": manual_missions_directory,
                "use_simulation": use_simulation,
                "fsm_request_service": fsm_request_service,
                "schedule_ics_path": schedule_ics_path,
                "robot_id": robot_id,
                "safety_stop_topic": safety_stop_topic,
                "teleop_odometry_topic": teleop_odometry_topic,
                "manual_mapping_odometry_topic": manual_mapping_odometry_topic,
                "record_mission_rosbag": record_mission_rosbag,
                "rosbag_topics_file": rosbag_topics_file,
                "manual_mission_inactivity_timeout_seconds": manual_mission_inactivity_timeout_seconds,
                "idling_profile_id": idling_profile_id,
                "mission_parser_node_name": mission_parser_node_name,
                "mission_parser_build_service": mission_parser_build_service,
            }],
        ),
    ])
