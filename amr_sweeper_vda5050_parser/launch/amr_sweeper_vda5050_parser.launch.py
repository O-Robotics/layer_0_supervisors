from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    mission_path = LaunchConfiguration("mission_path")
    missions_directory = LaunchConfiguration("missions_directory")
    missions_log_directory = LaunchConfiguration("missions_log_directory")
    auto_build_on_start = LaunchConfiguration("auto_build_on_start")
    watch_for_updates = LaunchConfiguration("watch_for_updates")
    build_discovered_missions = LaunchConfiguration("build_discovered_missions")
    config_file = PathJoinSubstitution(
        [FindPackageShare("amr_sweeper_vda5050_parser"), "config", "amr_sweeper_vda5050_parser.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("mission_path", default_value=""),
        DeclareLaunchArgument("missions_directory", default_value="missions/database"),
        DeclareLaunchArgument("missions_log_directory", default_value="missions/logs"),
        DeclareLaunchArgument("auto_build_on_start", default_value="true"),
        DeclareLaunchArgument("watch_for_updates", default_value="true"),
        DeclareLaunchArgument("build_discovered_missions", default_value="false"),
        Node(
            package="amr_sweeper_vda5050_parser",
            executable="vda5050_parser_node",
            namespace=namespace,
            name="vda5050_parser_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "use_sim_time": use_sim_time,
                    "mission_path": mission_path,
                    "missions_directory": missions_directory,
                    "missions_log_directory": missions_log_directory,
                    "auto_build_on_start": auto_build_on_start,
                    "watch_for_updates": watch_for_updates,
                    "build_discovered_missions": build_discovered_missions,
                },
            ],
        ),
    ])
