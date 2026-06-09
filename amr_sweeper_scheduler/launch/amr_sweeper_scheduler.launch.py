from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    schedule_ics_path = LaunchConfiguration("schedule_ics_path")
    missions_directory = LaunchConfiguration("missions_directory")
    default_schedule_filename = LaunchConfiguration("default_schedule_filename")
    mission_file_extension = LaunchConfiguration("mission_file_extension")
    robot_id = LaunchConfiguration("robot_id")
    mission_executor_execute_service = LaunchConfiguration("mission_executor_execute_service")
    mission_executor_prepare_service = LaunchConfiguration("mission_executor_prepare_service")
    trigger_running_on_work_window = LaunchConfiguration("trigger_running_on_work_window")
    config_file = PathJoinSubstitution(
        [FindPackageShare("amr_sweeper_scheduler"), "config", "amr_sweeper_scheduler.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("schedule_ics_path", default_value=""),
        DeclareLaunchArgument("missions_directory", default_value="missions/database"),
        DeclareLaunchArgument("default_schedule_filename", default_value=""),
        DeclareLaunchArgument("mission_file_extension", default_value=".json"),
        DeclareLaunchArgument("robot_id", default_value=""),
        DeclareLaunchArgument("mission_executor_execute_service", default_value="execute_mission"),
        DeclareLaunchArgument("mission_executor_prepare_service", default_value="prepare_manual_mission"),
        DeclareLaunchArgument("trigger_running_on_work_window", default_value="true"),
        Node(
            package="amr_sweeper_scheduler",
            executable="scheduler_node",
            name="amr_sweeper_scheduler",
            namespace=namespace,
            output="screen",
            parameters=[
                config_file,
                {
                    "use_sim_time": use_sim_time,
                    "schedule_ics_path": schedule_ics_path,
                    "missions_directory": missions_directory,
                    "default_schedule_filename": default_schedule_filename,
                    "mission_file_extension": mission_file_extension,
                    "robot_id": robot_id,
                    "mission_executor_execute_service": mission_executor_execute_service,
                    "mission_executor_prepare_service": mission_executor_prepare_service,
                    "trigger_running_on_work_window": trigger_running_on_work_window,
                },
            ],
        )
    ])
