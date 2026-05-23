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
    mission_builder_node_name = LaunchConfiguration("mission_builder_node_name")
    mission_builder_build_service = LaunchConfiguration("mission_builder_build_service")
    fsm_request_service = LaunchConfiguration("fsm_request_service")
    active_costmap_output_basename = LaunchConfiguration("active_costmap_output_basename")
    active_route_output_basename = LaunchConfiguration("active_route_output_basename")
    active_execution_pointer_filename = LaunchConfiguration("active_execution_pointer_filename")
    running_profile_id = LaunchConfiguration("running_profile_id")
    trigger_running_on_work_window = LaunchConfiguration("trigger_running_on_work_window")
    config_file = PathJoinSubstitution(
        [FindPackageShare("amr_sweeper_scheduler"), "config", "scheduler.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("schedule_ics_path", default_value=""),
        DeclareLaunchArgument("missions_directory", default_value="src/missions"),
        DeclareLaunchArgument("default_schedule_filename", default_value=""),
        DeclareLaunchArgument("mission_file_extension", default_value=".json"),
        DeclareLaunchArgument("robot_id", default_value="RBT-01"),
        DeclareLaunchArgument("mission_builder_node_name", default_value="mission_builder_node"),
        DeclareLaunchArgument("mission_builder_build_service", default_value="build_current_mission"),
        DeclareLaunchArgument("fsm_request_service", default_value="request_state"),
        DeclareLaunchArgument("active_costmap_output_basename", default_value="global_costmap"),
        DeclareLaunchArgument("active_route_output_basename", default_value="active_mission_path"),
        DeclareLaunchArgument("active_execution_pointer_filename", default_value="active_execution.json"),
        DeclareLaunchArgument("running_profile_id", default_value="201"),
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
                    "mission_builder_node_name": mission_builder_node_name,
                    "mission_builder_build_service": mission_builder_build_service,
                    "fsm_request_service": fsm_request_service,
                    "active_costmap_output_basename": active_costmap_output_basename,
                    "active_route_output_basename": active_route_output_basename,
                    "active_execution_pointer_filename": active_execution_pointer_filename,
                    "running_profile_id": running_profile_id,
                    "trigger_running_on_work_window": trigger_running_on_work_window,
                },
            ],
        )
    ])
