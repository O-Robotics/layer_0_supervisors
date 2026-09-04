#
# Copyright 2026 O-Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    mission_path = LaunchConfiguration("mission_path")
    missions_directory = LaunchConfiguration("missions_directory")
    missions_log_directory = LaunchConfiguration("missions_log_directory")
    maps_directory = LaunchConfiguration("maps_directory")
    launch_gaussian_splat_builder_node = LaunchConfiguration("launch_gaussian_splat_builder_node")
    fsm_state_topic = LaunchConfiguration("fsm_state_topic")
    fsm_status_topic = LaunchConfiguration("fsm_status_topic")
    auto_build_on_start = LaunchConfiguration("auto_build_on_start")
    watch_for_updates = LaunchConfiguration("watch_for_updates")
    build_discovered_missions = LaunchConfiguration("build_discovered_missions")
    mission_projection_use_first_polygon_vertex_as_origin = LaunchConfiguration(
        "mission_projection_use_first_polygon_vertex_as_origin"
    )
    mission_projection_origin_latitude = LaunchConfiguration("mission_projection_origin_latitude")
    mission_projection_origin_longitude = LaunchConfiguration("mission_projection_origin_longitude")
    mission_projection_origin_altitude = LaunchConfiguration("mission_projection_origin_altitude")
    config_file = PathJoinSubstitution(
        [FindPackageShare("amr_sweeper_mission_builder"), "config", "amr_sweeper_mission_builder.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("mission_path", default_value=""),
        DeclareLaunchArgument("missions_directory", default_value="missions/database"),
        DeclareLaunchArgument("missions_log_directory", default_value="missions/logs"),
        DeclareLaunchArgument("maps_directory", default_value="missions/maps"),
        DeclareLaunchArgument("launch_gaussian_splat_builder_node", default_value="false"),
        DeclareLaunchArgument("fsm_state_topic", default_value="fsm/supervisor_node/fsm_state"),
        DeclareLaunchArgument("fsm_status_topic", default_value="fsm/supervisor_node/fsm_status"),
        DeclareLaunchArgument("auto_build_on_start", default_value="true"),
        DeclareLaunchArgument("watch_for_updates", default_value="true"),
        DeclareLaunchArgument("build_discovered_missions", default_value="false"),
        DeclareLaunchArgument("mission_projection_use_first_polygon_vertex_as_origin", default_value="true"),
        DeclareLaunchArgument("mission_projection_origin_latitude", default_value="0.0"),
        DeclareLaunchArgument("mission_projection_origin_longitude", default_value="0.0"),
        DeclareLaunchArgument("mission_projection_origin_altitude", default_value="0.0"),
        Node(
            package="amr_sweeper_mission_builder",
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
                    "mission_projection_use_first_polygon_vertex_as_origin":
                        mission_projection_use_first_polygon_vertex_as_origin,
                    "mission_projection_origin_latitude": mission_projection_origin_latitude,
                    "mission_projection_origin_longitude": mission_projection_origin_longitude,
                    "mission_projection_origin_altitude": mission_projection_origin_altitude,
                },
            ],
        ),
        Node(
            package="amr_sweeper_mission_builder",
            executable="gaussian_splat_builder_node",
            namespace=namespace,
            name="gaussian_splat_builder_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "use_sim_time": use_sim_time,
                    "missions_log_directory": missions_log_directory,
                    "maps_directory": maps_directory,
                    "fsm_state_topic": fsm_state_topic,
                    "fsm_status_topic": fsm_status_topic,
                },
            ],
            condition=IfCondition(launch_gaussian_splat_builder_node),
        ),
    ])
