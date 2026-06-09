from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    console_output_format = "[{severity}] [{time}] [{name}] : {message}"
    runtime_override_arg_names = [
        "missions_directory",
        "auto_build_on_start",
        "watch_for_updates",
        "trigger_running_on_work_window",
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
        "use_amr_sweeper_joystick",
        "use_amr_sweeper_sweeping_controller",
        "use_amr_sweeper_attitude_controller",
        "use_amr_sweeper_collision_detector",
        "use_amr_sweeper_safety_controller",
        "use_joy_node",
        "joy_dev",
        "use_amr_sweeper_visual_odometry",
        "use_amr_sweeper_localization",
        "use_amr_sweeper_mapping",
        "use_amr_sweeper_waypoint_follower",
        "auto_start_mission",
    ]
    """
    Launch the AMR Sweeper FSM supervisor and all FSM-state lifecycle nodes.

    Configuration model:
      - config/state_parameters.yaml:
          - per-state fault block (global safety triggers for that state)
          - per-state profile file path (one file per FSM-state)
      - config/profiles/<state>_profiles.yaml:
          - per-profile process definitions and rules

    NOTE:
      This launch file passes only the state_parameters.yaml as a ROS params file.
      The per-state profile YAML files are referenced (by path) from state_parameters.yaml.

    Startup selection:
      - use_profile (launch argument) sets the supervisor's desired_profile parameter.
      - The supervisor derives the initial FSM-state from the profile band and enters that
        state node using the requested profile.
    """

    # -------------------------------------------------------------------------
    # Launch arguments
    # -------------------------------------------------------------------------

    # Namespace policy: top-level namespace for the robot.
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    state_params_file = LaunchConfiguration("state_params_file")
    use_profile = LaunchConfiguration("use_profile")
    tick_period_ms = LaunchConfiguration("tick_period_ms")
    use_test = LaunchConfiguration("use_test")
    test_output_directory = LaunchConfiguration("test_output_directory")
    runtime_override_args = {name: LaunchConfiguration(name) for name in runtime_override_arg_names}

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="amr_sweeper",
        description="Top-level namespace for all FSM nodes.",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use ROS time from /clock when true.",
    )

    declare_use_profile = DeclareLaunchArgument(
        "use_profile",
        default_value="001",
        description="Startup FSM profile id (default: 001).",
    )

    declare_tick_period_ms = DeclareLaunchArgument(
        "tick_period_ms",
        default_value="100",
        description="Supervisor tick period in milliseconds (default: 100).",
    )

    # Default: package_share/config/state_parameters.yaml
    default_state_params = os.path.join(
        get_package_share_directory("amr_sweeper_fsm"),
        "config",
        "state_parameters.yaml",
    )
    declare_state_params = DeclareLaunchArgument(
        "state_params_file",
        default_value=default_state_params,
        description=(
            "FSM state parameters YAML. Defines per-state fault blocks and default profiles, "
            "and references per-state profile YAML files."
        ),
    )

    declare_use_test = DeclareLaunchArgument(
        "use_test",
        default_value="false",
        description="Enable workspace test-mode launch behavior for FSM-managed processes.",
    )

    declare_test_output_directory = DeclareLaunchArgument(
        "test_output_directory",
        default_value="src/layer_3_navigation/tests",
        description="Shared generic artifact directory used by test-mode FSM-managed processes.",
    )

    runtime_override_declarations = [
        DeclareLaunchArgument(name, default_value="")
        for name in runtime_override_arg_names
    ]

    runtime_override_parameters = [
        {f"runtime.{name}": value}
        for name, value in runtime_override_args.items()
    ]

    # -------------------------------------------------------------------------
    # Nodes
    # -------------------------------------------------------------------------

    # Supervisor (non-lifecycle)
    supervisor = Node(
        package="amr_sweeper_fsm",
        executable="supervisor_node",
        namespace=namespace,
        name="supervisor",
        output="screen",
        parameters=[
            state_params_file,
            {"use_sim_time": use_sim_time},
            {"desired_profile": ParameterValue(use_profile, value_type=int)},
            {"tick_period_ms": ParameterValue(tick_period_ms, value_type=int)},
            {"runtime.use_test": use_test},
            {"runtime.test_output_directory": test_output_directory},
            *runtime_override_parameters,
        ],
    )

    # FSM-state lifecycle nodes (one per FSM-state)
    initializing = Node(
        package="amr_sweeper_fsm",
        executable="initializing_state_node",
        namespace=namespace,
        name="initializing_state",
        output="screen",
        parameters=[
            state_params_file,
            {"use_sim_time": use_sim_time},
            {"runtime.use_test": use_test},
            {"runtime.test_output_directory": test_output_directory},
            *runtime_override_parameters,
        ],
    )

    idling = Node(
        package="amr_sweeper_fsm",
        executable="idling_state_node",
        namespace=namespace,
        name="idling_state",
        output="screen",
        parameters=[
            state_params_file,
            {"use_sim_time": use_sim_time},
            {"runtime.use_test": use_test},
            {"runtime.test_output_directory": test_output_directory},
            *runtime_override_parameters,
        ],
    )

    running = Node(
        package="amr_sweeper_fsm",
        executable="running_state_node",
        namespace=namespace,
        name="running_state",
        output="screen",
        parameters=[
            state_params_file,
            {"use_sim_time": use_sim_time},
            {"runtime.use_test": use_test},
            {"runtime.test_output_directory": test_output_directory},
            *runtime_override_parameters,
        ],
    )

    charging = Node(
        package="amr_sweeper_fsm",
        executable="charging_state_node",
        namespace=namespace,
        name="charging_state",
        output="screen",
        parameters=[
            state_params_file,
            {"use_sim_time": use_sim_time},
            {"runtime.use_test": use_test},
            {"runtime.test_output_directory": test_output_directory},
            *runtime_override_parameters,
        ],
    )

    fault = Node(
        package="amr_sweeper_fsm",
        executable="fault_state_node",
        namespace=namespace,
        name="fault_state",
        output="screen",
        parameters=[
            state_params_file,
            {"use_sim_time": use_sim_time},
            {"runtime.use_test": use_test},
            {"runtime.test_output_directory": test_output_directory},
            *runtime_override_parameters,
        ],
    )

    # -------------------------------------------------------------------------
    # Launch description
    # -------------------------------------------------------------------------

    return LaunchDescription(
        [
            SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            SetEnvironmentVariable("RCUTILS_CONSOLE_OUTPUT_FORMAT", console_output_format),
            declare_namespace,
            declare_use_sim_time,
            declare_use_profile,
            declare_tick_period_ms,
            declare_state_params,
            declare_use_test,
            declare_test_output_directory,
            *runtime_override_declarations,
            supervisor,
            initializing,
            idling,
            running,
            charging,
            fault,
        ]
    )
