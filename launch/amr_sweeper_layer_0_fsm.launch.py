from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
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
      - start_profile (launch argument) sets the supervisor's desired_profile parameter.
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
    start_profile = LaunchConfiguration("start_profile")
    tick_period_ms = LaunchConfiguration("tick_period_ms")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="amr_sweeper",
        description="Top-level namespace for all FSM nodes.",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulated time (Gazebo/Sim).",
    )

    declare_start_profile = DeclareLaunchArgument(
        "start_profile",
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
        get_package_share_directory("amr_sweeper_layer_0_fsm"),
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

    # -------------------------------------------------------------------------
    # Nodes
    # -------------------------------------------------------------------------

    # Supervisor (non-lifecycle)
    supervisor = Node(
        package="amr_sweeper_layer_0_fsm",
        executable="supervisor_node",
        namespace=namespace,
        name="supervisor",
        output="screen",
        parameters=[
            state_params_file,
            {"use_sim_time": use_sim_time},
            {"desired_profile": ParameterValue(start_profile, value_type=int)},
            {"tick_period_ms": ParameterValue(tick_period_ms, value_type=int)},
        ],
    )

    # FSM-state lifecycle nodes (one per FSM-state)
    initializing = Node(
        package="amr_sweeper_layer_0_fsm",
        executable="initializing_state_node",
        namespace=namespace,
        name="initializing_state",
        output="screen",
        parameters=[
            state_params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    idling = Node(
        package="amr_sweeper_layer_0_fsm",
        executable="idling_state_node",
        namespace=namespace,
        name="idling_state",
        output="screen",
        parameters=[
            state_params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    running = Node(
        package="amr_sweeper_layer_0_fsm",
        executable="running_state_node",
        namespace=namespace,
        name="running_state",
        output="screen",
        parameters=[
            state_params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    charging = Node(
        package="amr_sweeper_layer_0_fsm",
        executable="charging_state_node",
        namespace=namespace,
        name="charging_state",
        output="screen",
        parameters=[
            state_params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    fault = Node(
        package="amr_sweeper_layer_0_fsm",
        executable="fault_state_node",
        namespace=namespace,
        name="fault_state",
        output="screen",
        parameters=[
            state_params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # -------------------------------------------------------------------------
    # Launch description
    # -------------------------------------------------------------------------

    return LaunchDescription(
        [
            declare_namespace,
            declare_use_sim_time,
            declare_start_profile,
            declare_tick_period_ms,
            declare_state_params,
            supervisor,
            initializing,
            idling,
            running,
            charging,
            fault,
        ]
    )
