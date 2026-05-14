"""Launch the vehicle_can_node with a vehicle-specific YAML config."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    schema_file = LaunchConfiguration("schema_file").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)

    # Layer order (later entries win):
    #   1. launch-argument defaults
    #   2. schema_file   (if provided)
    #   3. config_file
    #   4. dbc_file      (always wins — required per-environment value)
    parameters = [
        {
            "can_topic": LaunchConfiguration("can_topic"),
            "publish_all_signals": LaunchConfiguration("publish_all_signals"),
            "all_signals_topic": LaunchConfiguration("all_signals_topic"),
            "signal_timeout_ms": LaunchConfiguration("signal_timeout_ms"),
            "diagnostics_rate_hz": LaunchConfiguration("diagnostics_rate_hz"),
        }
    ]

    if schema_file:
        parameters.append(schema_file)

    parameters.append(config_file)
    parameters.append({"dbc_file": LaunchConfiguration("dbc_file")})

    node = Node(
        package="vehicle_can_decoder",
        executable="vehicle_can_node",
        name="vehicle_can_node",
        output="screen",
        parameters=parameters,
    )
    return [node]


def generate_launch_description():
    schema_file_arg = DeclareLaunchArgument(
        "schema_file",
        default_value="",
        description=(
            "Absolute path to the vehicle signal schema YAML (e.g. vehicle_schema.yaml). "
            "Optional — omit when using a single all-in-one config file."
        ),
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        description=(
            "Absolute path to the DBC-specific vehicle YAML config file "
            "(e.g. pacmod_v3.yaml or example_vehicle.yaml)."
        ),
    )

    dbc_file_arg = DeclareLaunchArgument(
        "dbc_file",
        description="Absolute path to the DBC file for this environment.",
    )

    can_topic_arg = DeclareLaunchArgument(
        "can_topic",
        default_value="/vehicle/from_can_bus",
        description="ROS 2 topic for incoming can_msgs/Frame messages.",
    )

    publish_all_signals_arg = DeclareLaunchArgument(
        "publish_all_signals",
        default_value="true",
        description="Publish all decoded signals as a single SignalGroup (firehose topic).",
    )

    all_signals_topic_arg = DeclareLaunchArgument(
        "all_signals_topic",
        default_value="/vehicle/decoded_can",
        description="Topic for the firehose SignalGroup (used when publish_all_signals is true).",
    )

    signal_timeout_ms_arg = DeclareLaunchArgument(
        "signal_timeout_ms",
        default_value="500",
        description="Milliseconds after which a signal is considered stale.",
    )

    diagnostics_rate_hz_arg = DeclareLaunchArgument(
        "diagnostics_rate_hz",
        default_value="1.0",
        description="Rate (Hz) at which SignalDiagnostic messages are published.",
    )

    return LaunchDescription(
        [
            schema_file_arg,
            config_file_arg,
            dbc_file_arg,
            can_topic_arg,
            publish_all_signals_arg,
            all_signals_topic_arg,
            signal_timeout_ms_arg,
            diagnostics_rate_hz_arg,
            OpaqueFunction(function=_launch_setup),
        ]
    )
