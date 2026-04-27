"""Launch the vehicle_can_node with a vehicle-specific YAML config."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config_file",
        description="Absolute path to the vehicle YAML config file.",
    )

    dbc_file_arg = DeclareLaunchArgument(
        "dbc_file",
        description="Absolute path to the DBC file for this environment.",
    )

    node = Node(
        package="vehicle_can_decoder",
        executable="vehicle_can_node",
        name="vehicle_can_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"dbc_file": LaunchConfiguration("dbc_file")},
        ],
    )

    return LaunchDescription([config_arg, dbc_file_arg, node])
