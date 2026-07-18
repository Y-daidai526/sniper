from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _config_file() -> Path:
    source_configs = (
        Path.cwd() / "config" / "sender_params.yaml",
        Path.cwd() / "sender" / "config" / "sender_params.yaml",
    )
    for source_config in source_configs:
        if source_config.exists():
            return source_config
    return Path(get_package_share_directory("sender")) / "config" / "sender_params.yaml"


def generate_launch_description():
    config_file = _config_file()
    show = LaunchConfiguration("show")
    show_argument = DeclareLaunchArgument(
        "show",
        default_value="false",
        description="Show the sender video preview window",
    )
    log_format = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_OUTPUT_FORMAT",
        "[{severity}] [{name}]: {message}",
    )

    node_specs = (
        ("sender", "camera_encoder_node"),
        ("sender", "rm_frame_encoder_node"),
        ("sender", "serial_sender_node"),
        ("sender", "mqtt_broker_node"),
        ("sender", "mqtt_sender_node"),
    )
    nodes = [
        Node(
            package=package,
            executable=executable,
            namespace="sender",
            parameters=[
                str(config_file),
                *(
                    [{"show": ParameterValue(show, value_type=bool)}]
                    if executable == "camera_encoder_node"
                    else []
                ),
            ],
            output="screen",
            emulate_tty=True,
        )
        for package, executable in node_specs
    ]
    exit_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[Shutdown(reason=f"{executable} exited")],
            )
        )
        for node, (_, executable) in zip(nodes, node_specs)
    ]

    return LaunchDescription(
        [
            show_argument,
            log_format,
            LogInfo(msg=f"sender config: {config_file}"),
            *exit_handlers,
            *nodes,
        ]
    )
