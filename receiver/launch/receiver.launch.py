from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _config_file() -> Path:
    source_configs = (
        Path.cwd() / "config" / "receiver_params.yaml",
        Path.cwd() / "receiver" / "config" / "receiver_params.yaml",
    )
    for source_config in source_configs:
        if source_config.exists():
            return source_config
    return Path(get_package_share_directory("receiver")) / "config" / "receiver_params.yaml"


def _launch_nodes(context, config_file: Path):
    client_id = LaunchConfiguration("client_id").perform(context).strip()
    if not client_id:
        try:
            client_id = input("receiver MQTT client id: ").strip()
        except (EOFError, KeyboardInterrupt) as exc:
            raise RuntimeError("client_id is required in a non-interactive launch") from exc
    if not client_id:
        raise RuntimeError("client_id must not be empty")

    mqtt_receiver = Node(
        package="receiver",
        executable="mqtt_receiver_node",
        name="mqtt_receiver",
        namespace="receiver",
        parameters=[str(config_file), {"client_id": client_id}],
        output="screen",
        emulate_tty=True,
    )

    video_decoder = Node(
        package="receiver",
        executable="video_decoder_node",
        name="video_decoder",
        namespace="receiver",
        parameters=[str(config_file)],
        output="screen",
        emulate_tty=True,
    )

    shutdown_on_mqtt_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=mqtt_receiver,
            on_exit=[EmitEvent(event=Shutdown(reason="mqtt_receiver exited"))],
        )
    )
    shutdown_on_decoder_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=video_decoder,
            on_exit=[EmitEvent(event=Shutdown(reason="video_decoder exited"))],
        )
    )
    return [shutdown_on_mqtt_exit, shutdown_on_decoder_exit, mqtt_receiver, video_decoder]


def generate_launch_description():
    config_file = _config_file()
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "RCUTILS_CONSOLE_OUTPUT_FORMAT",
                "[{severity}] [{name}]: {message}",
            ),
            DeclareLaunchArgument(
                "client_id",
                default_value="",
                description="Unique MQTT client id; prompts when omitted",
            ),
            LogInfo(msg=f"receiver config: {config_file}"),
            OpaqueFunction(function=_launch_nodes, args=[config_file]),
        ]
    )
