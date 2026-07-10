from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo, SetEnvironmentVariable
from launch_ros.actions import Node


def _config_file() -> Path:
    source_config = Path.cwd() / "receiver" / "config" / "receiver_params.yaml"
    if source_config.exists():
        return source_config
    return Path(get_package_share_directory("receiver")) / "config" / "receiver_params.yaml"


def generate_launch_description():
    config_file = _config_file()
    log_format = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_OUTPUT_FORMAT",
        "[{severity}] [{name}]: {message}",
    )

    receiver_node = Node(
        package="receiver",
        executable="receiver_node",
        name="video_decoder",
        parameters=[str(config_file)],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([log_format, LogInfo(msg=f"receiver config: {config_file}"), receiver_node])
