from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo, SetEnvironmentVariable
from launch_ros.actions import Node


def _config_file() -> Path:
    source_config = Path.cwd() / "sender" / "config" / "sender_params.yaml"
    if source_config.exists():
        return source_config
    return Path(get_package_share_directory("sender")) / "config" / "sender_params.yaml"


def generate_launch_description():
    config_file = _config_file()
    log_format = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_OUTPUT_FORMAT",
        "[{severity}] [{name}]: {message}",
    )

    sender_node = Node(
        package="sender",
        executable="sender_node",
        parameters=[str(config_file)],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([log_format, LogInfo(msg=f"sender config: {config_file}"), sender_node])
