from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = Path(get_package_share_directory("sender")) / "config" / "sender_params.yaml"

    sender_node = Node(
        package="sender",
        executable="sender_node",
        parameters=[str(config_file)],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([sender_node])
