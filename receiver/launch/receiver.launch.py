from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = Path(get_package_share_directory("receiver")) / "config" / "receiver_params.yaml"

    receiver_node = Node(
        package="receiver",
        executable="receiver_node",
        name="video_decoder",
        parameters=[str(config_file)],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([receiver_node])
