"""
receiver.launch.py — 接收端启动文件

启动 VideoDecoderNode（Python ROS2 Node）。
MQTT 回调驱动视频解码 + OpenCV 显示。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    # 默认参数从 YAML 加载，launch 文件内写的会覆盖 YAML
    config_dir = Path(__file__).parent.parent / 'config'
    decoder_node = Node(
        package='receiver',
        executable='decoder_node',
        name='video_decoder',
        parameters=[
            str(config_dir / 'receiver_params.yaml'),
            {'client_id': ''},  # 空 → 自动生成 sniper_decoder_xxxxxxxx
        ],
        output='screen',
    )

    return LaunchDescription([decoder_node])
