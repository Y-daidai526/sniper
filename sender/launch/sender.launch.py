"""
sender.launch.py — 发送端启动文件

ComposableNodeContainer 零拷贝:
  HikCameraNode (MVS SDK) → image_raw → VideoEncoderNode (GStreamer H264)

输出:
  - ROS2 topic /video_stream (VideoPacket, 150B) → 兼容原解码器
  - ROS2 topic /video_encoder/serial_data (UInt8MultiArray, 300B) → MQTT bridge
  - USB CDC serial (309B frame) → 裁判系统图传
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from pathlib import Path


def generate_launch_description():
    launch_path = Path(__file__).resolve()
    project_root = launch_path.parents[1]  # sender/ → workspace root
    config_dir = launch_path.parent.parent / 'config'
    params_file = str(config_dir / 'sender_params.yaml')

    # 从 YAML 加载所有参数，仅覆盖需要计算的值
    camera_params = [params_file]
    encoder_params = [
        params_file,
        {'debug_dump_dir': str(project_root / 'sniper_debug_imgs')},
    ]

    # 相机 + 编码器（零拷贝容器）
    encoder_container = ComposableNodeContainer(
        name='sniper_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='sender',
                plugin='sniper::camera::HikCameraNode',
                name='hik_camera',
                parameters=camera_params,
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='sender',
                plugin='sniper::encoder::VideoEncoderNode',
                name='video_encoder',
                parameters=encoder_params,
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    # MQTT 桥接节点（Python）
    mqtt_bridge = Node(
        package='sender',
        executable='mqtt_bridge_node.py',
        name='mqtt_bridge',
        parameters=[params_file],
        output='screen',
    )

    return LaunchDescription([encoder_container, mqtt_bridge])
