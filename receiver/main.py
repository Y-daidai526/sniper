#!/usr/bin/env python3
"""
receiver/main.py — 接收端入口。

启动流程:
  1. 提示输入 client_id（如非通过参数传入）
  2. 连接 MQTT broker 192.168.12.1:3333
  3. 创建 VideoDecoderNode（ROS2 Node）
  4. ROS2 spin
"""

import sys
import rclpy

from receiver.video_decoder_node import VideoDecoderNode


def main(args=None):
    rclpy.init(args=args)

    node = VideoDecoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[receiver] interrupted", file=sys.stderr)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
