#!/usr/bin/env python3
"""
main.py — debug 桥接子进程入口。

流程:
  stdin.read(309) → CRC验证 → 提取300B → Protobuf → MQTT publish

由 C++ debug_bridge 通过 stdin pipe 驱动。
"""

import sys
import os

# 确保当前目录在 path 中，以便导入 proto 和同目录模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from serial_parser import parse_frame, FRAME_SIZE
from mqtt_server import MqttPublisher


def main():
    print("[debug] child process started, reading stdin...", file=sys.stderr)

    publisher = MqttPublisher()
    if not publisher.connect():
        print("[debug] MQTT connection failed, exiting", file=sys.stderr)
        return 1

    frame_count = 0
    error_count = 0

    while True:
        # 阻塞读取 309B
        raw = sys.stdin.buffer.read(FRAME_SIZE)
        if len(raw) == 0:
            print(f"[debug] stdin EOF, total frames={frame_count} errors={error_count}",
                  file=sys.stderr)
            break
        if len(raw) != FRAME_SIZE:
            print(f"[debug] short read: {len(raw)}B, exiting", file=sys.stderr)
            break

        ok, data_300, seq, err = parse_frame(raw)
        if not ok:
            error_count += 1
            if error_count <= 5 or error_count % 100 == 0:
                print(f"[debug] frame #{frame_count + error_count} parse error: {err}",
                      file=sys.stderr)
            continue

        frame_count += 1
        # 提取内部 seq
        inner_seq = data_300[0]
        publisher.publish(data_300)

        if frame_count % 300 == 0:
            print(f"[debug] published {frame_count} frames, errors={error_count}",
                  file=sys.stderr)

    publisher.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
