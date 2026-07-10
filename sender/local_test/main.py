#!/usr/bin/env python3

import argparse
import sys

from mqtt_server import MqttPublisher
from serial_parser import parse_frame


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="stdin 309B referee frames -> local MQTT CustomByteBlock")
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", required=True, type=int)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--start-broker", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    publisher = MqttPublisher(args.host, args.port, args.topic, args.start_broker)
    if not publisher.connect():
        return 1

    frame_count = 0
    error_count = 0
    try:
        while True:
            raw = sys.stdin.buffer.read(309)
            if len(raw) == 0:
                print(f"[local_test] stdin EOF frames={frame_count} errors={error_count}", file=sys.stderr)
                break
            if len(raw) != 309:
                print(f"[local_test] short read: {len(raw)}B", file=sys.stderr)
                break

            ok, data_300, seq, error = parse_frame(raw)
            if not ok:
                error_count += 1
                if error_count <= 5 or error_count % 100 == 0:
                    print(f"[local_test] parse error: {error}", file=sys.stderr)
                continue

            publisher.publish(data_300)
            frame_count += 1
    finally:
        publisher.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
