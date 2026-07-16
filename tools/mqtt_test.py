#!/usr/bin/env python3

"""Subscribe to CustomByteBlock and print its inner sequence number."""

import sys
import time

import paho.mqtt.client as mqtt


CLIENT_ID = "1"
BROKER_HOST = "192.168.12.1"
BROKER_PORT = 3333
TOPIC = "CustomByteBlock"
QOS = 1


def decode_custom_byte_block(payload: bytes) -> bytes:
    if not payload or payload[0] != 0x0A:
        raise ValueError("expected bytes field 1")

    size = 0
    shift = 0
    offset = 1
    while offset < len(payload) and shift < 35:
        byte = payload[offset]
        offset += 1
        size |= (byte & 0x7F) << shift
        if byte < 0x80:
            end = offset + size
            if end != len(payload):
                raise ValueError("invalid data length")
            return payload[offset:end]
        shift += 7

    raise ValueError("invalid data length varint")


class SequenceMonitor:
    def __init__(self):
        self.last_seq = None
        self.packet_count = 0
        self.duplicate_count = 0
        self.missing_count = 0
        self.last_received_at = None

    def record(self, seq: int) -> None:
        received_at = time.monotonic()
        self.packet_count += 1

        if self.last_seq is None:
            print(f"seq={seq:3d} first", flush=True)
            self.last_seq = seq
            self.last_received_at = received_at
            return

        delta_s = received_at - self.last_received_at
        delta_ms = round(delta_s * 1000.0)
        frequency_hz = round(1.0 / delta_s) if delta_s > 0 else 0
        self.last_received_at = received_at
        diff = (seq - self.last_seq) & 0xFF
        if diff == 0:
            self.duplicate_count += 1
            print(
                f"seq={seq:3d} delta={delta_ms}ms hz={frequency_hz} DUPLICATE "
                f"(duplicates={self.duplicate_count})",
                flush=True,
            )
            return

        if diff > 1:
            missing = diff - 1
            self.missing_count += missing
            print(
                f"seq={seq:3d} delta={delta_ms}ms hz={frequency_hz} "
                f"GAP previous={self.last_seq:3d} "
                f"missing={missing} total_missing={self.missing_count}",
                flush=True,
            )
        else:
            print(
                f"seq={seq:3d} delta={delta_ms}ms hz={frequency_hz}", flush=True
            )

        self.last_seq = seq


monitor = SequenceMonitor()


def on_connect(client, userdata, flags, reason_code, properties=None):
    if reason_code.is_failure:
        print(f"[mqtt] connect rejected: reason_code={reason_code}", file=sys.stderr)
        return

    result, message_id = client.subscribe(TOPIC, qos=QOS)
    if result != mqtt.MQTT_ERR_SUCCESS:
        print(f"[mqtt] subscribe failed: result={result}", file=sys.stderr)
        return

    print(
        f"[mqtt] connected {BROKER_HOST}:{BROKER_PORT} topic={TOPIC} "
        f"qos={QOS} client_id={CLIENT_ID} subscribe_mid={message_id}",
        file=sys.stderr,
    )


def on_disconnect(client, userdata, disconnect_flags, reason_code, properties):
    print(f"[mqtt] disconnected: reason_code={reason_code}", file=sys.stderr)


def on_message(client, userdata, msg):
    try:
        data = decode_custom_byte_block(msg.payload)
        if len(data) != 300:
            print(
                f"[mqtt] ignore CustomByteBlock data size {len(data)}, expected 300",
                file=sys.stderr,
            )
            return

        monitor.record(data[0])
    except Exception as exc:
        print(f"[mqtt] parse error: {exc}", file=sys.stderr)


def main() -> int:
    client = mqtt.Client(
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        client_id=CLIENT_ID,
    )
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.reconnect_delay_set(min_delay=1, max_delay=5)

    print(
        f"[mqtt] connecting to {BROKER_HOST}:{BROKER_PORT} "
        f"topic={TOPIC} qos={QOS} client_id={CLIENT_ID}",
        file=sys.stderr,
    )
    try:
        client.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
        client.loop_forever()
    except KeyboardInterrupt:
        print("\n[mqtt] stopped by user", file=sys.stderr)
    except Exception as exc:
        print(f"[mqtt] error: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            client.disconnect()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
