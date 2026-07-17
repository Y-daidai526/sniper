#!/usr/bin/env python3

import threading
import time

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import UInt8MultiArray

from .parameters import required_parameter, required_string
from .proto import CustomByteBlock_pb2


class MqttReceiverNode(Node):
    def __init__(self):
        super().__init__("mqtt_receiver")
        self._destroyed = False

        self._host = required_string(self, "broker_host")
        self._port = int(required_parameter(self, "broker_port", Parameter.Type.INTEGER))
        self._topic = required_string(self, "topic")
        client_id = required_string(self, "client_id")
        if not 1 <= self._port <= 65535:
            raise RuntimeError("broker_port must be between 1 and 65535")

        self._publisher = self.create_publisher(UInt8MultiArray, "encoded_stream", 10)
        self._client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=client_id,
        )
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        self._client.reconnect_delay_set(min_delay=1, max_delay=5)

        self._connected = False
        self._loop_started = False
        self._stopping = False
        self._last_error = ""
        self._last_retry_log_s = 0.0

        self._stats_lock = threading.Lock()
        self._last_seq: int | None = None
        self._packets = 0
        self._bytes = 0
        self._gaps = 0
        self._dups = 0
        self._last_stats_s = time.monotonic()
        self._last_stats_packets = 0
        self._last_stats_bytes = 0

        self._maintenance_timer = self.create_timer(1.0, self._maintain_connection)
        self._stats_timer = self.create_timer(1.0, self._log_stats)
        self.get_logger().info(
            f"connecting MQTT {self._host}:{self._port} topic={self._topic} client_id={client_id}"
        )
        self._connect()

    def _connect(self) -> None:
        try:
            self._client.connect(self._host, self._port, keepalive=30)
            if not self._loop_started:
                self._client.loop_start()
                self._loop_started = True
            self._last_error = ""
        except Exception as exc:
            self._last_error = self._one_line(exc)

    def _maintain_connection(self) -> None:
        if self._connected:
            return

        # Paho reconnects an established network loop itself. The timer covers
        # startup failures where the loop could not be started at all.
        if not self._loop_started:
            self._connect()

        now = time.monotonic()
        if now - self._last_retry_log_s >= 3.0:
            detail = f": {self._last_error}" if self._last_error else ""
            self.get_logger().warn(f"MQTT disconnected, retrying{detail}")
            self._last_retry_log_s = now

    def _on_connect(self, client, userdata, flags, reason_code, properties=None) -> None:
        if reason_code.is_failure:
            self._connected = False
            self._last_error = f"reason_code={reason_code}"
            return

        self._connected = True
        self._last_error = ""
        client.subscribe(self._topic, qos=0)
        self.get_logger().info(f"MQTT connected {self._host}:{self._port} topic={self._topic}")

    def _on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties) -> None:
        self._connected = False
        if not self._stopping:
            self._last_error = f"reason_code={reason_code}"

    def _on_message(self, client, userdata, message) -> None:
        try:
            block = CustomByteBlock_pb2.CustomByteBlock()
            block.ParseFromString(message.payload)
            if not block.HasField("data"):
                return

            data = bytes(block.data)
            if len(data) != 300:
                self.get_logger().warn(f"ignore CustomByteBlock data size {len(data)}")
                return

            seq = data[0]
            encoded_data = data[1:]
            duplicate, gap = self._record_packet(seq, len(encoded_data))
            if duplicate:
                return
            if gap:
                self.get_logger().warn(f"sequence gap: missing={gap}")

            ros_message = UInt8MultiArray()
            ros_message.data = encoded_data
            self._publisher.publish(ros_message)
        except Exception as exc:
            self.get_logger().error(f"MQTT message error: {self._one_line(exc)}")

    def _record_packet(self, seq: int, payload_size: int) -> tuple[bool, int]:
        duplicate = False
        gap = 0
        with self._stats_lock:
            if self._last_seq is not None:
                diff = (seq - self._last_seq) & 0xFF
                if diff == 0:
                    duplicate = True
                    self._dups += 1
                elif diff > 1:
                    gap = diff - 1
                    self._gaps += gap

            if not duplicate:
                self._last_seq = seq
                self._packets += 1
                self._bytes += payload_size
        return duplicate, gap

    def _log_stats(self) -> None:
        now = time.monotonic()
        with self._stats_lock:
            elapsed = max(now - self._last_stats_s, 1e-6)
            packet_delta = self._packets - self._last_stats_packets
            byte_delta = self._bytes - self._last_stats_bytes
            packet_rate = packet_delta / elapsed
            data_rate = byte_delta / elapsed / 1000.0
            packets = self._packets
            gaps = self._gaps
            dups = self._dups
            self._last_stats_s = now
            self._last_stats_packets = self._packets
            self._last_stats_bytes = self._bytes

        self.get_logger().info(
            f"rate={packet_rate:.1f}pkt/s data={data_rate:.2f}kB/s "
            f"packets={packets} gaps={gaps} dups={dups}"
        )

    @staticmethod
    def _one_line(value: object) -> str:
        return " ".join(str(value).split()) or value.__class__.__name__

    def destroy_node(self) -> None:
        if self._destroyed:
            return
        self._destroyed = True
        self._stopping = True
        if self._loop_started:
            try:
                self._client.disconnect()
            except Exception:
                pass
            self._client.loop_stop()
            self._loop_started = False
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MqttReceiverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
