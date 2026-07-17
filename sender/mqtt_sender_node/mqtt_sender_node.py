#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import UInt8MultiArray

if __package__:
    from .publisher import MqttPublisher
    from .rm_frame import parse_frame
else:
    from mqtt_sender_node.publisher import MqttPublisher
    from mqtt_sender_node.rm_frame import parse_frame


def _required(node: Node, name: str, param_type: Parameter.Type):
    node.declare_parameter(name, param_type)
    value = node.get_parameter(name).value
    if value is None:
        raise RuntimeError(f"missing required parameter: {name}")
    return value


def _required_string(node: Node, name: str) -> str:
    value = str(_required(node, name, Parameter.Type.STRING))
    if not value:
        raise RuntimeError(f"required parameter is empty: {name}")
    return value


class MqttSenderNode(Node):
    def __init__(self):
        super().__init__("mqtt_sender")
        self._destroyed = False
        self._publisher: MqttPublisher | None = None
        self._subscription = None
        self._retry_timer = None
        self._last_retry_log_s = 0.0
        self._reported_connected = False
        self._parse_errors = 0

        enabled = bool(_required(self, "enabled", Parameter.Type.BOOL))
        host = _required_string(self, "host")
        port = int(_required(self, "port", Parameter.Type.INTEGER))
        topic = _required_string(self, "topic")
        if not 1 <= port <= 65535:
            raise RuntimeError(f"port out of range: {port}")

        if not enabled:
            self.get_logger().info("MQTT sender disabled")
            return

        self._publisher = MqttPublisher(host, port, topic)
        self._subscription = self.create_subscription(
            UInt8MultiArray,
            "rm_frame",
            self._on_frame,
            100,
        )
        self._retry_timer = self.create_timer(1.0, self._maintain_connection)
        self._maintain_connection()
        self.get_logger().info(f"MQTT sender started host={host}:{port} topic={topic}")

    def _on_frame(self, message: UInt8MultiArray) -> None:
        frame = bytes(message.data)
        if len(frame) != 309:
            self._report_parse_error(f"wrong size: {len(frame)} != 309")
            return

        ok, data_300, error = parse_frame(frame)
        if not ok or data_300 is None:
            self._report_parse_error(error)
            return

        if self._publisher is not None:
            self._publisher.publish(data_300)

    def _maintain_connection(self) -> None:
        if self._publisher is None:
            return

        if self._publisher.connected:
            if not self._reported_connected:
                self.get_logger().info("MQTT publisher connected")
                self._reported_connected = True
            return

        self._reported_connected = False
        self._publisher.connect()
        error = self._publisher.last_error
        if not error:
            return

        now = time.monotonic()
        if now - self._last_retry_log_s >= 3.0:
            self.get_logger().warn(f"MQTT disconnected, retrying: {error}")
            self._last_retry_log_s = now

    def _report_parse_error(self, error: str) -> None:
        self._parse_errors += 1
        if self._parse_errors <= 5 or self._parse_errors % 100 == 0:
            self.get_logger().warn(
                f"invalid RM frame count={self._parse_errors}: {' '.join(error.split())}"
            )

    def destroy_node(self) -> None:
        if self._destroyed:
            return
        self._destroyed = True
        if self._retry_timer is not None:
            self.destroy_timer(self._retry_timer)
        if self._subscription is not None:
            self.destroy_subscription(self._subscription)
        if self._publisher is not None:
            self._publisher.disconnect()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MqttSenderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
