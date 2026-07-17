#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

if __package__:
    from .broker import EmbeddedBroker
else:
    from mqtt_broker_node.broker import EmbeddedBroker


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


class MqttBrokerNode(Node):
    def __init__(self):
        super().__init__("mqtt_broker")
        self._destroyed = False
        self._broker: EmbeddedBroker | None = None

        enabled = bool(_required(self, "enabled", Parameter.Type.BOOL))
        host = _required_string(self, "host")
        port = int(_required(self, "port", Parameter.Type.INTEGER))
        if not 1 <= port <= 65535:
            raise RuntimeError(f"port out of range: {port}")

        if not enabled:
            self.get_logger().info("MQTT broker disabled")
            return

        self._broker = EmbeddedBroker(host, port)
        if self._broker.start():
            self.get_logger().info(f"MQTT broker listening on {host}:{port}")
        else:
            error = self._broker.last_error or "unknown error"
            self.get_logger().error(f"MQTT broker start failed: {error}")

    def destroy_node(self) -> None:
        if self._destroyed:
            return
        self._destroyed = True
        if self._broker is not None:
            self._broker.stop()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MqttBrokerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
