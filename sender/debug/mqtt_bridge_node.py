#!/usr/bin/env python3
"""
mqtt_bridge_node.py — ROS2 → MQTT 桥接节点

订阅编码器的 serial_data topic (std_msgs/UInt8MultiArray, 300B)，
Protobuf 序列化后发布到 MQTT。
"""

import sys
import os

# 确保 proto 可导入
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

import paho.mqtt.client as mqtt
from proto import CustomByteBlock_pb2


class MqttBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')

        self.declare_parameter('mqtt_broker_host', '192.168.12.1')
        self.declare_parameter('mqtt_broker_port', 3333)
        self.declare_parameter('mqtt_topic', 'CustomByteBlock')
        self.declare_parameter('serial_data_topic', '/video_encoder/serial_data')

        broker_host = self.get_parameter('mqtt_broker_host').value
        broker_port = self.get_parameter('mqtt_broker_port').value
        self._mqtt_topic = self.get_parameter('mqtt_topic').value
        serial_topic = self.get_parameter('serial_data_topic').value

        # --- MQTT ---
        self._mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        try:
            self._mqtt.connect(broker_host, broker_port, keepalive=10)
            self._mqtt.loop_start()
            self.get_logger().info(
                f'MQTT connected to {broker_host}:{broker_port} '
                f'topic={self._mqtt_topic}'
            )
            self._mqtt_ok = True
        except Exception as e:
            self.get_logger().error(f'MQTT connect failed: {e}')
            self._mqtt_ok = False

        # --- ROS2 subscription ---
        self._sub = self.create_subscription(
            UInt8MultiArray, serial_topic, self._on_serial_data,
            rclpy.QoS(rclpy.qos.qos_profile_sensor_data)
        )

        self._pkt_count = 0

    def _on_serial_data(self, msg: UInt8MultiArray):
        if not self._mqtt_ok:
            return
        try:
            pb = CustomByteBlock_pb2.CustomByteBlock()
            pb.data = bytes(msg.data)
            self._mqtt.publish(self._mqtt_topic, pb.SerializeToString(), qos=0)
            self._pkt_count += 1
            if self._pkt_count % 300 == 0:
                self.get_logger().info(f'MQTT published {self._pkt_count} packets')
        except Exception as e:
            self.get_logger().error(f'MQTT publish error: {e}')

    def destroy_node(self):
        if hasattr(self, '_mqtt') and self._mqtt_ok:
            self._mqtt.loop_stop()
            self._mqtt.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
