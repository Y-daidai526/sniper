#!/usr/bin/env python3
"""
mqtt_server.py — MQTT 发布管理。

职责:
  1. 自动查找/启动 mosquitto broker（本地，localhost:3333）
  2. 将 CustomByteBlock protobuf 发布到 MQTT topic "CustomByteBlock"
"""

import subprocess
import time
import sys
import socket

import paho.mqtt.client as mqtt

from proto import CustomByteBlock_pb2


BROKER_HOST = "127.0.0.1"
BROKER_PORT = 3333
TOPIC = "CustomByteBlock"


class MqttPublisher:
    def __init__(self, broker_host: str = BROKER_HOST, broker_port: int = BROKER_PORT):
        self._host = broker_host
        self._port = broker_port
        self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self._connected = False

    def connect(self) -> bool:
        try:
            self._client.connect(self._host, self._port, keepalive=10)
            self._client.loop_start()
            self._connected = True
            print(f"[mqtt] connected to {self._host}:{self._port}", file=sys.stderr)
            return True
        except Exception as e:
            print(f"[mqtt] connect failed: {e}", file=sys.stderr)
            return False

    def publish(self, data_300: bytes):
        """Protobuf 序列化 → MQTT publish"""
        if not self._connected:
            return
        try:
            msg = CustomByteBlock_pb2.CustomByteBlock()
            msg.data = data_300
            payload = msg.SerializeToString()
            self._client.publish(TOPIC, payload, qos=0)
        except Exception as e:
            print(f"[mqtt] publish error: {e}", file=sys.stderr)

    def disconnect(self):
        if self._connected:
            self._client.loop_stop()
            self._client.disconnect()
            self._connected = False
