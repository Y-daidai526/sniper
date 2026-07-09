#!/usr/bin/env python3
"""
mqtt_client.py — MQTT 订阅 + Protobuf 反序列化。

订阅 topic: CustomByteBlock
Broker: 192.168.12.1:3333
"""

import sys
import threading
import queue

import paho.mqtt.client as mqtt

from .proto import CustomByteBlock_pb2


class MqttReceiver:
    """MQTT 客户端，接收 CustomByteBlock 并写入队列供 decoder 消费。"""

    FRAME_SIZE = 300  # 1B seq + 299B H264

    def __init__(self,
                 client_id: str,
                 broker_host: str = "192.168.12.1",
                 broker_port: int = 3333,
                 topic: str = "CustomByteBlock",
                 max_queue: int = 1000):
        self._client_id = client_id
        self._host = broker_host
        self._port = broker_port
        self._topic = topic
        self._queue = queue.Queue(maxsize=max_queue)
        self._client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=client_id,
        )
        self._connected = False

        # 回调
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.on_disconnect = self._on_disconnect

    # --- public ---

    def connect(self) -> bool:
        try:
            self._client.connect(self._host, self._port, keepalive=30)
            self._client.loop_start()
            return True
        except Exception as e:
            print(f"[mqtt] connect failed: {e}", file=sys.stderr)
            return False

    def disconnect(self):
        self._client.loop_stop()
        self._client.disconnect()

    def get_data(self, timeout: float = 0.05) -> bytes | None:
        """非阻塞取一帧 300B data，无数据返回 None。"""
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    @property
    def connected(self) -> bool:
        return self._connected

    # --- callbacks ---

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            self._connected = True
            client.subscribe(self._topic, qos=0)
            print(f"[mqtt] connected to {self._host}:{self._port}, "
                  f"subscribed '{self._topic}' (client_id={self._client_id})",
                  file=sys.stderr)
        else:
            self._connected = False
            print(f"[mqtt] connect failed: reason_code={reason_code}", file=sys.stderr)

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        self._connected = False
        print(f"[mqtt] disconnected: reason_code={reason_code}", file=sys.stderr)

    def _on_message(self, client, userdata, msg):
        try:
            block = CustomByteBlock_pb2.CustomByteBlock()
            block.ParseFromString(msg.payload)
            if block.HasField("data") and len(block.data) == self.FRAME_SIZE:
                try:
                    self._queue.put_nowait(block.data)
                except queue.Full:
                    pass  # 丢弃最旧帧由 decoder 丢包检测处理，这里静默丢弃
        except Exception as e:
            print(f"[mqtt] parse error: {e}", file=sys.stderr)
