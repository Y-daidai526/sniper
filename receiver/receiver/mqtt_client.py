#!/usr/bin/env python3

import queue
import sys

import paho.mqtt.client as mqtt

from .proto import CustomByteBlock_pb2


class MqttReceiver:
    FRAME_SIZE = 300

    def __init__(self, client_id: str, broker_host: str, broker_port: int, topic: str, max_queue: int):
        self._client_id = client_id
        self._host = broker_host
        self._port = broker_port
        self._topic = topic
        self._queue: queue.Queue[bytes] = queue.Queue(maxsize=max_queue)
        self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        self._connected = False

        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message

    def connect(self) -> bool:
        try:
            self._client.connect(self._host, self._port, keepalive=30)
            self._client.loop_start()
            return True
        except Exception as exc:
            print(f"[mqtt] connect failed: {exc}", file=sys.stderr)
            return False

    def disconnect(self) -> None:
        self._client.loop_stop()
        self._client.disconnect()

    def get_data(self, timeout_s: float) -> bytes | None:
        try:
            return self._queue.get(timeout=timeout_s)
        except queue.Empty:
            return None

    @property
    def connected(self) -> bool:
        return self._connected

    def _on_connect(self, client, userdata, flags, reason_code, properties) -> None:
        if int(reason_code) == 0:
            self._connected = True
            client.subscribe(self._topic, qos=0)
            print(
                f"[mqtt] connected {self._host}:{self._port} topic={self._topic} client_id={self._client_id}",
                file=sys.stderr,
            )
        else:
            self._connected = False
            print(f"[mqtt] connect rejected: reason_code={reason_code}", file=sys.stderr)

    def _on_disconnect(self, client, userdata, flags, reason_code, properties) -> None:
        self._connected = False
        print(f"[mqtt] disconnected: reason_code={reason_code}", file=sys.stderr)

    def _on_message(self, client, userdata, msg) -> None:
        try:
            block = CustomByteBlock_pb2.CustomByteBlock()
            block.ParseFromString(msg.payload)
            if not block.HasField("data"):
                return
            data = bytes(block.data)
            if len(data) != self.FRAME_SIZE:
                print(f"[mqtt] ignore CustomByteBlock data size {len(data)}", file=sys.stderr)
                return
            try:
                self._queue.put_nowait(data)
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
                self._queue.put_nowait(data)
        except Exception as exc:
            print(f"[mqtt] parse error: {exc}", file=sys.stderr)
