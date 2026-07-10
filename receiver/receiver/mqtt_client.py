#!/usr/bin/env python3

import queue
import sys
import time

import paho.mqtt.client as mqtt

from .proto import CustomByteBlock_pb2


def _create_mqtt_client(client_id: str):
    if hasattr(mqtt, "CallbackAPIVersion"):
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    return mqtt.Client(client_id=client_id)


class MqttReceiver:
    def __init__(self, client_id: str, broker_host: str, broker_port: int, topic: str, max_queue: int):
        self._client_id = client_id
        self._host = broker_host
        self._port = broker_port
        self._topic = topic
        self._queue: queue.Queue[bytes] = queue.Queue(maxsize=max_queue)
        self._client = _create_mqtt_client(client_id)
        self._connected = False
        self._loop_started = False
        self._last_connect_attempt_s = 0.0
        self._last_error = ""

        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        self._client.reconnect_delay_set(min_delay=1, max_delay=5)

    def connect(self) -> bool:
        try:
            if self._loop_started:
                self._client.reconnect()
            else:
                self._client.connect(self._host, self._port, keepalive=30)
                self._client.loop_start()
                self._loop_started = True
            self._last_error = ""
            return True
        except Exception as exc:
            self._last_error = str(exc)
            return False

    def retry_connect(self, retry_interval_s: float) -> bool:
        now = time.monotonic()
        if self._connected or now - self._last_connect_attempt_s < retry_interval_s:
            return False
        self._last_connect_attempt_s = now
        return True

    def disconnect(self) -> None:
        if self._loop_started:
            self._client.loop_stop()
        try:
            self._client.disconnect()
        except Exception:
            pass

    def get_data(self, timeout_s: float) -> bytes | None:
        try:
            return self._queue.get(timeout=timeout_s)
        except queue.Empty:
            return None

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def last_error(self) -> str:
        return self._last_error

    def _on_connect(self, client, userdata, flags, reason_code, properties=None) -> None:
        if int(reason_code) != 0:
            self._connected = False
            print(f"[mqtt] connect rejected: reason_code={reason_code}", file=sys.stderr)
            return

        self._connected = True
        client.subscribe(self._topic, qos=0)
        print(
            f"[mqtt] connected {self._host}:{self._port} topic={self._topic} client_id={self._client_id}",
            file=sys.stderr,
        )

    def _on_disconnect(self, client, userdata, *args) -> None:
        reason_code = args[-2] if len(args) >= 2 else args[0] if args else 0
        self._connected = False
        print(f"[mqtt] disconnected: reason_code={reason_code}", file=sys.stderr)

    def _on_message(self, client, userdata, msg) -> None:
        try:
            block = CustomByteBlock_pb2.CustomByteBlock()
            block.ParseFromString(msg.payload)
            if not block.HasField("data"):
                return
            data = bytes(block.data)
            if len(data) != 300:
                print(f"[mqtt] ignore CustomByteBlock data size {len(data)}", file=sys.stderr)
                return
            self._push_latest(data)
        except Exception as exc:
            print(f"[mqtt] parse error: {exc}", file=sys.stderr)

    def _push_latest(self, data: bytes) -> None:
        try:
            self._queue.put_nowait(data)
            return
        except queue.Full:
            pass

        try:
            self._queue.get_nowait()
        except queue.Empty:
            pass
        self._queue.put_nowait(data)
