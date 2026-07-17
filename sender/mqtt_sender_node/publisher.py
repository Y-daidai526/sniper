import threading

import paho.mqtt.client as mqtt

from .proto import CustomByteBlock_pb2


def _one_line(error: object) -> str:
    return " ".join(str(error).split())


class MqttPublisher:
    """Publish CustomByteBlock messages and retry after disconnects."""

    def __init__(self, host: str, port: int, topic: str):
        self._host = host
        self._port = port
        self._topic = topic
        self._client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        self._client.on_connect = self._on_connect
        self._client.on_connect_fail = self._on_connect_fail
        self._client.on_disconnect = self._on_disconnect
        self._client.reconnect_delay_set(min_delay=1, max_delay=1)

        self._connected = False
        self._loop_started = False
        self._closed = False
        self._last_error = ""
        self._lock = threading.Lock()

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    def connect(self) -> bool:
        with self._lock:
            if self._closed:
                return False
            if self._connected or self._loop_started:
                return True

        try:
            self._client.connect(self._host, self._port, keepalive=30)
            self._client.loop_start()
        except Exception as exc:
            self._set_disconnected(_one_line(exc))
            return False

        with self._lock:
            self._loop_started = True
            self._last_error = ""
        return True

    def publish(self, data_300: bytes) -> bool:
        if not self.connected:
            return False

        message = CustomByteBlock_pb2.CustomByteBlock()
        message.data = data_300
        result = self._client.publish(self._topic, message.SerializeToString(), qos=0)
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            return True

        self._set_disconnected(mqtt.error_string(result.rc))
        return False

    def disconnect(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            loop_started = self._loop_started
            self._connected = False

        if not loop_started:
            return
        try:
            self._client.disconnect()
        except Exception:
            pass
        self._client.loop_stop()
        with self._lock:
            self._loop_started = False

    def _on_connect(self, client, userdata, flags, reason_code, properties=None) -> None:
        del client, userdata, flags, properties
        if reason_code.is_failure:
            self._set_disconnected(f"connection rejected: {reason_code}")
            return
        with self._lock:
            self._connected = True
            self._last_error = ""

    def _on_connect_fail(self, client, userdata) -> None:
        del client, userdata
        self._set_disconnected("connection attempt failed")

    def _on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties) -> None:
        del client, userdata, disconnect_flags, properties
        self._set_disconnected(f"reason_code={reason_code}")

    def _set_disconnected(self, error: str) -> None:
        with self._lock:
            self._connected = False
            self._last_error = error
