#!/usr/bin/env python3

import asyncio
import sys
import threading
import time

import paho.mqtt.client as mqtt

from proto import CustomByteBlock_pb2


def _create_mqtt_client():
    if hasattr(mqtt, "CallbackAPIVersion"):
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    return mqtt.Client()


class EmbeddedBroker:
    def __init__(self, host: str, port: int):
        self._host = host
        self._port = port
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._broker = None
        self._ready: threading.Event | None = None

    def start(self) -> bool:
        try:
            from amqtt.broker import Broker
        except Exception as exc:
            print(f"[mqtt] amqtt import failed: {exc}", file=sys.stderr)
            return False

        config = {
            "listeners": {
                "default": {
                    "type": "tcp",
                    "bind": f"{self._host}:{self._port}",
                }
            },
            "plugins": {
                "amqtt.plugins.authentication.AnonymousAuthPlugin": {
                    "allow_anonymous": True,
                },
            },
        }

        self._loop = asyncio.new_event_loop()
        self._ready = threading.Event()
        result: dict[str, object] = {"ok": False, "error": None}

        def run() -> None:
            asyncio.set_event_loop(self._loop)
            try:
                self._broker = Broker(config, loop=self._loop)
                self._loop.run_until_complete(self._broker.start())
                result["ok"] = True
                self._ready.set()
                self._loop.run_forever()
            except Exception as exc:
                result["error"] = exc
                self._ready.set()

        self._thread = threading.Thread(target=run, name="amqtt-broker", daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout=3.0):
            print("[mqtt] embedded broker startup timed out", file=sys.stderr)
            return False
        if not result["ok"]:
            print(f"[mqtt] embedded broker startup failed: {result['error']}", file=sys.stderr)
            return False
        time.sleep(0.1)
        print(f"[mqtt] embedded broker listening on {self._host}:{self._port}", file=sys.stderr)
        return True

    def stop(self) -> None:
        if not self._loop or not self._broker:
            return

        async def shutdown() -> None:
            await self._broker.shutdown()

        future = asyncio.run_coroutine_threadsafe(shutdown(), self._loop)
        try:
            future.result(timeout=2.0)
        except Exception as exc:
            print(f"[mqtt] broker shutdown warning: {exc}", file=sys.stderr)
        self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join(timeout=2.0)


class MqttPublisher:
    def __init__(self, host: str, port: int, topic: str, start_broker: bool):
        self._host = host
        self._port = port
        self._topic = topic
        self._broker = EmbeddedBroker(host, port) if start_broker else None
        self._client = _create_mqtt_client()
        self._connected = False

    def connect(self) -> bool:
        if self._broker and not self._broker.start():
            print("[mqtt] embedded broker unavailable, trying existing broker", file=sys.stderr)

        try:
            self._client.connect(self._host, self._port, keepalive=10)
            self._client.loop_start()
            self._connected = True
            print(f"[mqtt] publisher connected to {self._host}:{self._port} topic={self._topic}", file=sys.stderr)
            return True
        except Exception as exc:
            print(f"[mqtt] publisher connect failed: {exc}", file=sys.stderr)
            return False

    def publish(self, data_300: bytes) -> None:
        if not self._connected:
            return
        msg = CustomByteBlock_pb2.CustomByteBlock()
        msg.data = data_300
        self._client.publish(self._topic, msg.SerializeToString(), qos=0)

    def disconnect(self) -> None:
        if self._connected:
            self._client.loop_stop()
            self._client.disconnect()
            self._connected = False
        if self._broker:
            self._broker.stop()
