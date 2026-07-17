import asyncio
import logging
import threading


def _one_line(error: object) -> str:
    return " ".join(str(error).split())


class EmbeddedBroker:
    """Run an amqtt broker on its own asyncio thread."""

    def __init__(self, host: str, port: int):
        self._host = host
        self._port = port
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._broker = None
        self._ready = threading.Event()
        self._started = False
        self._stopped = False
        self._last_error = ""
        self._lock = threading.Lock()

    @property
    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    def start(self) -> bool:
        with self._lock:
            if self._started:
                return True
            if self._stopped:
                return False

        try:
            logging.getLogger("amqtt").setLevel(logging.CRITICAL)
            from amqtt.broker import Broker
        except Exception as exc:
            self._set_error(f"amqtt import failed: {_one_line(exc)}")
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

        def run() -> None:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            with self._lock:
                self._loop = loop

            try:
                broker = Broker(config, loop=loop)
                loop.run_until_complete(broker.start())
                with self._lock:
                    self._broker = broker
                    self._started = True
                    self._last_error = ""
                self._ready.set()
                loop.run_forever()
            except Exception as exc:
                self._set_error(_one_line(exc))
                self._ready.set()
            finally:
                with self._lock:
                    self._started = False
                loop.close()

        self._thread = threading.Thread(target=run, name="amqtt-broker", daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout=3.0):
            self._set_error("startup timed out")
            return False

        with self._lock:
            return self._started

    def stop(self) -> None:
        with self._lock:
            if self._stopped:
                return
            self._stopped = True
            loop = self._loop
            broker = self._broker
            thread = self._thread

        if loop is not None and broker is not None and loop.is_running():
            future = asyncio.run_coroutine_threadsafe(broker.shutdown(), loop)
            try:
                future.result(timeout=2.0)
            except Exception:
                future.cancel()

        if loop is not None and loop.is_running():
            loop.call_soon_threadsafe(loop.stop)
        if thread is not None:
            thread.join(timeout=2.0)

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._last_error = message
