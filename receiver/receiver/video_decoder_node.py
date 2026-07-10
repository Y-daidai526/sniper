#!/usr/bin/env python3

import queue
import threading
import time

import av
import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from .mqtt_client import MqttReceiver
from .stats_tracker import StatsTracker


def _required_param(node: Node, name: str, param_type: Parameter.Type):
    node.declare_parameter(name, param_type)
    value = node.get_parameter(name).value
    if value is None:
        raise RuntimeError(f"missing required parameter: {name}")
    return value


def _required_string(node: Node, name: str) -> str:
    value = str(_required_param(node, name, Parameter.Type.STRING))
    if not value:
        raise RuntimeError(f"required parameter is empty: {name}")
    return value


class VideoDecoderNode(Node):
    def __init__(self):
        super().__init__("video_decoder")

        # Runtime selection is intentionally config-only; change YAML for local or match use.
        broker_host = _required_string(self, "broker_host")
        client_id = _required_string(self, "client_id")
        broker_port = int(_required_param(self, "broker_port", Parameter.Type.INTEGER))
        topic = _required_string(self, "topic")
        mqtt_queue_size = int(_required_param(self, "mqtt_queue_size", Parameter.Type.INTEGER))
        self._display = bool(_required_param(self, "display", Parameter.Type.BOOL))
        self._display_scale = max(1, int(_required_param(self, "display_scale", Parameter.Type.INTEGER)))
        self._crosshair_x = int(_required_param(self, "crosshair_offset_x", Parameter.Type.INTEGER))
        self._crosshair_y = int(_required_param(self, "crosshair_offset_y", Parameter.Type.INTEGER))
        self._crosshair_w = max(1, int(_required_param(self, "crosshair_width", Parameter.Type.INTEGER)))

        self.get_logger().info(f"connecting MQTT {broker_host}:{broker_port} topic={topic} client_id={client_id}")
        self._mqtt = MqttReceiver(client_id, broker_host, broker_port, topic, mqtt_queue_size)
        if not self._mqtt.connect():
            self.get_logger().error(f"MQTT connection failed: {self._mqtt.last_error}")
        self._last_mqtt_retry_log_s = 0.0

        self._codec = None
        self._reset_decoder("startup")
        self._frame_count = 0
        self._stats = StatsTracker()

        if self._display:
            self._frame_queue: queue.Queue = queue.Queue(maxsize=3)
            self._display_thread = threading.Thread(target=self._display_loop, daemon=True)
            self._display_thread.start()

        self._timer = self.create_timer(0.001, self._poll_mqtt)
        self.get_logger().info("receiver started")

    def _reset_decoder(self, reason: str) -> None:
        self._codec = av.CodecContext.create("h264", "r")
        self._codec.thread_type = "FRAME"
        try:
            self._codec.flags |= av.codec.context.Flags.LOW_DELAY
        except Exception:
            pass
        self.get_logger().warn(f"reset decoder ({reason})")

    def _poll_mqtt(self) -> None:
        if not self._mqtt.connected and self._mqtt.retry_connect(1.0):
            if self._mqtt.connect():
                self.get_logger().info("MQTT reconnect requested")
            else:
                now = time.monotonic()
                if now - self._last_mqtt_retry_log_s >= 3.0:
                    self.get_logger().warn(
                        f"MQTT disconnected, retrying {self._mqtt.last_error}"
                    )
                    self._last_mqtt_retry_log_s = now

        while True:
            data = self._mqtt.get_data(0.0)
            if data is None:
                break

            seq = data[0]
            h264_slice = data[1:]  # 299B Annex-B chunk from the 0x0310 data field.
            status = self._stats.record(seq, len(h264_slice))
            if status.duplicate:
                continue
            if status.gap > 0:
                self.get_logger().warn(f"sequence gap: missing={status.gap}, reset decoder")
                self._reset_decoder("sequence gap")

            try:
                for packet in self._codec.parse(h264_slice):
                    for frame in self._codec.decode(packet):
                        self._handle_decoded_frame(frame)
            except Exception as exc:
                self._stats.record_error()
                self.get_logger().debug(f"decode error: {exc}")

    def _handle_decoded_frame(self, frame) -> None:
        if frame is None or frame.width == 0 or frame.height == 0:
            return
        img = frame.to_ndarray(format="bgr24")
        if img is None or img.size == 0:
            return

        self._frame_count += 1
        if self._display:
            try:
                self._frame_queue.put_nowait(img)
            except queue.Full:
                pass
        elif self._frame_count % 60 == 0:
            self.get_logger().info(f"decoded frames={self._frame_count}")

    def _display_loop(self) -> None:
        cv2.namedWindow("Sniper Receiver", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            try:
                img = self._frame_queue.get(timeout=0.05)
                if img is None:
                    break
                h, w = img.shape[:2]
                display_size = (w * self._display_scale, h * self._display_scale)
                img_disp = cv2.resize(img, display_size, interpolation=cv2.INTER_NEAREST)
                self._draw_overlay(img_disp)
                cv2.imshow("Sniper Receiver", img_disp)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    rclpy.shutdown()
                    break
            except queue.Empty:
                continue
            except Exception as exc:
                self.get_logger().error(f"display error: {exc}")
                break
        cv2.destroyAllWindows()

    def _draw_overlay(self, img) -> None:
        h, w = img.shape[:2]
        cx = max(0, min(w - 1, w // 2 + self._crosshair_x))
        cy = max(0, min(h - 1, h // 2 + self._crosshair_y))
        cross_color = (230, 190, 235)
        cv2.line(img, (0, cy), (w - 1, cy), cross_color, self._crosshair_w, cv2.LINE_AA)
        cv2.line(img, (cx, 0), (cx, h - 1), cross_color, self._crosshair_w, cv2.LINE_AA)
        cv2.circle(img, (w // 2, h // 2), 24, (170, 255, 170), 1, cv2.LINE_AA)

    def destroy_node(self) -> None:
        if self._display:
            try:
                self._frame_queue.put_nowait(None)
            except queue.Full:
                pass
            self._display_thread.join(timeout=1.0)
        self._mqtt.disconnect()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = VideoDecoderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
