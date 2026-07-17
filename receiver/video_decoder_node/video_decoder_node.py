#!/usr/bin/env python3

import queue
import threading

import av
import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray

from .parameters import required_parameter


# PyAV errors are reported by this node instead of FFmpeg's internal logger.
av.logging.set_level(av.logging.PANIC)


class VideoDecoderNode(Node):
    def __init__(self):
        super().__init__("video_decoder")
        self._destroyed = False

        queue_size = int(required_parameter(self, "queue_size", Parameter.Type.INTEGER))
        if queue_size <= 0:
            raise RuntimeError("queue_size must be greater than zero")
        self._display = bool(required_parameter(self, "display", Parameter.Type.BOOL))
        self._display_scale = max(
            1,
            int(required_parameter(self, "display_scale", Parameter.Type.INTEGER)),
        )
        self._crosshair_x = int(
            required_parameter(self, "crosshair_offset_x", Parameter.Type.INTEGER)
        )
        self._crosshair_y = int(
            required_parameter(self, "crosshair_offset_y", Parameter.Type.INTEGER)
        )
        self._crosshair_w = max(
            1,
            int(required_parameter(self, "crosshair_width", Parameter.Type.INTEGER)),
        )

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._raw_publisher = self.create_publisher(Image, "video_stream/raw", image_qos)
        self._video_publisher = self.create_publisher(Image, "video_stream", image_qos)

        self._codec = None
        self._reset_decoder()
        self._decode_error_count = 0
        self._subscription = self.create_subscription(
            UInt8MultiArray,
            "encoded_stream",
            self._decode,
            queue_size,
        )

        if self._display:
            self._display_window_size: tuple[int, int] | None = None
            self._frame_queue: queue.Queue = queue.Queue(maxsize=1)
            self._display_stop = threading.Event()
            self._display_thread = threading.Thread(target=self._display_loop, daemon=True)
            self._display_thread.start()

        self.get_logger().info("video decoder started")

    def _reset_decoder(self) -> None:
        with av.logging.Capture(local=False):
            self._codec = av.CodecContext.create("h264", "r")
            self._codec.thread_type = "FRAME"
            try:
                self._codec.flags |= av.codec.context.Flags.LOW_DELAY
            except Exception:
                pass

    def _decode(self, message: UInt8MultiArray) -> None:
        encoded_data = bytes(message.data)
        if len(encoded_data) != 299:
            self.get_logger().warn(f"ignore encoded data size {len(encoded_data)}")
            return

        try:
            # Capture FFmpeg output from both this callback and codec worker threads.
            with av.logging.Capture(local=False):
                for packet in self._codec.parse(encoded_data):
                    for frame in self._codec.decode(packet):
                        self._handle_decoded_frame(frame)
        except Exception as exc:
            self._decode_error_count += 1
            detail = " ".join(str(exc).split()) or exc.__class__.__name__
            self.get_logger().error(f"decode error count={self._decode_error_count}: {detail}")

    def _handle_decoded_frame(self, frame) -> None:
        raw_image = frame.to_ndarray(format="bgr24")
        height, width = raw_image.shape[:2]
        processed_image = cv2.resize(
            raw_image,
            (width * self._display_scale, height * self._display_scale),
            interpolation=cv2.INTER_NEAREST,
        )
        self._draw_overlay(processed_image)

        stamp = self.get_clock().now().to_msg()
        self._raw_publisher.publish(self._to_image_message(raw_image, stamp))
        self._video_publisher.publish(self._to_image_message(processed_image, stamp))

        if self._display:
            try:
                self._frame_queue.put_nowait(processed_image)
            except queue.Full:
                try:
                    self._frame_queue.get_nowait()
                    self._frame_queue.put_nowait(processed_image)
                except queue.Empty:
                    pass

    @staticmethod
    def _to_image_message(image, stamp) -> Image:
        message = Image()
        message.header.stamp = stamp
        message.height, message.width = image.shape[:2]
        message.encoding = "bgr8"
        message.is_bigendian = False
        message.step = message.width * 3
        message.data = image.tobytes()
        return message

    def _display_loop(self) -> None:
        cv2.namedWindow("Sniper Receiver", cv2.WINDOW_NORMAL)
        while not self._display_stop.is_set():
            try:
                image = self._frame_queue.get(timeout=0.05)
                if image is None:
                    break
                display_size = (image.shape[1], image.shape[0])
                cv2.imshow("Sniper Receiver", image)
                if display_size != self._display_window_size:
                    cv2.resizeWindow("Sniper Receiver", *display_size)
                    self._display_window_size = display_size
                cv2.waitKey(1)
            except queue.Empty:
                continue
            except Exception as exc:
                detail = " ".join(str(exc).split()) or exc.__class__.__name__
                self.get_logger().error(f"display error: {detail}")
                break
        cv2.destroyAllWindows()

    def _draw_overlay(self, image) -> None:
        height, width = image.shape[:2]
        center_x = max(0, min(width - 1, width // 2 + self._crosshair_x))
        center_y = max(0, min(height - 1, height // 2 + self._crosshair_y))
        cross_color = (230, 190, 235)
        cv2.line(
            image,
            (0, center_y),
            (width - 1, center_y),
            cross_color,
            self._crosshair_w,
            cv2.LINE_AA,
        )
        cv2.line(
            image,
            (center_x, 0),
            (center_x, height - 1),
            cross_color,
            self._crosshair_w,
            cv2.LINE_AA,
        )
        cv2.circle(
            image,
            (width // 2, height // 2),
            24,
            (170, 255, 170),
            1,
            cv2.LINE_AA,
        )

    def destroy_node(self) -> None:
        if self._destroyed:
            return
        self._destroyed = True
        if self._display:
            self._display_stop.set()
            try:
                self._frame_queue.put_nowait(None)
            except queue.Full:
                pass
            self._display_thread.join(timeout=1.0)
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
        rclpy.try_shutdown()
