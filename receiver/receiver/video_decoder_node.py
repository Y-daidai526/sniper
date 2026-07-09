"""
video_decoder_node.py — H264 视频解码 + 显示（ROS2 Python Node）。

从 MQTT 队列取 300B data:
  data[0]   = 内部 seq (1B) → 丢包检测
  data[1:]  = H264 Annex-B 切片 (299B) → PyAV 解码

参考原 doorlock_decoder 的结构，但数据源从 ROS2 Subscription 改为 MQTT 队列。
"""

import rclpy
from rclpy.node import Node

import av
import cv2
import threading
import queue
import time

from .mqtt_client import MqttReceiver
from .stats_tracker import StatsTracker


class VideoDecoderNode(Node):
    def __init__(self):
        super().__init__('video_decoder_node')

        # --- 参数 ---
        self.declare_parameter('broker_host', '192.168.12.1')
        self.declare_parameter('broker_port', 3333)
        self.declare_parameter('topic', 'CustomByteBlock')
        self.declare_parameter('client_id', '')
        self.declare_parameter('display', True)
        self.declare_parameter('width', 300)
        self.declare_parameter('height', 300)
        self.declare_parameter('display_scale', 2)
        self.declare_parameter('crosshair_offset_x', 0)
        self.declare_parameter('crosshair_offset_y', 0)
        self.declare_parameter('crosshair_width', 1)
        self.declare_parameter('debug_dump_enable', False)
        self.declare_parameter('debug_dump_every_n_frames', 20)
        self.declare_parameter('debug_dump_dir', 'sniper_debug_imgs')

        broker_host = self.get_parameter('broker_host').value
        broker_port = self.get_parameter('broker_port').value
        topic = self.get_parameter('topic').value
        client_id = self.get_parameter('client_id').value
        self._display = self.get_parameter('display').value
        self._width = self.get_parameter('width').value
        self._height = self.get_parameter('height').value
        self._display_scale = max(1, self.get_parameter('display_scale').value)
        self._crosshair_x = self.get_parameter('crosshair_offset_x').value
        self._crosshair_y = self.get_parameter('crosshair_offset_y').value
        self._crosshair_w = max(1, self.get_parameter('crosshair_width').value)

        # 如果没有提供 client_id，自动生成
        if not client_id:
            import uuid
            client_id = f"sniper_decoder_{uuid.uuid4().hex[:8]}"

        self.get_logger().info(
            f"Connecting to MQTT {broker_host}:{broker_port} "
            f"topic='{topic}' client_id='{client_id}'"
        )

        # --- MQTT ---
        self._mqtt = MqttReceiver(
            client_id=client_id,
            broker_host=broker_host,
            broker_port=broker_port,
            topic=topic,
        )
        if not self._mqtt.connect():
            self.get_logger().error("Failed to connect to MQTT broker")

        # --- 解码器 ---
        self._codec = None
        self._reset_decoder(reason='startup')
        self._frame_count = 0
        self._packet_count = 0
        self._last_seq = None

        # --- 统计 ---
        self._stats = StatsTracker()

        # --- 显示 ---
        if self._display:
            self._frame_queue = queue.Queue(maxsize=3)
            self._display_thread = threading.Thread(
                target=self._display_loop, daemon=True
            )
            self._display_thread.start()

        # --- 主循环定时器 (替代 ROS2 Subscription) ---
        self._timer = self.create_timer(0.001, self._poll_mqtt)  # 1ms poll

        self.get_logger().info("Decoder started")

    # ================================================================
    # 解码器
    # ================================================================

    def _reset_decoder(self, reason: str = ''):
        self._codec = av.CodecContext.create('h264', 'r')
        self._codec.thread_type = 'FRAME'
        self._codec.flags |= av.codec.context.Flags.LOW_DELAY
        if reason:
            self.get_logger().warn(f"Reset decoder ({reason})")

    def _handle_decoded_frame(self, frame):
        if frame is None or frame.width == 0 or frame.height == 0:
            return
        img = frame.to_ndarray(format='bgr24')
        if img is None or img.size == 0:
            return

        self._frame_count += 1
        if self._display:
            try:
                self._frame_queue.put_nowait(img)
            except queue.Full:
                pass
        elif self._frame_count % 60 == 0:
            self.get_logger().info(f"Decoded {self._frame_count} frames")

    # ================================================================
    # MQTT poll
    # ================================================================

    def _poll_mqtt(self):
        """定时器回调：从 MQTT 队列取数据，包给 PyAV 解码。"""
        while True:
            data = self._mqtt.get_data(timeout=0.0)
            if data is None:
                break

            self._packet_count += 1

            # data[0] = 内部 seq
            seq = data[0]
            h264_slice = bytes(data[1:])

            # 丢包检测
            if self._last_seq is not None:
                diff = (seq - self._last_seq) & 0xFF
                if diff == 0:
                    self._stats.record(seq, 0)  # dup
                elif diff > 1:
                    self.get_logger().warn(
                        f"Gap: {self._last_seq} -> {seq} (gap={diff - 1}), "
                        f"reset decoder"
                    )
                    self._reset_decoder(reason='sequence gap')

            self._last_seq = seq
            self._stats.record(seq, len(h264_slice))

            # PyAV 解码
            try:
                parsed = self._codec.parse(h264_slice)
                for packet in parsed:
                    for frame in self._codec.decode(packet):
                        self._handle_decoded_frame(frame)
            except av.AVError as e:
                self.get_logger().debug(f"Decode error: {e}")

    # ================================================================
    # 显示
    # ================================================================

    def _display_loop(self):
        display_w = self._width * self._display_scale
        display_h = self._height * self._display_scale

        cv2.namedWindow('Sniper Decoder', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Sniper Decoder', display_w, display_h)

        while rclpy.ok():
            try:
                img = self._frame_queue.get(timeout=0.05)
                if img is None:
                    break
                if img.size > 0:
                    img_disp = cv2.resize(
                        img, (display_w, display_h),
                        interpolation=cv2.INTER_NEAREST,
                    )
                    self._draw_overlay(img_disp)
                    cv2.imshow('Sniper Decoder', img_disp)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.get_logger().info('User quit')
                        rclpy.shutdown()
                        break
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Display error: {e}")
                break

        cv2.destroyAllWindows()

    def _draw_overlay(self, img):
        h, w = img.shape[:2]
        cx = max(0, min(w - 1, w // 2 + self._crosshair_x))
        cy = max(0, min(h - 1, h // 2 + self._crosshair_y))

        cross_color = (230, 190, 235)  # BGR 淡紫
        cv2.line(img, (0, cy), (w - 1, cy), cross_color, self._crosshair_w, cv2.LINE_AA)
        cv2.line(img, (cx, 0), (cx, h - 1), cross_color, self._crosshair_w, cv2.LINE_AA)

        center_color = (170, 255, 170)  # BGR 淡绿
        cv2.circle(img, (w // 2, h // 2), 24, center_color, 1, cv2.LINE_AA)

    # ================================================================
    # 生命周期
    # ================================================================

    def destroy_node(self):
        if self._display:
            try:
                self._frame_queue.put_nowait(None)
            except queue.Full:
                pass
            self._display_thread.join(timeout=1.0)
        self._mqtt.disconnect()
        super().destroy_node()


def main(args=None):
    """Entry point for ros2 run."""
    rclpy.init(args=args)
    node = VideoDecoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
