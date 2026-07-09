"""
stats_tracker.py — 包速率 / 数据速率 / 丢包率统计。

每 1 秒打印一次统计信息。
"""

import time
import sys


class StatsTracker:
    def __init__(self):
        self._start_time = time.time()
        self._packet_count = 0
        self._byte_count = 0      # 仅 H264 数据 (299B/包)
        self._gap_count = 0
        self._dup_count = 0
        self._last_seq = None
        self._last_print = time.time()

    def record(self, seq: int, data_len: int):
        """记录一帧: seq = data[0] (1B), data_len = len(data[1:]) = 299"""
        self._packet_count += 1
        self._byte_count += data_len

        if self._last_seq is not None:
            diff = (seq - self._last_seq) & 0xFF
            if diff == 0:
                self._dup_count += 1
            elif diff > 1:
                self._gap_count += (diff - 1)
        self._last_seq = seq

        # 每秒打印
        now = time.time()
        if now - self._last_print >= 1.0:
            self._print_stats(now)
            self._last_print = now

    def _print_stats(self, now: float):
        elapsed = now - self._start_time
        pps = self._packet_count / elapsed if elapsed > 0 else 0
        kbps = (self._byte_count / elapsed) / 1000.0 if elapsed > 0 else 0
        total = self._packet_count + self._gap_count
        loss_rate = (self._gap_count / total * 100) if total > 0 else 0.0

        print(
            f"[stats] packets={self._packet_count} "
            f"rate={pps:.0f}pkt/s {kbps:.1f}kB/s "
            f"gaps={self._gap_count} dups={self._dup_count} "
            f"loss={loss_rate:.1f}%",
            file=sys.stderr,
        )

    def reset(self):
        self.__init__()
