import sys
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class PacketStatus:
    duplicate: bool
    gap: int


class StatsTracker:
    def __init__(self, print_interval_s: float):
        self._print_interval_s = print_interval_s
        self._start_time = time.time()
        self._last_print = self._start_time
        self._packet_count = 0
        self._byte_count = 0
        self._gap_count = 0
        self._dup_count = 0
        self._last_seq: int | None = None

    def record(self, seq: int, payload_len: int) -> PacketStatus:
        duplicate = False
        gap = 0

        if self._last_seq is not None:
            diff = (seq - self._last_seq) & 0xFF
            if diff == 0:
                duplicate = True
                self._dup_count += 1
            elif diff > 1:
                gap = diff - 1
                self._gap_count += gap

        if not duplicate:
            self._packet_count += 1
            self._byte_count += payload_len
            self._last_seq = seq

        now = time.time()
        if now - self._last_print >= self._print_interval_s:
            self._print(now)
            self._last_print = now

        return PacketStatus(duplicate=duplicate, gap=gap)

    def _print(self, now: float) -> None:
        elapsed = max(now - self._start_time, 1e-6)
        packet_rate = self._packet_count / elapsed
        data_rate = (self._byte_count / elapsed) / 1000.0
        total_expected = self._packet_count + self._gap_count
        loss_rate = (self._gap_count / total_expected * 100.0) if total_expected > 0 else 0.0
        print(
            f"[stats] packets={self._packet_count} rate={packet_rate:.1f}pkt/s "
            f"data={data_rate:.2f}kB/s gaps={self._gap_count} dups={self._dup_count} loss={loss_rate:.2f}%",
            file=sys.stderr,
        )
