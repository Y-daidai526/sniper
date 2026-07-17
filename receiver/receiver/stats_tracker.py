import sys
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class PacketStatus:
    duplicate: bool
    gap: int


class StatsTracker:
    def __init__(self):
        self._last_print = time.monotonic()
        self._packet_count = 0
        self._byte_count = 0
        self._gap_count = 0
        self._dup_count = 0
        self._error_count = 0
        self._last_print_packet_count = 0
        self._last_print_byte_count = 0
        self._last_seq: int | None = None

    def record_error(self) -> None:
        self._error_count += 1

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

        now = time.monotonic()
        if now - self._last_print >= 1.0:
            self._print(now)
            self._last_print = now

        return PacketStatus(duplicate=duplicate, gap=gap)

    def _print(self, now: float) -> None:
        elapsed = max(now - self._last_print, 1e-6)
        packet_delta = self._packet_count - self._last_print_packet_count
        byte_delta = self._byte_count - self._last_print_byte_count
        packet_rate = packet_delta / elapsed
        data_rate = (byte_delta / elapsed) / 1000.0
        print(
            f"[receiver stats]: rate={packet_rate:.1f}pkt/s data={data_rate:.2f}kB/s "
            f"packets={self._packet_count} gaps={self._gap_count} dups={self._dup_count} errors={self._error_count}",
            file=sys.stderr,
        )
        self._last_print_packet_count = self._packet_count
        self._last_print_byte_count = self._byte_count
