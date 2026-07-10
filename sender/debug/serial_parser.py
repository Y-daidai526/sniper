#!/usr/bin/env python3

import struct

SOF = 0xA5
CMD_ID = 0x0310
DATA_SIZE = 300
FRAME_SIZE = 309
CRC8_POLY_REFLECTED = 0x8C
CRC16_POLY_REFLECTED = 0x8408


def crc8(data: bytes) -> int:
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = ((crc >> 1) ^ CRC8_POLY_REFLECTED) & 0xFF
            else:
                crc = (crc >> 1) & 0xFF
    return crc


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = ((crc >> 1) ^ CRC16_POLY_REFLECTED) & 0xFFFF
            else:
                crc = (crc >> 1) & 0xFFFF
    return crc


def parse_frame(frame: bytes) -> tuple[bool, bytes | None, int | None, str]:
    if len(frame) != FRAME_SIZE:
        return False, None, None, f"wrong size: {len(frame)} != {FRAME_SIZE}"
    if frame[0] != SOF:
        return False, None, None, f"bad SOF: 0x{frame[0]:02X}"

    data_len = struct.unpack_from("<H", frame, 1)[0]
    if data_len != DATA_SIZE:
        return False, None, None, f"bad data_len: {data_len}"

    seq = frame[3]
    expected_crc8 = frame[4]
    actual_crc8 = crc8(frame[:4])
    if actual_crc8 != expected_crc8:
        return False, None, None, f"CRC8 mismatch: got 0x{actual_crc8:02X}, expected 0x{expected_crc8:02X}"

    cmd_id = struct.unpack_from("<H", frame, 5)[0]
    if cmd_id != CMD_ID:
        return False, None, None, f"bad cmd_id: 0x{cmd_id:04X}"

    expected_crc16 = struct.unpack_from("<H", frame, FRAME_SIZE - 2)[0]
    actual_crc16 = crc16(frame[:FRAME_SIZE - 2])
    if actual_crc16 != expected_crc16:
        return False, None, None, f"CRC16 mismatch: got 0x{actual_crc16:04X}, expected 0x{expected_crc16:04X}"

    return True, frame[7:307], seq, ""
