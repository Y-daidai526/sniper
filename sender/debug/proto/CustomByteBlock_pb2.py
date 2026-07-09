# -*- coding: utf-8 -*-
"""
Minimal CustomByteBlock protobuf implementation (no protoc required).

Proto definition:
  syntax = "proto3";
  message CustomByteBlock {
      optional bytes data = 1;
  }

Wire format for field 1 (bytes):
  Tag: 0x0A (field=1, wire_type=2)
  Length: varint
  Value: raw bytes
"""

import struct


class CustomByteBlock:
    """Protobuf-compatible CustomByteBlock message."""

    def __init__(self):
        self.data: bytes | None = None

    def HasField(self, name: str) -> bool:
        if name == 'data':
            return self.data is not None
        raise ValueError(f"Unknown field: {name}")

    def SerializeToString(self) -> bytes:
        """Serialize to protobuf wire format."""
        result = bytearray()
        if self.data is not None:
            # Tag: field 1, wire type 2 (length-delimited)
            result.append(0x0A)
            # Varint length
            result.extend(self._encode_varint(len(self.data)))
            # Value
            result.extend(self.data)
        return bytes(result)

    def ParseFromString(self, payload: bytes):
        """Parse from protobuf wire format."""
        pos = 0
        while pos < len(payload):
            tag, pos = self._decode_varint(payload, pos)
            field_number = tag >> 3
            wire_type = tag & 0x07

            if field_number == 1 and wire_type == 2:
                length, pos = self._decode_varint(payload, pos)
                self.data = payload[pos:pos + length]
                pos += length
            else:
                # Skip unknown field
                if wire_type == 0:  # varint
                    _, pos = self._decode_varint(payload, pos)
                elif wire_type == 2:  # length-delimited
                    length, pos = self._decode_varint(payload, pos)
                    pos += length
                elif wire_type == 5:  # 32-bit
                    pos += 4
                elif wire_type == 1:  # 64-bit
                    pos += 8
                else:
                    raise ValueError(f"Unknown wire type: {wire_type}")

    @staticmethod
    def _encode_varint(value: int) -> bytes:
        result = bytearray()
        while value > 0x7F:
            result.append((value & 0x7F) | 0x80)
            value >>= 7
        result.append(value & 0x7F)
        return bytes(result)

    @staticmethod
    def _decode_varint(buf: bytes, pos: int) -> tuple:
        value = 0
        shift = 0
        while pos < len(buf):
            byte = buf[pos]
            pos += 1
            value |= (byte & 0x7F) << shift
            shift += 7
            if not (byte & 0x80):
                break
        return value, pos
