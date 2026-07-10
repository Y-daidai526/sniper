#ifndef SNIPER_PROTOCOL_CRC16_HPP_
#define SNIPER_PROTOCOL_CRC16_HPP_

#include <cstddef>
#include <cstdint>

namespace sniper::protocol {

constexpr uint16_t kCrc16Init = 0xFFFF;
constexpr uint16_t kCrc16PolyReflected = 0x8408;  // normal polynomial 0x1021

inline uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = kCrc16Init;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x0001U) != 0U) {
                crc = static_cast<uint16_t>((crc >> 1U) ^ kCrc16PolyReflected);
            } else {
                crc = static_cast<uint16_t>(crc >> 1U);
            }
        }
    }
    return crc;
}

} // namespace sniper::protocol

#endif
