#ifndef SNIPER_PROTOCOL_CRC8_HPP_
#define SNIPER_PROTOCOL_CRC8_HPP_

#include <cstddef>
#include <cstdint>

namespace sniper::protocol {

constexpr uint8_t kCrc8Init = 0xFF;
constexpr uint8_t kCrc8PolyReflected = 0x8C;  // normal polynomial 0x31

inline uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = kCrc8Init;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x01U) != 0U) {
                crc = static_cast<uint8_t>((crc >> 1U) ^ kCrc8PolyReflected);
            } else {
                crc = static_cast<uint8_t>(crc >> 1U);
            }
        }
    }
    return crc;
}

} // namespace sniper::protocol

#endif
