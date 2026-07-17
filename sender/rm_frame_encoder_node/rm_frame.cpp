#include "rm_frame_encoder_node/rm_frame.hpp"

#include <cstring>

namespace sniper::protocol {
namespace {

constexpr uint8_t kCrc8Init = 0xFF;
constexpr uint8_t kCrc8PolyReflected = 0x8C;
constexpr uint16_t kCrc16Init = 0xFFFF;
constexpr uint16_t kCrc16PolyReflected = 0x8408;

uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = kCrc8Init;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x01U) != 0U
                ? static_cast<uint8_t>((crc >> 1U) ^ kCrc8PolyReflected)
                : static_cast<uint8_t>(crc >> 1U);
        }
    }
    return crc;
}

uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = kCrc16Init;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x0001U) != 0U
                ? static_cast<uint16_t>((crc >> 1U) ^ kCrc16PolyReflected)
                : static_cast<uint16_t>(crc >> 1U);
        }
    }
    return crc;
}

} // namespace

RmFrame build_rm_frame(uint8_t seq, const uint8_t *data_300) {
    RmFrame frame{};
    frame[0] = kSof;
    frame[1] = static_cast<uint8_t>(kDataLen & 0xFF);
    frame[2] = static_cast<uint8_t>((kDataLen >> 8) & 0xFF);
    frame[3] = seq;
    frame[4] = crc8(frame.data(), kCrc8Span);
    frame[5] = static_cast<uint8_t>(kCmdId & 0xFF);
    frame[6] = static_cast<uint8_t>((kCmdId >> 8) & 0xFF);
    std::memcpy(frame.data() + 7, data_300, kDataSize);

    const uint16_t crc = crc16(frame.data(), kFrameSize - kFooterSize);
    frame[kFrameSize - 2] = static_cast<uint8_t>(crc & 0xFF);
    frame[kFrameSize - 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    return frame;
}

} // namespace sniper::protocol
