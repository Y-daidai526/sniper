#ifndef SNIPER_PROTOCOL_SERIAL_FRAME_HPP_
#define SNIPER_PROTOCOL_SERIAL_FRAME_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace sniper::protocol {

constexpr uint8_t kSof = 0xA5;
constexpr uint16_t kDataLen = 300;
constexpr uint16_t kCmdId = 0x0310;
constexpr size_t kFrameSize = 309;
constexpr size_t kDataSize = 300;
constexpr size_t kH264SliceSize = 299;
constexpr size_t kHeaderSize = 5;
constexpr size_t kFooterSize = 2;
constexpr size_t kCrc8Span = 4;

using SerialFrame = std::array<uint8_t, kFrameSize>;

SerialFrame build_frame(uint8_t seq, const uint8_t *data_300);

inline uint8_t extract_inner_seq(const uint8_t *data_300) {
    return data_300[0];
}

} // namespace sniper::protocol

#endif
