#ifndef SNIPER_RM_FRAME_ENCODER_NODE_RM_FRAME_HPP_
#define SNIPER_RM_FRAME_ENCODER_NODE_RM_FRAME_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace sniper::protocol {

constexpr uint8_t kSof = 0xA5;
constexpr uint16_t kDataLen = 300;
constexpr uint16_t kCmdId = 0x0310;
constexpr size_t kFrameSize = 309;
constexpr size_t kDataSize = 300;
constexpr size_t kEncodedChunkSize = 299;
constexpr size_t kHeaderSize = 5;
constexpr size_t kFooterSize = 2;
constexpr size_t kCrc8Span = 4;

using RmFrame = std::array<uint8_t, kFrameSize>;

RmFrame build_rm_frame(uint8_t seq, const uint8_t *data_300);

} // namespace sniper::protocol

#endif
