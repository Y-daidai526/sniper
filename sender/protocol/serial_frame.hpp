#ifndef SNIPER_PROTOCOL_SERIAL_FRAME_HPP_
#define SNIPER_PROTOCOL_SERIAL_FRAME_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace sniper::protocol {

// 串口帧结构 (309B total):
//   SOF      1B    0xA5
//   data_len 2B    300 (0x012C) 小端
//   seq      1B    0-255
//   CRC8     1B    覆盖 SOF+data_len+seq (前4B)
//   cmd_id   2B    0x0310 小端
//   data     300B  1B内部seq + 299B H264切片
//   CRC16    2B    覆盖 SOF~data末尾 (前307B) 小端

constexpr uint8_t  kSof           = 0xA5;
constexpr uint16_t kDataLen       = 300;       // 0x012C
constexpr uint16_t kCmdId         = 0x0310;
constexpr size_t   kFrameSize     = 309;       // 总字节
constexpr size_t   kDataSize      = 300;       // data 段长度
constexpr size_t   kH264SliceSize = 299;       // H264 切片长度
constexpr size_t   kHeaderSize    = 5;         // SOF+data_len+seq+CRC8
constexpr size_t   kFooterSize    = 2;         // CRC16
constexpr size_t   kCrc8Span      = 4;         // CRC8 覆盖前4B

// data 段内部:
//   [0]      = 内部 seq (uint8, 0-255)
//   [1..299] = H264 Annex-B 切片

using SerialFrame = std::array<uint8_t, kFrameSize>;

// 构造完整 309B 串口帧
//   h264_data: 指向 300B 数据 (data[0]=内部seq, data[1..299]=H264)
//   输出: 309B 帧 (SOF+data_len+seq+CRC8+cmd_id+data+CRC16)
SerialFrame build_frame(uint8_t seq, const uint8_t *h264_data);

// 提取 data 段中的内部序列号
inline uint8_t extract_inner_seq(const uint8_t *data_300) {
    return data_300[0];
}

} // namespace sniper::protocol

#endif
