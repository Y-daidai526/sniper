#include "serial_frame.hpp"
#include "crc8.hpp"
#include "crc16.hpp"
#include <cstring>

namespace sniper::protocol {

SerialFrame build_frame(uint8_t seq, const uint8_t *h264_data) {
    SerialFrame frame{};

    // SOF
    frame[0] = kSof;  // 0xA5

    // data_len (2B, 小端)
    frame[1] = static_cast<uint8_t>(kDataLen & 0xFF);
    frame[2] = static_cast<uint8_t>((kDataLen >> 8) & 0xFF);

    // seq (1B) — 串口帧头 sequence，与 data[0] 内部 seq 不同
    frame[3] = seq;

    // CRC8 覆盖 SOF+data_len+seq (前4B)
    frame[4] = crc8(frame.data(), kCrc8Span);

    // cmd_id (2B, 小端) — 0x0310
    frame[5] = static_cast<uint8_t>(kCmdId & 0xFF);
    frame[6] = static_cast<uint8_t>((kCmdId >> 8) & 0xFF);

    // data (300B) — 1B内部seq + 299B H264
    std::memcpy(frame.data() + 7, h264_data, kDataSize);

    // CRC16 覆盖 SOF~data 末尾 (前 307B)，小端
    uint16_t crc = crc16(frame.data(), kFrameSize - kFooterSize);
    frame[kFrameSize - 2] = static_cast<uint8_t>(crc & 0xFF);
    frame[kFrameSize - 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    return frame;
}

} // namespace sniper::protocol
