#include "serial_frame.hpp"

#include "crc16.hpp"
#include "crc8.hpp"

#include <cstring>

namespace sniper::protocol {

SerialFrame build_frame(uint8_t seq, const uint8_t *data_300) {
    SerialFrame frame{};

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
