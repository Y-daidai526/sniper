#ifndef SNIPER_SERIAL_SERIAL_WRITER_HPP_
#define SNIPER_SERIAL_SERIAL_WRITER_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace sniper::serial {

// 串口参数
constexpr int kSerialBaudRate = 921600;
constexpr int kSerialDataBits = 8;   // CS8

// USB CDC 热插拔 + termios 写
class SerialWriter {
public:
    // 状态回调：设备插入/拔出时通知
    using StatusCallback = std::function<void(const std::string &device, bool connected)>;

    SerialWriter();
    ~SerialWriter();

    // 启动热插拔监控线程
    void start(StatusCallback cb = nullptr);

    // 停止监控，关闭串口
    void stop();

    // 写 309B 帧到串口，未连接时静默丢弃
    void write_frame(const uint8_t *frame, size_t len);

    // 查询连接状态
    bool is_connected() const { return fd_ >= 0; }

private:
    void monitor_loop();
    bool try_open(const std::string &device);
    void close_current();
    std::string find_device();  // 扫描 /dev/ttyACM*, /dev/ttyUSB*

    int fd_ = -1;
    std::string current_device_;
    std::thread monitor_thread_;
    std::atomic<bool> running_{false};
    std::mutex write_mutex_;
    StatusCallback status_cb_;
};

} // namespace sniper::serial

#endif
