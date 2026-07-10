#ifndef SNIPER_SERIAL_SERIAL_WRITER_HPP_
#define SNIPER_SERIAL_SERIAL_WRITER_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace sniper::serial {

class SerialWriter {
public:
    using StatusCallback = std::function<void(const std::string &device, bool connected)>;

    SerialWriter();
    ~SerialWriter();

    void start(StatusCallback cb);
    void stop();
    bool write_frame(const uint8_t *frame, size_t len);
    std::string current_device() const;

private:
    void monitor_loop();
    bool try_open(const std::string &device);
    void close_current();
    void close_current_locked();
    std::string find_device();

    int fd_ = -1;
    std::string current_device_;
    std::thread monitor_thread_;
    std::atomic<bool> running_{false};
    mutable std::mutex write_mutex_;
    StatusCallback status_cb_;
};

} // namespace sniper::serial

#endif
