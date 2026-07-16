#ifndef SNIPER_SERIAL_SERIAL_SEND_WORKER_HPP_
#define SNIPER_SERIAL_SERIAL_SEND_WORKER_HPP_

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "local_test/local_test_bridge.hpp"
#include "serial/serial_writer.hpp"

namespace sniper::serial {

struct SerialSendConfig {
    bool enable_serial = false;
    bool enable_local_test = false;
    bool local_test_start_broker = false;
    std::string local_test_script_dir;
    std::string local_test_mqtt_host;
    int local_test_mqtt_port = 0;
    std::string local_test_mqtt_topic;
    double max_rate_hz = 0.0;
    double max_tx_delay_s = 0.0;
};

class SerialSendWorker {
public:
    explicit SerialSendWorker(SerialSendConfig config);
    ~SerialSendWorker();

    SerialSendWorker(const SerialSendWorker &) = delete;
    SerialSendWorker &operator=(const SerialSendWorker &) = delete;

    void start();
    void stop();
    void enqueue(const uint8_t *data, size_t size);

private:
    struct ClipReport {
        bool should_log = false;
        size_t dropped = 0;
        size_t backlog = 0;
        uint64_t total_dropped = 0;
    };

    void run();
    ClipReport clip_backlog_locked();

    SerialSendConfig config_;
    SerialWriter serial_writer_;
    sniper::local_test::LocalTestBridge local_test_bridge_;

    std::mutex buffer_mutex_;
    std::condition_variable buffer_cv_;
    std::vector<uint8_t> stream_buffer_;
    bool stop_requested_ = false;
    std::thread send_thread_;

    uint8_t inner_seq_ = 0;
    uint8_t frame_seq_ = 0;
    uint64_t packets_sent_ = 0;
    uint64_t dropped_bytes_ = 0;
    uint32_t drop_events_ = 0;
};

} // namespace sniper::serial

#endif
