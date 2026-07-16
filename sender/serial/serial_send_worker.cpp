#include "serial_send_worker.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <utility>

#include "protocol/serial_frame.hpp"

namespace sniper::serial {
namespace {

size_t align_drop_to_annexb_start(const std::vector<uint8_t> &buffer, size_t target_drop) {
    for (size_t i = target_drop; i + 4 < buffer.size(); ++i) {
        const bool start_code_3 = buffer[i] == 0 && buffer[i + 1] == 0 && buffer[i + 2] == 1;
        const bool start_code_4 =
            buffer[i] == 0 && buffer[i + 1] == 0 && buffer[i + 2] == 0 && buffer[i + 3] == 1;
        if (start_code_3 || start_code_4) {
            return i;
        }
    }
    return target_drop;
}

} // namespace

SerialSendWorker::SerialSendWorker(SerialSendConfig config)
    : config_(std::move(config)) {}

SerialSendWorker::~SerialSendWorker() {
    stop();
}

void SerialSendWorker::start() {
    if (send_thread_.joinable()) {
        return;
    }

    if (config_.enable_serial) {
        serial_writer_.start([](const std::string &device, bool connected) {
            if (connected) {
                std::fprintf(stdout, "[sender] serial connected: %s\n", device.c_str());
            } else {
                std::fprintf(stdout, "[sender] serial disconnected: %s\n", device.c_str());
            }
        });
    }

    if (config_.enable_local_test) {
        const std::filesystem::path script_path =
            std::filesystem::path(config_.local_test_script_dir) / "main.py";
        if (!std::filesystem::exists(script_path)) {
            RCLCPP_WARN(
                rclcpp::get_logger("sender_runtime"),
                "local_test bridge disabled: %s not found",
                script_path.string().c_str());
        } else if (!local_test_bridge_.start(
                       config_.local_test_script_dir,
                       config_.local_test_mqtt_host,
                       config_.local_test_mqtt_port,
                       config_.local_test_mqtt_topic,
                       config_.local_test_start_broker)) {
            RCLCPP_ERROR(rclcpp::get_logger("sender_runtime"), "failed to start local_test bridge");
        }
    }

    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        stop_requested_ = false;
    }
    send_thread_ = std::thread(&SerialSendWorker::run, this);
}

void SerialSendWorker::stop() {
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        stop_requested_ = true;
    }
    buffer_cv_.notify_one();
    if (send_thread_.joinable()) {
        send_thread_.join();
    }
    local_test_bridge_.stop();
    serial_writer_.stop();
}

void SerialSendWorker::enqueue(const uint8_t *data, size_t size) {
    if (!data || size == 0) {
        return;
    }
    ClipReport report;
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        const size_t old_size = stream_buffer_.size();
        stream_buffer_.resize(old_size + size);
        std::memcpy(stream_buffer_.data() + old_size, data, size);
        report = clip_backlog_locked();
    }
    if (report.should_log) {
        RCLCPP_WARN(
            rclcpp::get_logger("serial stats"),
            "serial backlog clipped: dropped=%zuB backlog=%zuB total_dropped=%luB",
            report.dropped,
            report.backlog,
            report.total_dropped);
    }
    buffer_cv_.notify_one();
}

SerialSendWorker::ClipReport SerialSendWorker::clip_backlog_locked() {
    const size_t max_backlog = static_cast<size_t>(
        std::max(1.0, config_.max_rate_hz) *
        static_cast<double>(sniper::protocol::kH264SliceSize) *
        config_.max_tx_delay_s);
    if (stream_buffer_.size() <= max_backlog) {
        return {};
    }

    const size_t target_drop = stream_buffer_.size() - max_backlog;
    const size_t drop_bytes = align_drop_to_annexb_start(stream_buffer_, target_drop);
    stream_buffer_.erase(
        stream_buffer_.begin(),
        stream_buffer_.begin() + static_cast<std::ptrdiff_t>(drop_bytes));
    dropped_bytes_ += drop_bytes;
    drop_events_++;
    return {drop_events_ % 20 == 1, drop_bytes, stream_buffer_.size(), dropped_bytes_};
}

void SerialSendWorker::run() {
    using Clock = std::chrono::steady_clock;
    const auto period = std::chrono::nanoseconds(
        static_cast<int64_t>(1000000000.0 / config_.max_rate_hz));
    auto next_tx = Clock::now();
    auto last_stats = Clock::now();
    uint64_t last_stats_packets = 0;

    while (true) {
        std::array<uint8_t, sniper::protocol::kDataSize> data{};
        size_t backlog = 0;
        uint32_t drops = 0;
        bool send_packet = false;
        bool print_stats = false;
        double stats_elapsed_s = 0.0;

        {
            std::unique_lock<std::mutex> lock(buffer_mutex_);
            const auto now = Clock::now();
            if (stop_requested_) {
                break;
            }

            if (stream_buffer_.size() >= sniper::protocol::kH264SliceSize && now >= next_tx) {
                data[0] = inner_seq_++;
                std::memcpy(
                    data.data() + 1,
                    stream_buffer_.data(),
                    sniper::protocol::kH264SliceSize);
                stream_buffer_.erase(
                    stream_buffer_.begin(),
                    stream_buffer_.begin() + static_cast<std::ptrdiff_t>(sniper::protocol::kH264SliceSize));
                send_packet = true;
                next_tx = now + period;
            }

            if (now - last_stats >= std::chrono::seconds(1)) {
                print_stats = true;
                stats_elapsed_s = std::chrono::duration<double>(now - last_stats).count();
                backlog = stream_buffer_.size();
                drops = drop_events_;
                last_stats = now;
            }

            if (!send_packet && !print_stats) {
                auto wake_time = last_stats + std::chrono::seconds(1);
                if (stream_buffer_.size() >= sniper::protocol::kH264SliceSize) {
                    wake_time = std::min(wake_time, next_tx);
                }
                buffer_cv_.wait_until(lock, wake_time);
                continue;
            }
        }

        if (send_packet) {
            const auto frame = sniper::protocol::build_frame(frame_seq_++, data.data());
            if (config_.enable_serial) {
                serial_writer_.write_frame(frame.data(), frame.size());
            }
            if (config_.enable_local_test && local_test_bridge_.is_running()) {
                local_test_bridge_.write_frame(frame.data(), frame.size());
            }
            packets_sent_++;
        }

        if (print_stats) {
            const uint64_t packet_delta = packets_sent_ - last_stats_packets;
            const double rate_hz = stats_elapsed_s > 0.0
                ? static_cast<double>(packet_delta) / stats_elapsed_s
                : 0.0;
            const std::string current_device = serial_writer_.current_device();
            const std::string port = config_.enable_serial
                ? (current_device.empty() ? "disconnected" : current_device)
                : "disabled";
            RCLCPP_INFO(
                rclcpp::get_logger("serial stats"),
                "rate=%.1fpkt/s data=%.2fkB/s packets=%lu drops=%u backlog=%zuB port=%s",
                rate_hz,
                rate_hz * static_cast<double>(sniper::protocol::kH264SliceSize) / 1000.0,
                packets_sent_,
                drops,
                backlog,
                port.c_str());
            last_stats_packets = packets_sent_;
        }
    }
}

} // namespace sniper::serial
