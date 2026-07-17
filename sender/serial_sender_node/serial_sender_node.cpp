#include "serial_sender_node/serial_sender_node.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace sniper::serial {
namespace {

bool require_bool_parameter(rclcpp::Node &node, const std::string &name) {
    node.declare_parameter(name, rclcpp::ParameterType::PARAMETER_BOOL);
    return node.get_parameter(name).as_bool();
}

double require_double_parameter(rclcpp::Node &node, const std::string &name) {
    node.declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE);
    return node.get_parameter(name).as_double();
}

int require_int_parameter(rclcpp::Node &node, const std::string &name) {
    node.declare_parameter(name, rclcpp::ParameterType::PARAMETER_INTEGER);
    return static_cast<int>(node.get_parameter(name).as_int());
}

} // namespace

SerialSenderNode::SerialSenderNode(const rclcpp::NodeOptions &options)
    : Node("serial_sender", options), last_stats_time_(Clock::now()) {
    enabled_ = require_bool_parameter(*this, "enabled");
    max_rate_hz_ = require_double_parameter(*this, "max_rate_hz");
    const int max_backlog_frames = require_int_parameter(*this, "max_backlog_frames");

    if (max_rate_hz_ <= 0.0 || max_rate_hz_ > 50.0) {
        RCLCPP_WARN(
            get_logger(),
            "max_rate_hz=%.2f invalid, clamped to 50",
            max_rate_hz_);
        max_rate_hz_ = 50.0;
    }
    if (max_backlog_frames <= 0) {
        throw std::invalid_argument("max_backlog_frames must be greater than zero");
    }
    max_backlog_frames_ = static_cast<size_t>(max_backlog_frames);

    rm_frame_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        "rm_frame",
        rclcpp::QoS(rclcpp::KeepLast(max_backlog_frames_)),
        std::bind(&SerialSenderNode::rm_frame_callback, this, std::placeholders::_1));
    stats_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { log_stats(); });

    if (enabled_) {
        serial_writer_.start();
    }
    send_thread_ = std::thread(&SerialSenderNode::send_loop, this);
}

SerialSenderNode::~SerialSenderNode() {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        stop_requested_ = true;
    }
    queue_cv_.notify_one();
    if (send_thread_.joinable()) {
        send_thread_.join();
    }
    serial_writer_.stop();
}

void SerialSenderNode::rm_frame_callback(
    const std_msgs::msg::UInt8MultiArray::ConstSharedPtr msg) {
    if (msg->data.size() != sniper::protocol::kFrameSize) {
        RCLCPP_WARN(
            get_logger(),
            "ignored RM frame with invalid size: %zu",
            msg->data.size());
        return;
    }

    sniper::protocol::RmFrame frame{};
    std::copy(msg->data.begin(), msg->data.end(), frame.begin());
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        frame_queue_.push_back(std::move(frame));
        if (frame_queue_.size() >= max_backlog_frames_) {
            // A full queue is stale as a whole; the newly received frame is discarded too.
            frame_queue_.clear();
            drop_events_.fetch_add(1, std::memory_order_relaxed);
        }
    }
    queue_cv_.notify_one();
}

void SerialSenderNode::send_loop() {
    const auto period = std::chrono::nanoseconds(
        static_cast<int64_t>(1000000000.0 / max_rate_hz_));
    auto next_tx = Clock::now();

    while (true) {
        sniper::protocol::RmFrame frame{};
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]() {
                return stop_requested_ || !frame_queue_.empty();
            });
            if (stop_requested_) {
                return;
            }

            const auto now = Clock::now();
            if (now < next_tx) {
                queue_cv_.wait_until(lock, next_tx, [this]() { return stop_requested_; });
                continue;
            }

            frame = std::move(frame_queue_.front());
            frame_queue_.pop_front();
            next_tx = Clock::now() + period;
        }

        // write_frame also waits for tcdrain; a disconnected writer returns immediately.
        serial_writer_.write_frame(frame.data(), frame.size());
        packets_sent_.fetch_add(1, std::memory_order_relaxed);
    }
}

void SerialSenderNode::log_stats() {
    const auto now = Clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_stats_time_).count();
    const uint64_t packets = packets_sent_.load(std::memory_order_relaxed);
    const uint64_t packet_delta = packets - last_stats_packets_;
    const double rate = elapsed > 0.0 ? static_cast<double>(packet_delta) / elapsed : 0.0;

    size_t backlog = 0;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        backlog = frame_queue_.size();
    }

    const std::string device = serial_writer_.current_device();
    const std::string port = enabled_
        ? (device.empty() ? "disconnected" : device)
        : "disabled";
    RCLCPP_INFO(
        rclcpp::get_logger("serial stats"),
        "rate=%.1fpkt/s data=%.2fkB/s packets=%" PRIu64
        " drops=%" PRIu64 " backlog=%zuframes port=%s",
        rate,
        rate * static_cast<double>(sniper::protocol::kEncodedChunkSize) / 1000.0,
        packets,
        drop_events_.load(std::memory_order_relaxed),
        backlog,
        port.c_str());

    last_stats_time_ = now;
    last_stats_packets_ = packets;
}

} // namespace sniper::serial

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<sniper::serial::SerialSenderNode>(rclcpp::NodeOptions{});
        rclcpp::spin(node);
        node.reset();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "[serial_sender] fatal error: %s\n", error.what());
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 1;
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
