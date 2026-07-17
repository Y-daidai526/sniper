#ifndef SNIPER_SERIAL_SERIAL_SENDER_NODE_HPP_
#define SNIPER_SERIAL_SERIAL_SENDER_NODE_HPP_

#include "rm_frame_encoder_node/rm_frame.hpp"
#include "serial_sender_node/serial_writer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>

namespace sniper::serial {

class SerialSenderNode : public rclcpp::Node {
public:
    explicit SerialSenderNode(const rclcpp::NodeOptions &options);
    ~SerialSenderNode() override;

    SerialSenderNode(const SerialSenderNode &) = delete;
    SerialSenderNode &operator=(const SerialSenderNode &) = delete;

private:
    using Clock = std::chrono::steady_clock;

    void rm_frame_callback(const std_msgs::msg::UInt8MultiArray::ConstSharedPtr msg);
    void send_loop();
    void log_stats();

    bool enabled_ = false;
    double max_rate_hz_ = 0.0;
    size_t max_backlog_frames_ = 0;

    SerialWriter serial_writer_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rm_frame_sub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;

    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<sniper::protocol::RmFrame> frame_queue_;
    bool stop_requested_ = false;
    std::thread send_thread_;

    std::atomic<uint64_t> packets_sent_{0};
    std::atomic<uint64_t> drop_events_{0};
    Clock::time_point last_stats_time_;
    uint64_t last_stats_packets_ = 0;
};

} // namespace sniper::serial

#endif
