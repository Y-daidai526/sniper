#include "rm_frame_encoder_node/rm_frame_encoder_node.hpp"

#include "rm_frame_encoder_node/rm_frame.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <functional>
#include <memory>
#include <utility>

namespace sniper::protocol {

RmFrameEncoderNode::RmFrameEncoderNode(const rclcpp::NodeOptions &options)
    : Node("rm_frame_encoder", options) {
    rm_frame_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("rm_frame", 10);
    encoded_stream_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        "encoded_stream",
        10,
        std::bind(
            &RmFrameEncoderNode::encoded_stream_callback,
            this,
            std::placeholders::_1));
}

void RmFrameEncoderNode::encoded_stream_callback(
    const std_msgs::msg::UInt8MultiArray::ConstSharedPtr msg) {
    if (msg->data.empty()) {
        return;
    }

    stream_buffer_.insert(stream_buffer_.end(), msg->data.begin(), msg->data.end());

    size_t consumed = 0;
    while (stream_buffer_.size() - consumed >= kEncodedChunkSize) {
        std::array<uint8_t, kDataSize> data{};
        data[0] = inner_seq_++;
        std::copy_n(stream_buffer_.begin() + static_cast<std::ptrdiff_t>(consumed),
                    kEncodedChunkSize,
                    data.begin() + 1);

        const auto frame = build_rm_frame(frame_seq_++, data.data());
        std_msgs::msg::UInt8MultiArray output;
        output.data.assign(frame.begin(), frame.end());
        rm_frame_pub_->publish(std::move(output));
        consumed += kEncodedChunkSize;
    }

    if (consumed > 0) {
        stream_buffer_.erase(
            stream_buffer_.begin(),
            stream_buffer_.begin() + static_cast<std::ptrdiff_t>(consumed));
    }
}

} // namespace sniper::protocol

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<sniper::protocol::RmFrameEncoderNode>(rclcpp::NodeOptions{});
        rclcpp::spin(node);
        node.reset();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "[rm_frame_encoder] fatal error: %s\n", error.what());
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
