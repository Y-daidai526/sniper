#ifndef SNIPER_RM_FRAME_ENCODER_NODE_RM_FRAME_ENCODER_NODE_HPP_
#define SNIPER_RM_FRAME_ENCODER_NODE_RM_FRAME_ENCODER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <cstdint>
#include <vector>

namespace sniper::protocol {

class RmFrameEncoderNode : public rclcpp::Node {
public:
    explicit RmFrameEncoderNode(const rclcpp::NodeOptions &options);

private:
    void encoded_stream_callback(const std_msgs::msg::UInt8MultiArray::ConstSharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr encoded_stream_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr rm_frame_pub_;
    std::vector<uint8_t> stream_buffer_;
    uint8_t inner_seq_ = 0;
    uint8_t frame_seq_ = 0;
};

} // namespace sniper::protocol

#endif
