#ifndef SNIPER_ENCODER_VIDEO_ENCODER_NODE_HPP_
#define SNIPER_ENCODER_VIDEO_ENCODER_NODE_HPP_

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <array>
#include <atomic>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace sniper::encoder {

using SerialStreamCallback = std::function<void(const uint8_t *data, size_t size)>;

class VideoEncoderNode : public rclcpp::Node {
public:
    explicit VideoEncoderNode(const rclcpp::NodeOptions &options);
    ~VideoEncoderNode() override;

    void set_serial_stream_callback(SerialStreamCallback cb);
    double serial_max_rate_hz() const { return param_serial_max_rate_hz_; }
    double max_tx_delay_s() const { return param_max_tx_delay_s_; }

private:
    void initialize_gstreamer();
    void shutdown_gstreamer();

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    cv::Mat preprocess_image(
        const cv::Mat &input,
        cv::Mat *roi_downsample,
        cv::Mat *static_removed);
    void push_frame_to_gstreamer(const cv::Mat &frame);
    void pull_stream_and_packetize();

    void publish_video_stream_packets(
        int64_t now_ns,
        int64_t window_ns,
        size_t window_limit_bytes,
        size_t max_packets_this_pull);
    void publish_video_packet(const uint8_t *payload_150, int64_t timestamp_ns);
    rclcpp::SerializedMessage serialize_video_packet(
        uint64_t sequence_id,
        uint64_t timestamp_ns,
        const uint8_t *payload_150) const;
    void clip_video_stream_backlog();

    void display_loop();

    GstElement *pipeline_ = nullptr;
    GstElement *appsrc_ = nullptr;
    GstElement *appsink_ = nullptr;
    GstBus *bus_ = nullptr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::shared_ptr<rclcpp::GenericPublisher> video_packet_pub_;

    std::vector<uint8_t> ros2_stream_buffer_;
    std::deque<std::pair<int64_t, size_t>> sent_window_;
    std::mutex buffer_mutex_;
    SerialStreamCallback serial_stream_cb_;

    uint64_t video_packet_sequence_id_ = 0;
    int64_t last_encode_stamp_ns_ = 0;
    int64_t last_telemetry_ns_ = 0;
    uint64_t last_telemetry_ros2_packets_ = 0;
    uint64_t video_stream_dropped_bytes_ = 0;
    uint32_t video_stream_drop_events_ = 0;
    uint64_t ros2_packets_sent_ = 0;
    size_t sent_window_bytes_ = 0;

    std::thread display_thread_;
    std::atomic<bool> display_running_{false};
    std::mutex frame_mutex_;
    cv::Mat display_raw_frame_;
    cv::Mat display_roi_frame_;
    cv::Mat display_static_frame_;
    cv::Mat display_frame_;

    cv::Mat background_gray_f32_;
    cv::Mat motion_erode_kernel_;
    cv::Mat motion_dilate_kernel_;
    std::deque<cv::Mat> motion_mask_history_;
    std::deque<cv::Mat> trail_frame_history_;

    std::string param_input_topic_;
    std::string param_video_stream_topic_;
    int param_crop_size_ = 0;
    int param_output_size_ = 0;
    int param_output_fps_ = 0;
    int param_target_bitrate_ = 0;
    bool param_static_simplify_ = false;
    bool param_enable_video_stream_ = false;
    int param_motion_threshold_ = 0;
    int param_motion_erode_px_ = 0;
    int param_motion_dilate_px_ = 0;
    int param_motion_trail_frames_ = 0;
    double param_trail_disable_motion_ratio_ = 0.0;
    double param_bg_update_alpha_ = 0.0;
    double param_bg_blur_sigma_ = 0.0;
    int param_center_clear_size_ = 0;
    bool param_force_monochrome_ = false;
    double param_bandwidth_limit_kbytes_ = 0.0;
    double param_bandwidth_window_s_ = 0.0;
    double param_serial_max_rate_hz_ = 0.0;
    double param_max_tx_delay_s_ = 0.0;
    bool param_enable_display_ = false;
    std::string param_x264_preset_;
};

} // namespace sniper::encoder

#endif
