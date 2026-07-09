#ifndef SNIPER_ENCODER_VIDEO_ENCODER_NODE_HPP_
#define SNIPER_ENCODER_VIDEO_ENCODER_NODE_HPP_

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "doorlock_sniper/msg/video_packet.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <atomic>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace sniper::encoder {

// 回调: 每个 300B 数据包 (1B内部seq + 299B H264) → 串口 / debug bridge
using SerialDataCallback = std::function<void(const uint8_t *data_300)>;

class VideoEncoderNode : public rclcpp::Node {
public:
    explicit VideoEncoderNode(const rclcpp::NodeOptions &options);
    ~VideoEncoderNode() override;

    // 注册串口数据输出回调（main 负责把 300B 送给 serial_writer + debug_bridge）
    void set_serial_data_callback(SerialDataCallback cb) { serial_data_cb_ = std::move(cb); }

private:
    // --- GStreamer ---
    void initialize_gstreamer();
    void shutdown_gstreamer();

    // --- 图像处理 ---
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    cv::Mat preprocess_image(
        const cv::Mat &input,
        cv::Mat *roi_downsample = nullptr,
        cv::Mat *static_removed = nullptr);
    void push_frame_to_gstreamer(const cv::Mat &frame);

    // --- 双路分包 ---
    void pull_stream_and_packetize_dual();

    // --- 显示 ---
    void display_loop();

    // --- GStreamer 元素 ---
    GstElement *pipeline_ = nullptr;
    GstElement *appsrc_ = nullptr;
    GstElement *appsink_ = nullptr;
    GstBus *bus_ = nullptr;

    // --- ROS2 ---
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<doorlock_sniper::msg::VideoPacket>::SharedPtr packet_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_data_pub_;

    // --- 双路 stream buffer ---
    std::vector<uint8_t> ros2_stream_buffer_;     // 150B 切片 → VideoPacket
    std::vector<uint8_t> serial_stream_buffer_;    // 299B 切片 → serial
    std::mutex buffer_mutex_;

    // --- 150B 路径限速 (bandwidth window) ---
    std::deque<std::pair<int64_t, size_t>> sent_window_;
    size_t sent_window_bytes_ = 0;
    uint64_t dropped_bytes_ = 0;
    uint32_t dropped_events_ = 0;

    // --- 300B 路径限速 (独立) ---
    std::deque<std::pair<int64_t, size_t>> serial_sent_window_;
    size_t serial_sent_window_bytes_ = 0;

    // --- 串口输出回调 ---
    SerialDataCallback serial_data_cb_;

    // --- 计数器 ---
    uint64_t packet_sequence_id_ = 0;
    uint8_t serial_seq_ = 0;        // 0-255, data[0] 内部序列号
    uint8_t frame_seq_ = 0;         // 0-255, 串口帧头 sequence
    uint32_t frame_count_ = 0;
    int64_t last_telemetry_ns_ = 0;

    // --- 显示 ---
    std::thread display_thread_;
    std::atomic<bool> display_running_{false};
    std::mutex frame_mutex_;
    cv::Mat display_raw_frame_;
    cv::Mat display_roi_frame_;
    cv::Mat display_static_frame_;
    cv::Mat display_frame_;
    uint64_t display_frame_counter_ = 0;

    // --- 预处理状态 ---
    int64_t last_encode_stamp_ns_ = 0;
    cv::Mat background_gray_f32_;
    cv::Mat motion_erode_kernel_;
    cv::Mat motion_dilate_kernel_;
    std::deque<cv::Mat> motion_mask_history_;
    std::deque<cv::Mat> trail_frame_history_;

    // --- 参数 ---
    int param_crop_size_ = 800;
    int param_output_size_ = 400;
    int param_output_fps_ = 60;
    int param_target_bitrate_ = 40;
    bool param_static_simplify_ = true;
    int param_motion_threshold_ = 14;
    int param_motion_erode_px_ = 1;
    int param_motion_dilate_px_ = 2;
    int param_motion_trail_frames_ = 3;
    double param_trail_disable_motion_ratio_ = 0.30;
    double param_bg_update_alpha_ = 0.01;
    double param_bg_blur_sigma_ = 1.2;
    int param_center_clear_size_ = 100;
    bool param_force_monochrome_ = false;
    double param_bandwidth_limit_kbytes_ = 14.0;
    double param_bandwidth_window_s_ = 2.0;
    double param_max_tx_delay_s_ = 1.0;
    bool param_enable_display_ = true;
    std::string param_x264_preset_ = "auto";
    std::string param_input_topic_;
    bool param_debug_dump_enable_ = false;
    int param_debug_dump_every_n_frames_ = 20;
    bool param_debug_dump_save_raw_ = true;
    bool param_debug_dump_save_roi_ = true;
    bool param_debug_dump_save_static_ = true;
    bool param_debug_dump_save_final_ = true;
    std::string param_debug_dump_dir_ = "sniper_debug_imgs";
};

} // namespace sniper::encoder

#endif
