#ifndef SNIPER_ENCODER_VIDEO_ENCODER_NODE_HPP_
#define SNIPER_ENCODER_VIDEO_ENCODER_NODE_HPP_

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

namespace sniper::encoder {

class VideoEncoderNode : public rclcpp::Node {
public:
    VideoEncoderNode(double frame_rate, const rclcpp::NodeOptions &options);
    ~VideoEncoderNode() override;

private:
    void initialize_gstreamer();
    void shutdown_gstreamer();

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    cv::Mat preprocess_image(
        const cv::Mat &input,
        cv::Mat *roi_downsample,
        cv::Mat *static_removed);
    void push_frame_to_gstreamer(const cv::Mat &frame);
    void pull_encoded_stream();

    cv::Mat build_preview(
        const cv::Mat &raw_frame,
        const cv::Mat &roi_frame,
        const cv::Mat &static_frame,
        const cv::Mat &encoder_frame) const;
    void display_callback();

    GstElement *pipeline_ = nullptr;
    GstElement *appsrc_ = nullptr;
    GstElement *appsink_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr encoded_stream_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr preview_pub_;

    rclcpp::TimerBase::SharedPtr display_timer_;
    cv::Mat preview_frame_;

    cv::Mat background_gray_f32_;
    cv::Mat motion_erode_kernel_;
    cv::Mat motion_dilate_kernel_;
    std::deque<cv::Mat> motion_mask_history_;
    std::deque<cv::Mat> trail_frame_history_;

    double frame_rate_ = 0.0;
    int param_crop_size_ = 0;
    int param_output_size_ = 0;
    int param_target_bitrate_ = 0;
    bool param_static_simplify_ = false;
    int param_motion_threshold_ = 0;
    int param_motion_erode_px_ = 0;
    int param_motion_dilate_px_ = 0;
    int param_motion_trail_frames_ = 0;
    double param_trail_disable_motion_ratio_ = 0.0;
    double param_bg_update_alpha_ = 0.0;
    double param_bg_blur_sigma_ = 0.0;
    int param_center_clear_size_ = 0;
    bool param_force_monochrome_ = false;
    bool show_ = false;
    std::string param_x264_preset_;
};

} // namespace sniper::encoder

#endif
