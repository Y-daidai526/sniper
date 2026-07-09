#include "video_encoder_node.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace sniper::encoder {

namespace {
    constexpr int kVideoPacketBytes = 150;   // VideoPacket.msg 硬约束, 兼容原解码器
    constexpr int kSerialSliceBytes  = 299;   // 299B H264 + 1B seq = 300B → 0x0310 data 段
    constexpr int kSerialDataBytes   = 300;   // = 1B seq + 299B H264
} // namespace

VideoEncoderNode::VideoEncoderNode(const rclcpp::NodeOptions &options)
    : Node("video_encoder_node", options) {

    // --- 参数声明 ---
    param_input_topic_ = this->declare_parameter("input_topic", "/image_raw");
    param_crop_size_ = this->declare_parameter("crop_size", 800);
    param_output_size_ = this->declare_parameter("output_size", 400);
    param_output_fps_ = this->declare_parameter("output_fps", 60);
    param_target_bitrate_ = this->declare_parameter("target_bitrate", 40);
    param_static_simplify_ = this->declare_parameter("static_simplify", true);
    param_motion_threshold_ = this->declare_parameter("motion_threshold", 14);
    param_motion_erode_px_ = this->declare_parameter("motion_erode_px", 1);
    param_motion_dilate_px_ = this->declare_parameter("motion_dilate_px", 2);
    param_motion_trail_frames_ = this->declare_parameter("motion_trail_frames", 3);
    param_trail_disable_motion_ratio_ =
        this->declare_parameter("trail_disable_motion_ratio", 0.30);
    param_bg_update_alpha_ = this->declare_parameter("bg_update_alpha", 0.01);
    param_bg_blur_sigma_ = this->declare_parameter("bg_blur_sigma", 1.2);
    param_center_clear_size_ = this->declare_parameter("center_clear_size", 100);
    param_force_monochrome_ = this->declare_parameter("force_monochrome", false);
    param_bandwidth_limit_kbytes_ =
        this->declare_parameter("bandwidth_limit_kbytes", 7.0);
    param_bandwidth_window_s_ = this->declare_parameter("bandwidth_window_s", 2.0);
    param_max_tx_delay_s_ = this->declare_parameter("max_tx_delay_s", 1.0);
    param_enable_display_ = this->declare_parameter("enable_display", true);
    param_x264_preset_ = this->declare_parameter("x264_preset", std::string("auto"));
    param_debug_dump_enable_ = this->declare_parameter("debug_dump_enable", false);
    param_debug_dump_every_n_frames_ =
        this->declare_parameter("debug_dump_every_n_frames", 20);
    param_debug_dump_save_raw_ = this->declare_parameter("debug_dump_save_raw", true);
    param_debug_dump_save_roi_ = this->declare_parameter("debug_dump_save_roi", true);
    param_debug_dump_save_static_ =
        this->declare_parameter("debug_dump_save_static", true);
    param_debug_dump_save_final_ =
        this->declare_parameter("debug_dump_save_final", true);
    param_debug_dump_dir_ =
        this->declare_parameter("debug_dump_dir", std::string("sniper_debug_imgs"));

    // --- 参数范围校验 ---
    if (param_output_fps_ < 1) {
        RCLCPP_WARN(this->get_logger(), "Invalid output_fps=%d, clamp to 1", param_output_fps_);
        param_output_fps_ = 1;
    }
    if (param_output_fps_ > 60) {
        RCLCPP_WARN(this->get_logger(), "output_fps=%d too high, clamp to 60", param_output_fps_);
        param_output_fps_ = 60;
    }
    if (param_target_bitrate_ < 200) {
        RCLCPP_WARN(this->get_logger(),
            "Very low bitrate (%d kbps) detected, using low-bitrate optimized pipeline",
            param_target_bitrate_);
    }
    if (param_motion_trail_frames_ < 0) {
        RCLCPP_WARN(this->get_logger(), "motion_trail_frames=%d invalid, clamp to 0",
            param_motion_trail_frames_);
        param_motion_trail_frames_ = 0;
    }
    if (param_motion_trail_frames_ > 15) {
        RCLCPP_WARN(this->get_logger(), "motion_trail_frames=%d too high, clamp to 15",
            param_motion_trail_frames_);
        param_motion_trail_frames_ = 15;
    }
    if (param_trail_disable_motion_ratio_ < 0.0) {
        RCLCPP_WARN(this->get_logger(),
            "trail_disable_motion_ratio=%.3f invalid, clamp to 0.0",
            param_trail_disable_motion_ratio_);
        param_trail_disable_motion_ratio_ = 0.0;
    }
    if (param_trail_disable_motion_ratio_ > 1.0) {
        RCLCPP_WARN(this->get_logger(),
            "trail_disable_motion_ratio=%.3f invalid, clamp to 1.0",
            param_trail_disable_motion_ratio_);
        param_trail_disable_motion_ratio_ = 1.0;
    }
    if (param_motion_erode_px_ < 0) {
        RCLCPP_WARN(this->get_logger(), "motion_erode_px=%d invalid, clamp to 0",
            param_motion_erode_px_);
        param_motion_erode_px_ = 0;
    }
    if (param_motion_erode_px_ > 20) {
        RCLCPP_WARN(this->get_logger(), "motion_erode_px=%d too high, clamp to 20",
            param_motion_erode_px_);
        param_motion_erode_px_ = 20;
    }
    if (param_motion_dilate_px_ < 0) {
        RCLCPP_WARN(this->get_logger(), "motion_dilate_px=%d invalid, clamp to 0",
            param_motion_dilate_px_);
        param_motion_dilate_px_ = 0;
    }
    if (param_motion_dilate_px_ > 20) {
        RCLCPP_WARN(this->get_logger(), "motion_dilate_px=%d too high, clamp to 20",
            param_motion_dilate_px_);
        param_motion_dilate_px_ = 20;
    }
    if (param_bandwidth_limit_kbytes_ < 1.0) {
        RCLCPP_WARN(this->get_logger(),
            "bandwidth_limit_kbytes=%.2f too low, clamp to 1.0",
            param_bandwidth_limit_kbytes_);
        param_bandwidth_limit_kbytes_ = 1.0;
    }
    if (param_bandwidth_window_s_ < 0.2) {
        RCLCPP_WARN(this->get_logger(),
            "bandwidth_window_s=%.2f too low, clamp to 0.2",
            param_bandwidth_window_s_);
        param_bandwidth_window_s_ = 0.2;
    }
    if (param_max_tx_delay_s_ < 0.05) {
        RCLCPP_WARN(this->get_logger(),
            "max_tx_delay_s=%.2f too low, clamp to 0.05",
            param_max_tx_delay_s_);
        param_max_tx_delay_s_ = 0.05;
    }
    if (param_debug_dump_every_n_frames_ < 1) {
        RCLCPP_WARN(this->get_logger(),
            "debug_dump_every_n_frames=%d invalid, clamp to 1",
            param_debug_dump_every_n_frames_);
        param_debug_dump_every_n_frames_ = 1;
    }
    if (param_debug_dump_enable_) {
        const bool any_encoder_save =
            param_debug_dump_save_raw_ || param_debug_dump_save_roi_ ||
            param_debug_dump_save_static_ || param_debug_dump_save_final_;
        if (!any_encoder_save) {
            RCLCPP_WARN(this->get_logger(),
                "debug_dump_enable=true but all encoder dump switches are off");
        } else {
            const std::filesystem::path dump_dir =
                std::filesystem::path(param_debug_dump_dir_) / "encoder";
            std::error_code ec;
            std::filesystem::create_directories(dump_dir, ec);
            if (ec) {
                RCLCPP_WARN(this->get_logger(),
                    "Create debug dump dir failed: %s (%s), disable debug dump",
                    dump_dir.string().c_str(), ec.message().c_str());
                param_debug_dump_enable_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(),
                    "Debug dump enabled: every %d frames -> %s",
                    param_debug_dump_every_n_frames_, dump_dir.string().c_str());
            }
        }
    }

    // --- ROS2 订阅/发布 ---
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        param_input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&VideoEncoderNode::image_callback, this, std::placeholders::_1));

    packet_pub_ = this->create_publisher<doorlock_sniper::msg::VideoPacket>(
        "video_stream", rclcpp::QoS(rclcpp::KeepLast(3000)).reliable());

    serial_data_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "serial_data", rclcpp::QoS(rclcpp::KeepLast(3000)).reliable());

    // --- GStreamer ---
    initialize_gstreamer();

    // --- 显示 ---
    if (param_enable_display_) {
        display_running_ = true;
        display_thread_ = std::thread(&VideoEncoderNode::display_loop, this);
    }

    RCLCPP_INFO(this->get_logger(),
        "VideoEncoderNode: crop=%d -> %dx%d@%dfps %dkbps, "
        "dual-output: 150B(VideoPacket/ros2) + 299B(serial 0x0310), "
        "tx_limit=%.2fkB/s@%.2fs max_delay=%.2fs x264_preset=%s",
        param_crop_size_, param_output_size_, param_output_size_,
        param_output_fps_, param_target_bitrate_,
        param_bandwidth_limit_kbytes_, param_bandwidth_window_s_,
        param_max_tx_delay_s_, param_x264_preset_.c_str());
}

VideoEncoderNode::~VideoEncoderNode() {
    if (param_enable_display_) {
        display_running_ = false;
        if (display_thread_.joinable()) display_thread_.join();
        cv::destroyAllWindows();
    }
    shutdown_gstreamer();
}

// ============================================================================
// GStreamer pipeline (same as original)
// ============================================================================

void VideoEncoderNode::initialize_gstreamer() {
    gst_init(nullptr, nullptr);

    pipeline_ = gst_pipeline_new("encoder_pipe");
    appsrc_ = gst_element_factory_make("appsrc", "source");
    appsink_ = gst_element_factory_make("appsink", "sink");
    GstElement *convert = gst_element_factory_make("videoconvert", "convert");
    GstElement *encoder = gst_element_factory_make("x264enc", "encoder");
    GstElement *parser = gst_element_factory_make("h264parse", "parser");

    if (!pipeline_ || !appsrc_ || !appsink_ || !convert || !encoder || !parser) {
        RCLCPP_FATAL(this->get_logger(), "GStreamer element creation failed");
        return;
    }

    // appsrc caps
    GstCaps *caps = gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "BGR",
        "width", G_TYPE_INT, param_output_size_,
        "height", G_TYPE_INT, param_output_size_,
        "framerate", GST_TYPE_FRACTION, param_output_fps_, 1,
        nullptr);
    g_object_set(G_OBJECT(appsrc_),
        "caps", caps,
        "stream-type", 0,
        "format", GST_FORMAT_TIME,
        "is-live", TRUE,
        "do-timestamp", TRUE,
        nullptr);
    gst_caps_unref(caps);

    // x264enc configuration (dual-mode: low-bitrate vs low-latency)
    const bool low_bitrate_mode = (param_target_bitrate_ <= 80);
    const int key_int = std::max(8 * param_output_fps_, 30);
    const int default_speed_preset = low_bitrate_mode ? 9 : 3;

    int speed_preset = default_speed_preset;
    std::string preset_lower = param_x264_preset_;
    std::transform(preset_lower.begin(), preset_lower.end(), preset_lower.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (!preset_lower.empty() && preset_lower != "auto") {
        if (preset_lower == "ultrafast") speed_preset = 1;
        else if (preset_lower == "superfast") speed_preset = 2;
        else if (preset_lower == "veryfast") speed_preset = 3;
        else if (preset_lower == "faster") speed_preset = 4;
        else if (preset_lower == "fast") speed_preset = 5;
        else if (preset_lower == "medium") speed_preset = 6;
        else if (preset_lower == "slow") speed_preset = 7;
        else if (preset_lower == "slower") speed_preset = 8;
        else if (preset_lower == "veryslow") speed_preset = 9;
        else if (preset_lower == "placebo") speed_preset = 10;
        else {
            RCLCPP_WARN(this->get_logger(),
                "Unknown x264_preset='%s', fallback to auto default",
                param_x264_preset_.c_str());
            speed_preset = default_speed_preset;
        }
    }

    if (low_bitrate_mode) {
        g_object_set(G_OBJECT(encoder),
            "bitrate", param_target_bitrate_,
            "speed-preset", speed_preset,
            "tune", 0,
            "byte-stream", TRUE,
            "key-int-max", key_int,
            "bframes", 4,
            "rc-lookahead", 40,
            "sync-lookahead", 20,
            "sliced-threads", FALSE,
            "ref", 5,
            "aud", TRUE,
            "vbv-buf-capacity", 500,
            "option-string",
                "repeat-headers=1:scenecut=0:aq-mode=2:aq-strength=1.2:"
                "mbtree=1:qcomp=0.75:subme=8:trellis=2:deblock=1,1:force-cfr=1",
            "pass", 0,
            nullptr);
    } else {
        g_object_set(G_OBJECT(encoder),
            "bitrate", param_target_bitrate_,
            "speed-preset", speed_preset,
            "tune", 0x00000004,           // zerolatency
            "byte-stream", TRUE,
            "key-int-max", 2 * param_output_fps_,
            "bframes", 0,
            "rc-lookahead", 0,
            "sync-lookahead", 0,
            "sliced-threads", TRUE,
            "aud", TRUE,
            "option-string", "repeat-headers=1:scenecut=0:ref=1:force-cfr=1",
            "pass", 0,
            nullptr);
    }

    // h264parse: byte-stream output with periodic SPS/PPS
    g_object_set(G_OBJECT(parser),
        "config-interval", -1,
        "disable-passthrough", TRUE,
        nullptr);

    // appsink caps
    GstCaps *h264_caps = gst_caps_new_simple(
        "video/x-h264",
        "stream-format", G_TYPE_STRING, "byte-stream",
        "alignment", G_TYPE_STRING, "au",
        nullptr);
    g_object_set(G_OBJECT(appsink_),
        "caps", h264_caps,
        "max-buffers", 5,
        "drop", FALSE,
        "emit-signals", FALSE,
        "sync", FALSE,
        nullptr);
    gst_caps_unref(h264_caps);

    // link pipeline
    gst_bin_add_many(GST_BIN(pipeline_), appsrc_, convert, encoder, parser, appsink_, nullptr);
    if (!gst_element_link_many(appsrc_, convert, encoder, parser, appsink_, nullptr)) {
        RCLCPP_FATAL(this->get_logger(), "GStreamer pipeline link failed");
        return;
    }

    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_FATAL(this->get_logger(), "GStreamer pipeline start failed");
        return;
    }

    bus_ = gst_element_get_bus(pipeline_);
    RCLCPP_INFO(this->get_logger(),
        "GStreamer encoder ready (%s mode, byte-stream)",
        low_bitrate_mode ? "low-bitrate" : "low-latency");
}

void VideoEncoderNode::shutdown_gstreamer() {
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        if (bus_) gst_object_unref(bus_);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
}

// ============================================================================
// 图像预处理 (same as original)
// ============================================================================

cv::Mat VideoEncoderNode::preprocess_image(
    const cv::Mat &input, cv::Mat *roi_downsample, cv::Mat *static_removed) {
    int x = (input.cols - param_crop_size_) / 2;
    int y = (input.rows - param_crop_size_) / 2;
    x = std::max(0, x);
    y = std::max(0, y);
    int w = std::min(param_crop_size_, input.cols - x);
    int h = std::min(param_crop_size_, input.rows - y);

    cv::Mat cropped = input(cv::Rect(x, y, w, h));
    cv::Mat resized;
    cv::resize(cropped, resized, cv::Size(param_output_size_, param_output_size_),
        0, 0, cv::INTER_LINEAR);
    if (roi_downsample) {
        resized.copyTo(*roi_downsample);
    }
    cv::Mat working = resized;
    if (param_force_monochrome_) {
        cv::Mat gray_full;
        cv::cvtColor(working, gray_full, cv::COLOR_BGR2GRAY);
        cv::cvtColor(gray_full, working, cv::COLOR_GRAY2BGR);
    }

    if (!param_static_simplify_) {
        if (static_removed) working.copyTo(*static_removed);
        return working;
    }

    cv::Mat gray;
    cv::cvtColor(working, gray, cv::COLOR_BGR2GRAY);
    if (background_gray_f32_.empty()) {
        gray.convertTo(background_gray_f32_, CV_32F);
        return working;
    }

    cv::Mat bg_u8;
    cv::convertScaleAbs(background_gray_f32_, bg_u8);

    cv::Mat diff;
    cv::absdiff(gray, bg_u8, diff);

    cv::Mat motion_mask;
    cv::threshold(diff, motion_mask, param_motion_threshold_, 255, cv::THRESH_BINARY);
    if (param_motion_erode_px_ > 0) {
        if (motion_erode_kernel_.empty()) {
            const int k = 2 * param_motion_erode_px_ + 1;
            motion_erode_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
        }
        cv::erode(motion_mask, motion_mask, motion_erode_kernel_, cv::Point(-1, -1), 1);
    }
    if (param_motion_dilate_px_ > 0) {
        if (motion_dilate_kernel_.empty()) {
            const int k = 2 * param_motion_dilate_px_ + 1;
            motion_dilate_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
        }
        cv::dilate(motion_mask, motion_mask, motion_dilate_kernel_, cv::Point(-1, -1), 1);
    }
    const double motion_ratio_raw =
        static_cast<double>(cv::countNonZero(motion_mask)) /
        static_cast<double>(motion_mask.total());
    const bool suppress_trail = (motion_ratio_raw >= param_trail_disable_motion_ratio_);

    // 中心区域保护
    if (param_center_clear_size_ > 0) {
        const int clear_size =
            std::min({param_center_clear_size_, working.cols, working.rows});
        const int x0 = std::max(0, working.cols / 2 - clear_size / 2);
        const int y0 = std::max(0, working.rows / 2 - clear_size / 2);
        const int cw = std::min(clear_size, working.cols - x0);
        const int ch = std::min(clear_size, working.rows - y0);
        cv::rectangle(motion_mask, cv::Rect(x0, y0, cw, ch), cv::Scalar(255), cv::FILLED);
    }

    cv::Mat static_base = working.clone();
    if (!param_force_monochrome_ && param_target_bitrate_ <= 80) {
        cv::Mat gray_bg;
        cv::cvtColor(static_base, gray_bg, cv::COLOR_BGR2GRAY);
        cv::cvtColor(gray_bg, static_base, cv::COLOR_GRAY2BGR);
    }
    cv::Mat blurred_static;
    cv::GaussianBlur(static_base, blurred_static, cv::Size(),
        std::max(0.0, param_bg_blur_sigma_), std::max(0.0, param_bg_blur_sigma_));

    cv::Mat focused = blurred_static.clone();
    working.copyTo(focused, motion_mask);
    if (static_removed) focused.copyTo(*static_removed);

    // 运动拖影
    if (param_motion_trail_frames_ > 0) {
        motion_mask_history_.push_back(motion_mask.clone());
        trail_frame_history_.push_back(working.clone());
        const size_t max_history = static_cast<size_t>(param_motion_trail_frames_ + 1);
        while (motion_mask_history_.size() > max_history) motion_mask_history_.pop_front();
        while (trail_frame_history_.size() > max_history) trail_frame_history_.pop_front();

        const size_t history_size = motion_mask_history_.size();
        if (!suppress_trail && history_size > 1 && history_size == trail_frame_history_.size()) {
            cv::Mat trail_mask = motion_mask.clone();
            cv::Mat trail_img = working.clone();
            for (size_t i = 0; i < history_size - 1; ++i) {
                cv::bitwise_or(trail_mask, motion_mask_history_[i], trail_mask);
                cv::max(trail_img, trail_frame_history_[i], trail_img);
            }
            trail_img.copyTo(focused, trail_mask);
        }
    } else {
        motion_mask_history_.clear();
        trail_frame_history_.clear();
    }

    cv::accumulateWeighted(gray, background_gray_f32_,
        std::clamp(param_bg_update_alpha_, 0.001, 0.2));
    return focused;
}

void VideoEncoderNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        if (param_output_fps_ < 60) {
            const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
            const int64_t frame_interval_ns = 1000000000LL / std::max(param_output_fps_, 1);
            const int64_t now_ns = (stamp_ns > 0) ? stamp_ns : this->now().nanoseconds();
            if (last_encode_stamp_ns_ > 0 && (now_ns - last_encode_stamp_ns_) < frame_interval_ns) {
                return;
            }
            last_encode_stamp_ns_ = now_ns;
        }

        cv::Mat input = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat roi_downsample;
        cv::Mat static_removed;
        cv::Mat processed = preprocess_image(input, &roi_downsample, &static_removed);

        if (param_enable_display_) {
            cv::Mat raw_preview;
            cv::resize(input, raw_preview,
                cv::Size(std::max(1, input.cols / 2), std::max(1, input.rows / 2)),
                0, 0, cv::INTER_AREA);
            std::lock_guard<std::mutex> lock(frame_mutex_);
            raw_preview.copyTo(display_raw_frame_);
            roi_downsample.copyTo(display_roi_frame_);
            static_removed.copyTo(display_static_frame_);
            processed.copyTo(display_frame_);
        }

        push_frame_to_gstreamer(processed);
        pull_stream_and_packetize_dual();

        frame_count_++;
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    }
}

void VideoEncoderNode::push_frame_to_gstreamer(const cv::Mat &frame) {
    if (!appsrc_ || frame.empty()) return;

    size_t size = frame.total() * frame.elemSize();
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);

    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        memcpy(map.data, frame.data, size);
        gst_buffer_unmap(buffer, &map);

        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
        if (ret != GST_FLOW_OK) {
            RCLCPP_WARN(this->get_logger(), "Push buffer failed: %d", ret);
        }
    }
    gst_buffer_unref(buffer);
}

// ============================================================================
// 双路分包（核心改造）
// ============================================================================

void VideoEncoderNode::pull_stream_and_packetize_dual() {
    if (!appsink_) return;

    // --- 150B 路径限速参数 ---
    const size_t ros2_packet_bytes = static_cast<size_t>(kVideoPacketBytes);
    const int64_t window_ns = static_cast<int64_t>(param_bandwidth_window_s_ * 1e9);
    const size_t window_limit_bytes = static_cast<size_t>(
        param_bandwidth_limit_kbytes_ * 1000.0 * param_bandwidth_window_s_);
    const size_t max_backlog_bytes = static_cast<size_t>(
        param_bandwidth_limit_kbytes_ * 1000.0 * param_max_tx_delay_s_);

    // --- 300B 路径限速参数（独立） ---
    const size_t serial_slice_bytes = static_cast<size_t>(kSerialSliceBytes);
    const size_t serial_window_limit_bytes = window_limit_bytes;  // same limit
    const size_t serial_max_backlog = max_backlog_bytes;           // same backlog

    while (true) {
        GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), 0);
        if (!sample) break;

        GstBuffer *buffer = gst_sample_get_buffer(sample);
        if (!buffer) {
            gst_sample_unref(sample);
            continue;
        }

        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            std::lock_guard<std::mutex> lock(buffer_mutex_);

            // --- 追加到两个独立 buffer ---
            size_t ros2_old = ros2_stream_buffer_.size();
            ros2_stream_buffer_.resize(ros2_old + map.size);
            memcpy(ros2_stream_buffer_.data() + ros2_old, map.data, map.size);

            size_t serial_old = serial_stream_buffer_.size();
            serial_stream_buffer_.resize(serial_old + map.size);
            memcpy(serial_stream_buffer_.data() + serial_old, map.data, map.size);

            // ================================================================
            // 150B 路径 → VideoPacket publish (兼容原解码器)
            // ================================================================
            while (ros2_stream_buffer_.size() >= ros2_packet_bytes) {
                const int64_t now_ns = this->now().nanoseconds();

                // 滑动窗口限速
                while (!sent_window_.empty() &&
                       (now_ns - sent_window_.front().first) > window_ns) {
                    sent_window_bytes_ -= sent_window_.front().second;
                    sent_window_.pop_front();
                }

                if (sent_window_bytes_ + ros2_packet_bytes > window_limit_bytes) {
                    break;  // 等窗口滑出
                }

                doorlock_sniper::msg::VideoPacket pkt;
                pkt.sequence_id = packet_sequence_id_++;
                pkt.timestamp_ns = now_ns;

                pkt.data.fill(0);
                memcpy(pkt.data.data(), ros2_stream_buffer_.data(), kVideoPacketBytes);

                packet_pub_->publish(pkt);
                sent_window_.emplace_back(now_ns, ros2_packet_bytes);
                sent_window_bytes_ += ros2_packet_bytes;

                // consume
                memmove(ros2_stream_buffer_.data(),
                    ros2_stream_buffer_.data() + kVideoPacketBytes,
                    ros2_stream_buffer_.size() - kVideoPacketBytes);
                ros2_stream_buffer_.resize(
                    ros2_stream_buffer_.size() - kVideoPacketBytes);
            }

            // 150B backlog clipping
            if (ros2_stream_buffer_.size() > max_backlog_bytes) {
                const size_t target_drop =
                    ros2_stream_buffer_.size() - max_backlog_bytes;
                size_t drop_bytes = target_drop;
                for (size_t i = target_drop;
                     i + 4 < ros2_stream_buffer_.size(); ++i) {
                    const bool sc3 = (ros2_stream_buffer_[i] == 0 &&
                                      ros2_stream_buffer_[i + 1] == 0 &&
                                      ros2_stream_buffer_[i + 2] == 1);
                    const bool sc4 = (ros2_stream_buffer_[i] == 0 &&
                                      ros2_stream_buffer_[i + 1] == 0 &&
                                      ros2_stream_buffer_[i + 2] == 0 &&
                                      ros2_stream_buffer_[i + 3] == 1);
                    if (sc3 || sc4) { drop_bytes = i; break; }
                }
                memmove(ros2_stream_buffer_.data(),
                    ros2_stream_buffer_.data() + drop_bytes,
                    ros2_stream_buffer_.size() - drop_bytes);
                ros2_stream_buffer_.resize(
                    ros2_stream_buffer_.size() - drop_bytes);
                dropped_bytes_ += drop_bytes;
                dropped_events_++;
                if (dropped_events_ % 20 == 1) {
                    RCLCPP_WARN(this->get_logger(),
                        "ROS2 backlog clipped: dropped=%zuB total_dropped=%luB",
                        drop_bytes, dropped_bytes_);
                }
            }

            // ================================================================
            // 299B 路径 → serial (串口 + debug bridge)
            // ================================================================
            while (serial_stream_buffer_.size() >= serial_slice_bytes) {
                const int64_t now_ns = this->now().nanoseconds();

                // 独立滑动窗口限速
                while (!serial_sent_window_.empty() &&
                       (now_ns - serial_sent_window_.front().first) > window_ns) {
                    serial_sent_window_bytes_ -= serial_sent_window_.front().second;
                    serial_sent_window_.pop_front();
                }

                if (serial_sent_window_bytes_ + kSerialDataBytes >
                    serial_window_limit_bytes) {
                    break;  // 等窗口滑出
                }

                // 构造 300B data: [0]=seq, [1..299]=H264
                uint8_t data_300[kSerialDataBytes];
                data_300[0] = serial_seq_++;
                memcpy(data_300 + 1,
                    serial_stream_buffer_.data(), kSerialSliceBytes);

                // 传给 serial_writer + debug_bridge
                if (serial_data_cb_) {
                    serial_data_cb_(data_300);
                }

                // 同时发布到 ROS2 topic → MQTT bridge 订阅
                {
                    auto msg = std_msgs::msg::UInt8MultiArray();
                    msg.data.assign(data_300, data_300 + kSerialDataBytes);
                    serial_data_pub_->publish(msg);
                }

                serial_sent_window_.emplace_back(now_ns, kSerialDataBytes);
                serial_sent_window_bytes_ += kSerialDataBytes;

                // consume 299B
                memmove(serial_stream_buffer_.data(),
                    serial_stream_buffer_.data() + kSerialSliceBytes,
                    serial_stream_buffer_.size() - kSerialSliceBytes);
                serial_stream_buffer_.resize(
                    serial_stream_buffer_.size() - kSerialSliceBytes);
            }

            // 299B backlog clipping
            if (serial_stream_buffer_.size() > serial_max_backlog) {
                const size_t target_drop =
                    serial_stream_buffer_.size() - serial_max_backlog;
                size_t drop_bytes = target_drop;
                for (size_t i = target_drop;
                     i + 4 < serial_stream_buffer_.size(); ++i) {
                    const bool sc3 = (serial_stream_buffer_[i] == 0 &&
                                      serial_stream_buffer_[i + 1] == 0 &&
                                      serial_stream_buffer_[i + 2] == 1);
                    const bool sc4 = (serial_stream_buffer_[i] == 0 &&
                                      serial_stream_buffer_[i + 1] == 0 &&
                                      serial_stream_buffer_[i + 2] == 0 &&
                                      serial_stream_buffer_[i + 3] == 1);
                    if (sc3 || sc4) { drop_bytes = i; break; }
                }
                memmove(serial_stream_buffer_.data(),
                    serial_stream_buffer_.data() + drop_bytes,
                    serial_stream_buffer_.size() - drop_bytes);
                serial_stream_buffer_.resize(
                    serial_stream_buffer_.size() - drop_bytes);
            }

            // --- 遥测 ---
            const int64_t telemetry_ns = this->now().nanoseconds();
            if (telemetry_ns - last_telemetry_ns_ > 1000000000LL) {
                const double ros2_window_kb =
                    static_cast<double>(sent_window_bytes_) / 1000.0;
                const double serial_window_kb =
                    static_cast<double>(serial_sent_window_bytes_) / 1000.0;
                RCLCPP_INFO(this->get_logger(),
                    "TX: ros2=%.2fkB serial=%.2fkB (limit=%.2fkB) "
                    "backlog ros2=%zuB serial=%zuB",
                    ros2_window_kb, serial_window_kb,
                    static_cast<double>(window_limit_bytes) / 1000.0,
                    ros2_stream_buffer_.size(), serial_stream_buffer_.size());
                last_telemetry_ns_ = telemetry_ns;
            }

            gst_buffer_unmap(buffer, &map);
        }
        gst_sample_unref(sample);
    }
}

// ============================================================================
// 显示 (same as original)
// ============================================================================

void VideoEncoderNode::display_loop() {
    cv::namedWindow("Doorlock Sniper Raw", cv::WINDOW_NORMAL);
    cv::namedWindow("Doorlock Sniper ROI", cv::WINDOW_NORMAL);
    cv::namedWindow("Doorlock Sniper Static", cv::WINDOW_NORMAL);
    cv::namedWindow("Doorlock Sniper", cv::WINDOW_NORMAL);
    cv::setWindowProperty("Doorlock Sniper Raw",
        cv::WND_PROP_ASPECT_RATIO, cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("Doorlock Sniper ROI",
        param_output_size_, param_output_size_);
    cv::resizeWindow("Doorlock Sniper Static",
        param_output_size_, param_output_size_);
    cv::resizeWindow("Doorlock Sniper",
        param_output_size_, param_output_size_);

    while (display_running_ && rclcpp::ok()) {
        cv::Mat raw_frame, roi_frame, static_frame, frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (!display_raw_frame_.empty()) display_raw_frame_.copyTo(raw_frame);
            if (!display_roi_frame_.empty()) display_roi_frame_.copyTo(roi_frame);
            if (!display_static_frame_.empty()) display_static_frame_.copyTo(static_frame);
            if (!display_frame_.empty()) display_frame_.copyTo(frame);
        }

        if (!raw_frame.empty()) cv::imshow("Doorlock Sniper Raw", raw_frame);
        if (!roi_frame.empty()) cv::imshow("Doorlock Sniper ROI", roi_frame);
        if (!static_frame.empty()) cv::imshow("Doorlock Sniper Static", static_frame);
        if (!frame.empty()) cv::imshow("Doorlock Sniper", frame);

        if (param_debug_dump_enable_ && !frame.empty()) {
            display_frame_counter_++;
            if ((display_frame_counter_ %
                 static_cast<uint64_t>(param_debug_dump_every_n_frames_)) == 0U) {
                const std::filesystem::path dump_dir =
                    std::filesystem::path(param_debug_dump_dir_) / "encoder";
                std::ostringstream idx;
                idx << std::setw(8) << std::setfill('0') << display_frame_counter_;
                const std::string frame_id = idx.str();
                if (param_debug_dump_save_raw_ && !raw_frame.empty())
                    cv::imwrite((dump_dir / ("raw_" + frame_id + ".png")).string(), raw_frame);
                if (param_debug_dump_save_roi_ && !roi_frame.empty())
                    cv::imwrite((dump_dir / ("roi_" + frame_id + ".png")).string(), roi_frame);
                if (param_debug_dump_save_static_ && !static_frame.empty())
                    cv::imwrite((dump_dir / ("static_" + frame_id + ".png")).string(), static_frame);
                if (param_debug_dump_save_final_)
                    cv::imwrite((dump_dir / ("final_" + frame_id + ".png")).string(), frame);
            }
        }
        cv::waitKey(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    cv::destroyWindow("Doorlock Sniper Raw");
    cv::destroyWindow("Doorlock Sniper ROI");
    cv::destroyWindow("Doorlock Sniper Static");
    cv::destroyWindow("Doorlock Sniper");
}

} // namespace sniper::encoder

RCLCPP_COMPONENTS_REGISTER_NODE(sniper::encoder::VideoEncoderNode)
