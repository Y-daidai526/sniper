#include "video_encoder_node.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>

namespace sniper::encoder {

namespace {
constexpr size_t kVideoPacketPayloadBytes = 150;
constexpr size_t kSerialSliceBytes = 299;
constexpr size_t kSerialDataBytes = 300;
constexpr size_t kCdrHeaderBytes = 4;
constexpr size_t kSerializedVideoPacketBytes = 172;

bool require_bool_parameter(rclcpp::Node &node, const std::string &name) {
    node.declare_parameter(name, rclcpp::ParameterType::PARAMETER_BOOL);
    return node.get_parameter(name).as_bool();
}

int require_int_parameter(rclcpp::Node &node, const std::string &name) {
    node.declare_parameter(name, rclcpp::ParameterType::PARAMETER_INTEGER);
    return static_cast<int>(node.get_parameter(name).as_int());
}

double require_double_parameter(rclcpp::Node &node, const std::string &name) {
    node.declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE);
    return node.get_parameter(name).as_double();
}

std::string require_string_parameter(rclcpp::Node &node, const std::string &name) {
    node.declare_parameter(name, rclcpp::ParameterType::PARAMETER_STRING);
    return node.get_parameter(name).as_string();
}

void write_u64_le(std::array<uint8_t, kSerializedVideoPacketBytes> &bytes, size_t &offset, uint64_t value) {
    for (size_t i = 0; i < 8; ++i) {
        bytes[offset + i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFFU);
    }
    offset += 8;
}

void append_bytes(std::vector<uint8_t> &buffer, const uint8_t *data, size_t size) {
    const size_t old_size = buffer.size();
    buffer.resize(old_size + size);
    std::memcpy(buffer.data() + old_size, data, size);
}

void erase_front(std::vector<uint8_t> &buffer, size_t count) {
    buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(count));
}

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

VideoEncoderNode::VideoEncoderNode(const rclcpp::NodeOptions &options)
    : Node("video_encoder", options) {
    param_input_topic_ = require_string_parameter(*this, "input_topic");
    param_video_stream_topic_ = require_string_parameter(*this, "video_stream_topic");
    param_crop_size_ = require_int_parameter(*this, "crop_size");
    param_output_size_ = require_int_parameter(*this, "output_size");
    param_output_fps_ = require_int_parameter(*this, "output_fps");
    param_target_bitrate_ = require_int_parameter(*this, "target_bitrate");
    param_x264_preset_ = require_string_parameter(*this, "x264_preset");
    param_enable_video_stream_ = require_bool_parameter(*this, "enable_video_stream");
    param_static_simplify_ = require_bool_parameter(*this, "static_simplify");
    param_motion_threshold_ = require_int_parameter(*this, "motion_threshold");
    param_motion_erode_px_ = require_int_parameter(*this, "motion_erode_px");
    param_motion_dilate_px_ = require_int_parameter(*this, "motion_dilate_px");
    param_motion_trail_frames_ = require_int_parameter(*this, "motion_trail_frames");
    param_trail_disable_motion_ratio_ = require_double_parameter(*this, "trail_disable_motion_ratio");
    param_bg_update_alpha_ = require_double_parameter(*this, "bg_update_alpha");
    param_bg_blur_sigma_ = require_double_parameter(*this, "bg_blur_sigma");
    param_center_clear_size_ = require_int_parameter(*this, "center_clear_size");
    param_force_monochrome_ = require_bool_parameter(*this, "force_monochrome");
    param_bandwidth_limit_kbytes_ = require_double_parameter(*this, "bandwidth_limit_kbytes");
    param_bandwidth_window_s_ = require_double_parameter(*this, "bandwidth_window_s");
    param_serial_max_rate_hz_ = require_double_parameter(*this, "serial_max_rate_hz");
    param_max_tx_delay_s_ = require_double_parameter(*this, "max_tx_delay_s");
    param_enable_display_ = require_bool_parameter(*this, "enable_display");

    if (param_output_fps_ < 1) {
        RCLCPP_WARN(this->get_logger(), "output_fps=%d invalid, clamped to 1", param_output_fps_);
        param_output_fps_ = 1;
    }
    if (param_output_fps_ > 60) {
        RCLCPP_WARN(this->get_logger(), "output_fps=%d exceeds encoder design, clamped to 60", param_output_fps_);
        param_output_fps_ = 60;
    }
    if (param_motion_trail_frames_ < 0) {
        param_motion_trail_frames_ = 0;
    }
    if (param_motion_trail_frames_ > 15) {
        RCLCPP_WARN(this->get_logger(), "motion_trail_frames too high, clamped to 15");
        param_motion_trail_frames_ = 15;
    }
    param_trail_disable_motion_ratio_ = std::clamp(param_trail_disable_motion_ratio_, 0.0, 1.0);
    param_motion_erode_px_ = std::clamp(param_motion_erode_px_, 0, 20);
    param_motion_dilate_px_ = std::clamp(param_motion_dilate_px_, 0, 20);
    param_bg_update_alpha_ = std::clamp(param_bg_update_alpha_, 0.001, 0.2);
    if (param_bandwidth_limit_kbytes_ < 1.0) {
        RCLCPP_WARN(this->get_logger(), "bandwidth_limit_kbytes=%.2f too low, clamped to 1.0", param_bandwidth_limit_kbytes_);
        param_bandwidth_limit_kbytes_ = 1.0;
    }
    if (param_bandwidth_window_s_ < 0.2) {
        RCLCPP_WARN(this->get_logger(), "bandwidth_window_s=%.2f too low, clamped to 0.2", param_bandwidth_window_s_);
        param_bandwidth_window_s_ = 0.2;
    }
    if (param_serial_max_rate_hz_ <= 0.0 || param_serial_max_rate_hz_ > 50.0) {
        RCLCPP_WARN(this->get_logger(), "serial_max_rate_hz=%.2f invalid, clamped to 50", param_serial_max_rate_hz_);
        param_serial_max_rate_hz_ = 50.0;
    }
    if (param_max_tx_delay_s_ < 0.05) {
        param_max_tx_delay_s_ = 0.05;
    }

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        param_input_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&VideoEncoderNode::image_callback, this, std::placeholders::_1));

    if (param_enable_video_stream_) {
        try {
            video_packet_pub_ = this->create_generic_publisher(
                param_video_stream_topic_,
                "doorlock_sniper/msg/VideoPacket",
                rclcpp::QoS(rclcpp::KeepLast(3000)).reliable());
            RCLCPP_INFO(
                this->get_logger(),
                "VideoPacket compatibility publisher enabled: topic=%s type=doorlock_sniper/msg/VideoPacket",
                param_video_stream_topic_.c_str());
        } catch (const std::exception &e) {
            RCLCPP_WARN(
                this->get_logger(),
                "VideoPacket compatibility publisher disabled: missing doorlock_sniper/msg/VideoPacket type support or incompatible ROS environment (%s). 0x0310 serial/local_test/MQTT path is still enabled.",
                e.what());
            param_enable_video_stream_ = false;
        }
    }

    initialize_gstreamer();

    if (param_enable_display_) {
        display_running_ = true;
        display_thread_ = std::thread(&VideoEncoderNode::display_loop, this);
    }

    RCLCPP_INFO(
        this->get_logger(),
        "VideoEncoderNode ready: crop=%d output=%dx%d@%dfps bitrate=%dkbps serial=%.2fHz video_topic=%s",
        param_crop_size_,
        param_output_size_,
        param_output_size_,
        param_output_fps_,
        param_target_bitrate_,
        param_serial_max_rate_hz_,
        param_video_stream_topic_.c_str());
}

VideoEncoderNode::~VideoEncoderNode() {
    if (param_enable_display_) {
        display_running_ = false;
        if (display_thread_.joinable()) {
            display_thread_.join();
        }
        cv::destroyAllWindows();
    }
    shutdown_gstreamer();
}

void VideoEncoderNode::set_serial_data_callback(SerialDataCallback cb) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    serial_data_cb_ = std::move(cb);
}

void VideoEncoderNode::set_serial_status_provider(SerialStatusProvider provider) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    serial_status_provider_ = std::move(provider);
}

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

    GstCaps *caps = gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "BGR",
        "width", G_TYPE_INT, param_output_size_,
        "height", G_TYPE_INT, param_output_size_,
        "framerate", GST_TYPE_FRACTION, param_output_fps_, 1,
        nullptr);
    g_object_set(
        G_OBJECT(appsrc_),
        "caps", caps,
        "stream-type", 0,
        "format", GST_FORMAT_TIME,
        "is-live", TRUE,
        "do-timestamp", TRUE,
        nullptr);
    gst_caps_unref(caps);

    const bool low_bitrate_mode = (param_target_bitrate_ <= 80);
    const int key_int = std::max(8 * param_output_fps_, 30);
    const int default_speed_preset = low_bitrate_mode ? 9 : 3;
    int speed_preset = default_speed_preset;
    std::string preset_lower = param_x264_preset_;
    std::transform(
        preset_lower.begin(),
        preset_lower.end(),
        preset_lower.begin(),
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
            RCLCPP_WARN(this->get_logger(), "unknown x264_preset='%s', using auto", param_x264_preset_.c_str());
            speed_preset = default_speed_preset;
        }
    }

    if (low_bitrate_mode) {
        g_object_set(
            G_OBJECT(encoder),
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
        g_object_set(
            G_OBJECT(encoder),
            "bitrate", param_target_bitrate_,
            "speed-preset", speed_preset,
            "tune", 0x00000004,
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

    g_object_set(G_OBJECT(parser), "config-interval", -1, "disable-passthrough", TRUE, nullptr);

    GstCaps *h264_caps = gst_caps_new_simple(
        "video/x-h264",
        "stream-format", G_TYPE_STRING, "byte-stream",
        "alignment", G_TYPE_STRING, "au",
        nullptr);
    g_object_set(
        G_OBJECT(appsink_),
        "caps", h264_caps,
        "max-buffers", 5,
        "drop", FALSE,
        "emit-signals", FALSE,
        "sync", FALSE,
        nullptr);
    gst_caps_unref(h264_caps);

    gst_bin_add_many(GST_BIN(pipeline_), appsrc_, convert, encoder, parser, appsink_, nullptr);
    if (!gst_element_link_many(appsrc_, convert, encoder, parser, appsink_, nullptr)) {
        RCLCPP_FATAL(this->get_logger(), "GStreamer pipeline link failed");
        return;
    }

    const GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_FATAL(this->get_logger(), "GStreamer pipeline start failed");
        return;
    }

    bus_ = gst_element_get_bus(pipeline_);
    RCLCPP_INFO(this->get_logger(), "GStreamer encoder ready (%s mode)", low_bitrate_mode ? "low-bitrate" : "low-latency");
}

void VideoEncoderNode::shutdown_gstreamer() {
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        if (bus_) {
            gst_object_unref(bus_);
            bus_ = nullptr;
        }
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
}

cv::Mat VideoEncoderNode::preprocess_image(
    const cv::Mat &input,
    cv::Mat *roi_downsample,
    cv::Mat *static_removed) {
    int x = (input.cols - param_crop_size_) / 2;
    int y = (input.rows - param_crop_size_) / 2;
    x = std::max(0, x);
    y = std::max(0, y);
    const int w = std::min(param_crop_size_, input.cols - x);
    const int h = std::min(param_crop_size_, input.rows - y);

    cv::Mat cropped = input(cv::Rect(x, y, w, h));
    cv::Mat resized;
    cv::resize(cropped, resized, cv::Size(param_output_size_, param_output_size_), 0, 0, cv::INTER_LINEAR);
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
        if (static_removed) {
            working.copyTo(*static_removed);
        }
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
        static_cast<double>(cv::countNonZero(motion_mask)) / static_cast<double>(motion_mask.total());
    const bool suppress_trail = motion_ratio_raw >= param_trail_disable_motion_ratio_;

    if (param_center_clear_size_ > 0) {
        const int clear_size = std::min({param_center_clear_size_, working.cols, working.rows});
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
    cv::GaussianBlur(
        static_base,
        blurred_static,
        cv::Size(),
        std::max(0.0, param_bg_blur_sigma_),
        std::max(0.0, param_bg_blur_sigma_));

    cv::Mat focused = blurred_static.clone();
    working.copyTo(focused, motion_mask);
    if (static_removed) {
        focused.copyTo(*static_removed);
    }

    if (param_motion_trail_frames_ > 0) {
        motion_mask_history_.push_back(motion_mask.clone());
        trail_frame_history_.push_back(working.clone());
        const size_t max_history = static_cast<size_t>(param_motion_trail_frames_ + 1);
        while (motion_mask_history_.size() > max_history) {
            motion_mask_history_.pop_front();
        }
        while (trail_frame_history_.size() > max_history) {
            trail_frame_history_.pop_front();
        }

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

    cv::accumulateWeighted(gray, background_gray_f32_, param_bg_update_alpha_);
    return focused;
}

void VideoEncoderNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        if (param_output_fps_ < 60) {
            const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
            const int64_t frame_interval_ns = 1000000000LL / std::max(param_output_fps_, 1);
            const int64_t now_ns = stamp_ns > 0 ? stamp_ns : this->now().nanoseconds();
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
            cv::resize(
                input,
                raw_preview,
                cv::Size(std::max(1, input.cols / 2), std::max(1, input.rows / 2)),
                0,
                0,
                cv::INTER_AREA);
            std::lock_guard<std::mutex> lock(frame_mutex_);
            raw_preview.copyTo(display_raw_frame_);
            roi_downsample.copyTo(display_roi_frame_);
            static_removed.copyTo(display_static_frame_);
            processed.copyTo(display_frame_);
        }

        push_frame_to_gstreamer(processed);
        pull_stream_and_packetize();
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    }
}

void VideoEncoderNode::push_frame_to_gstreamer(const cv::Mat &frame) {
    if (!appsrc_ || frame.empty()) {
        return;
    }

    const size_t size = frame.total() * frame.elemSize();
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);

    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        std::memcpy(map.data, frame.data, size);
        gst_buffer_unmap(buffer, &map);

        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
        if (ret != GST_FLOW_OK) {
            RCLCPP_WARN(this->get_logger(), "push-buffer failed: %d", ret);
        }
    }
    gst_buffer_unref(buffer);
}

void VideoEncoderNode::pull_stream_and_packetize() {
    if (!appsink_) {
        return;
    }

    const int64_t window_ns = static_cast<int64_t>(param_bandwidth_window_s_ * 1e9);
    const size_t window_limit_bytes = static_cast<size_t>(
        param_bandwidth_limit_kbytes_ * 1000.0 * param_bandwidth_window_s_);
    const double video_packets_per_frame =
        (param_bandwidth_limit_kbytes_ * 1000.0) /
        (static_cast<double>(kVideoPacketPayloadBytes) * static_cast<double>(param_output_fps_));
    const size_t max_video_packets_per_pull = std::max<size_t>(
        1,
        static_cast<size_t>(std::ceil(video_packets_per_frame)));

    while (true) {
        GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), 0);
        if (!sample) {
            break;
        }

        GstBuffer *buffer = gst_sample_get_buffer(sample);
        if (!buffer) {
            gst_sample_unref(sample);
            continue;
        }

        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            const int64_t now_ns = this->now().nanoseconds();
            std::lock_guard<std::mutex> lock(buffer_mutex_);

            // Keep the Pacific-compatible ROS2 stream and the 0x0310 main stream independent.
            if (param_enable_video_stream_) {
                append_bytes(ros2_stream_buffer_, map.data, map.size);
            }
            append_bytes(serial_stream_buffer_, map.data, map.size);

            publish_video_stream_packets(now_ns, window_ns, window_limit_bytes, max_video_packets_per_pull);
            clip_video_stream_backlog();
            emit_serial_packets(now_ns);
            clip_serial_backlog();

            if (now_ns - last_telemetry_ns_ > 1000000000LL) {
                const double telemetry_elapsed_s =
                    last_telemetry_ns_ == 0 ? 0.0 : static_cast<double>(now_ns - last_telemetry_ns_) / 1e9;
                const uint64_t serial_delta = serial_packets_sent_ - last_telemetry_serial_packets_;
                const uint64_t ros2_delta = ros2_packets_sent_ - last_telemetry_ros2_packets_;
                const double serial_rate_hz = telemetry_elapsed_s > 0.0
                    ? static_cast<double>(serial_delta) / telemetry_elapsed_s
                    : 0.0;
                const double topic_rate_hz = telemetry_elapsed_s > 0.0
                    ? static_cast<double>(ros2_delta) / telemetry_elapsed_s
                    : 0.0;
                const double serial_data_kbytes_per_s =
                    serial_rate_hz * static_cast<double>(kSerialSliceBytes) / 1000.0;
                const double topic_data_kbytes_per_s =
                    topic_rate_hz * static_cast<double>(kVideoPacketPayloadBytes) / 1000.0;
                const std::string serial_port = serial_status_provider_
                    ? serial_status_provider_()
                    : std::string("unknown");
                RCLCPP_INFO(
                    rclcpp::get_logger("serial stats"),
                    "rate=%.1fpkt/s data=%.2fkB/s packets=%lu drops=%u backlog=%zuB port=%s",
                    serial_rate_hz,
                    serial_data_kbytes_per_s,
                    serial_packets_sent_,
                    serial_drop_events_,
                    serial_stream_buffer_.size(),
                    serial_port.c_str());
                if (param_enable_video_stream_ && video_packet_pub_) {
                    RCLCPP_INFO(
                        rclcpp::get_logger("topic stats"),
                        "rate=%.1fpkt/s data=%.2fkB/s packets=%lu drops=%u backlog=%zuB",
                        topic_rate_hz,
                        topic_data_kbytes_per_s,
                        ros2_packets_sent_,
                        video_stream_drop_events_,
                        ros2_stream_buffer_.size());
                }
                last_telemetry_ns_ = now_ns;
                last_telemetry_serial_packets_ = serial_packets_sent_;
                last_telemetry_ros2_packets_ = ros2_packets_sent_;
            }

            gst_buffer_unmap(buffer, &map);
        }

        gst_sample_unref(sample);
    }
}

void VideoEncoderNode::publish_video_stream_packets(
    int64_t now_ns,
    int64_t window_ns,
    size_t window_limit_bytes,
    size_t max_packets_this_pull) {
    size_t packets_this_pull = 0;
    while (param_enable_video_stream_ &&
           packets_this_pull < max_packets_this_pull &&
           ros2_stream_buffer_.size() >= kVideoPacketPayloadBytes) {
        while (!sent_window_.empty() && (now_ns - sent_window_.front().first) > window_ns) {
            sent_window_bytes_ -= sent_window_.front().second;
            sent_window_.pop_front();
        }
        if (sent_window_bytes_ + kVideoPacketPayloadBytes > window_limit_bytes) {
            break;
        }

        publish_video_packet(ros2_stream_buffer_.data(), now_ns);
        sent_window_.emplace_back(now_ns, kVideoPacketPayloadBytes);
        sent_window_bytes_ += kVideoPacketPayloadBytes;
        erase_front(ros2_stream_buffer_, kVideoPacketPayloadBytes);
        packets_this_pull++;
    }
}

void VideoEncoderNode::publish_video_packet(const uint8_t *payload_150, int64_t timestamp_ns) {
    if (!video_packet_pub_) {
        return;
    }

    auto serialized = serialize_video_packet(
        video_packet_sequence_id_++,
        static_cast<uint64_t>(timestamp_ns),
        payload_150);
    try {
        video_packet_pub_->publish(serialized);
        ros2_packets_sent_++;
    } catch (const std::exception &e) {
        RCLCPP_WARN(
            this->get_logger(),
            "VideoPacket compatibility publisher disabled after publish failure: %s. 0x0310 serial/local_test path remains enabled.",
            e.what());
        video_packet_pub_.reset();
        param_enable_video_stream_ = false;
        ros2_stream_buffer_.clear();
    }
}

rclcpp::SerializedMessage VideoEncoderNode::serialize_video_packet(
    uint64_t sequence_id,
    uint64_t timestamp_ns,
    const uint8_t *payload_150) const {
    std::array<uint8_t, kSerializedVideoPacketBytes> bytes{};

    bytes[0] = 0x00;
    bytes[1] = 0x01;  // CDR little-endian encapsulation.
    bytes[2] = 0x00;
    bytes[3] = 0x00;

    size_t offset = kCdrHeaderBytes;
    write_u64_le(bytes, offset, sequence_id);
    write_u64_le(bytes, offset, timestamp_ns);
    std::memcpy(bytes.data() + offset, payload_150, kVideoPacketPayloadBytes);
    offset += kVideoPacketPayloadBytes;

    rclcpp::SerializedMessage serialized(kSerializedVideoPacketBytes);
    auto &rmw_msg = serialized.get_rcl_serialized_message();
    std::memcpy(rmw_msg.buffer, bytes.data(), offset);
    rmw_msg.buffer_length = offset;
    return serialized;
}

void VideoEncoderNode::emit_serial_packets(int64_t now_ns) {
    if (!serial_data_cb_) {
        return;
    }
    if (next_serial_tx_ns_ == 0) {
        next_serial_tx_ns_ = now_ns;
    }

    // 没有有足够的数据
    if (serial_stream_buffer_.size() < kSerialSliceBytes) {
        return;
    }

    // 没有足够的延迟
    while (now_ns < next_serial_tx_ns_) {
        return;
    }
    
    uint8_t data_300[kSerialDataBytes];
    data_300[0] = serial_inner_seq_++;
    std::memcpy(data_300 + 1, serial_stream_buffer_.data(), kSerialSliceBytes);

    serial_data_cb_(data_300);
    serial_packets_sent_++;

    erase_front(serial_stream_buffer_, kSerialSliceBytes);
    const int64_t period_ns = static_cast<int64_t>(1000000000.0 / param_serial_max_rate_hz_);
    next_serial_tx_ns_ = this->now().nanoseconds() + period_ns;
}

void VideoEncoderNode::clip_video_stream_backlog() {
    if (!param_enable_video_stream_) {
        return;
    }

    const size_t max_backlog = static_cast<size_t>(
        param_bandwidth_limit_kbytes_ * 1000.0 * param_max_tx_delay_s_);
    if (ros2_stream_buffer_.size() <= max_backlog) {
        return;
    }

    const size_t target_drop = ros2_stream_buffer_.size() - max_backlog;
    const size_t drop_bytes = align_drop_to_annexb_start(ros2_stream_buffer_, target_drop);
    erase_front(ros2_stream_buffer_, drop_bytes);
    video_stream_dropped_bytes_ += drop_bytes;
    video_stream_drop_events_++;
    if (video_stream_drop_events_ % 20 == 1) {
        RCLCPP_WARN(
            this->get_logger(),
            "video stream backlog clipped: dropped=%zuB backlog=%zuB total_dropped=%luB",
            drop_bytes,
            ros2_stream_buffer_.size(),
            video_stream_dropped_bytes_);
    }
}

void VideoEncoderNode::clip_serial_backlog() {
    const size_t max_backlog = static_cast<size_t>(
        std::max(1.0, param_serial_max_rate_hz_) *
        static_cast<double>(kSerialSliceBytes) *
        param_max_tx_delay_s_);
    if (serial_stream_buffer_.size() <= max_backlog) {
        return;
    }

    const size_t target_drop = serial_stream_buffer_.size() - max_backlog;
    const size_t drop_bytes = align_drop_to_annexb_start(serial_stream_buffer_, target_drop);
    erase_front(serial_stream_buffer_, drop_bytes);
    serial_dropped_bytes_ += drop_bytes;
    serial_drop_events_++;
    if (serial_drop_events_ % 20 == 1) {
        RCLCPP_WARN(
            this->get_logger(),
            "serial backlog clipped: dropped=%zuB backlog=%zuB total_dropped=%luB",
            drop_bytes,
            serial_stream_buffer_.size(),
            serial_dropped_bytes_);
    }
}

void VideoEncoderNode::display_loop() {
    constexpr const char *kWindowName = "Sniper Sender";
    cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(kWindowName, param_output_size_ * 2, param_output_size_ * 2);

    const auto render_tile = [this](
                                 const cv::Mat &source,
                                 const char *label,
                                 cv::Mat &tile) {
        tile = cv::Mat::zeros(param_output_size_, param_output_size_, CV_8UC3);
        if (!source.empty()) {
            cv::Mat color_source;
            if (source.channels() == 1) {
                cv::cvtColor(source, color_source, cv::COLOR_GRAY2BGR);
            } else if (source.channels() == 4) {
                cv::cvtColor(source, color_source, cv::COLOR_BGRA2BGR);
            } else {
                color_source = source;
            }

            const double scale = std::min(
                static_cast<double>(param_output_size_) / color_source.cols,
                static_cast<double>(param_output_size_) / color_source.rows);
            const cv::Size fitted_size(
                std::max(1, static_cast<int>(color_source.cols * scale)),
                std::max(1, static_cast<int>(color_source.rows * scale)));
            cv::Mat fitted;
            cv::resize(color_source, fitted, fitted_size, 0.0, 0.0, cv::INTER_AREA);
            const int x = (param_output_size_ - fitted.cols) / 2;
            const int y = (param_output_size_ - fitted.rows) / 2;
            fitted.copyTo(tile(cv::Rect(x, y, fitted.cols, fitted.rows)));
        }

        cv::putText(
            tile,
            label,
            cv::Point(10, 26),
            cv::FONT_HERSHEY_SIMPLEX,
            0.65,
            cv::Scalar(255, 255, 255),
            2,
            cv::LINE_AA);
    };

    while (display_running_ && rclcpp::ok()) {
        cv::Mat raw_frame;
        cv::Mat roi_frame;
        cv::Mat static_frame;
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (!display_raw_frame_.empty()) {
                display_raw_frame_.copyTo(raw_frame);
            }
            if (!display_roi_frame_.empty()) {
                display_roi_frame_.copyTo(roi_frame);
            }
            if (!display_static_frame_.empty()) {
                display_static_frame_.copyTo(static_frame);
            }
            if (!display_frame_.empty()) {
                display_frame_.copyTo(frame);
            }
        }

        cv::Mat raw_tile;
        cv::Mat roi_tile;
        cv::Mat static_tile;
        cv::Mat encoder_tile;
        render_tile(raw_frame, "Raw", raw_tile);
        render_tile(roi_frame, "ROI", roi_tile);
        render_tile(static_frame, "Static", static_tile);
        render_tile(frame, "Encoder", encoder_tile);

        cv::Mat top_row;
        cv::Mat bottom_row;
        cv::Mat dashboard;
        cv::hconcat(raw_tile, roi_tile, top_row);
        cv::hconcat(static_tile, encoder_tile, bottom_row);
        cv::vconcat(top_row, bottom_row, dashboard);
        cv::imshow(kWindowName, dashboard);

        cv::waitKey(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    cv::destroyWindow(kWindowName);
}

} // namespace sniper::encoder

RCLCPP_COMPONENTS_REGISTER_NODE(sniper::encoder::VideoEncoderNode)
