#include "video_encoder_node/video_encoder_node.hpp"

#include <cv_bridge/cv_bridge.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace sniper::encoder {

namespace {
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

} // namespace

VideoEncoderNode::VideoEncoderNode(double frame_rate, const rclcpp::NodeOptions &options)
    : Node("video_encoder", options), frame_rate_(frame_rate) {
    param_crop_size_ = require_int_parameter(*this, "crop_size");
    param_output_size_ = require_int_parameter(*this, "output_size");
    param_force_monochrome_ = require_bool_parameter(*this, "force_monochrome");
    param_static_simplify_ = require_bool_parameter(*this, "static_simplify");
    param_motion_threshold_ = require_int_parameter(*this, "motion_threshold");
    param_motion_erode_px_ = require_int_parameter(*this, "motion_erode_px");
    param_motion_dilate_px_ = require_int_parameter(*this, "motion_dilate_px");
    param_motion_trail_frames_ = require_int_parameter(*this, "motion_trail_frames");
    param_trail_disable_motion_ratio_ = require_double_parameter(*this, "trail_disable_motion_ratio");
    param_bg_update_alpha_ = require_double_parameter(*this, "bg_update_alpha");
    param_bg_blur_sigma_ = require_double_parameter(*this, "bg_blur_sigma");
    param_center_clear_size_ = require_int_parameter(*this, "center_clear_size");
    param_target_bitrate_ = require_int_parameter(*this, "target_bitrate");
    param_x264_preset_ = require_string_parameter(*this, "x264_preset");
    show_ = this->declare_parameter<bool>("show", false);

    if (!std::isfinite(frame_rate_) || frame_rate_ <= 0.0) {
        throw std::invalid_argument("frame_rate must be greater than zero");
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
    auto image_qos = rclcpp::SensorDataQoS();
    image_qos.keep_last(1);
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw",
        image_qos,
        std::bind(&VideoEncoderNode::image_callback, this, std::placeholders::_1));
    encoded_stream_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "encoded_stream",
        rclcpp::QoS(20).reliable());
    auto preview_qos = rclcpp::SensorDataQoS();
    preview_qos.keep_last(1);
    preview_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "video_preview",
        preview_qos);

    initialize_gstreamer();

    if (show_) {
        cv::namedWindow("Sniper Sender", cv::WINDOW_NORMAL);
        cv::resizeWindow("Sniper Sender", param_output_size_ * 2, param_output_size_ * 2);
        display_timer_ = create_wall_timer(
            std::chrono::milliseconds(16),
            std::bind(&VideoEncoderNode::display_callback, this));
    }

    RCLCPP_INFO(
        this->get_logger(),
        "VideoEncoderNode ready: crop=%d output=%dx%d@%.2ffps bitrate=%dkbps",
        param_crop_size_,
        param_output_size_,
        param_output_size_,
        frame_rate_,
        param_target_bitrate_);
}

VideoEncoderNode::~VideoEncoderNode() {
    if (show_) {
        display_timer_.reset();
        cv::destroyWindow("Sniper Sender");
    }
    shutdown_gstreamer();
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
        shutdown_gstreamer();
        throw std::runtime_error("GStreamer element creation failed");
    }

    gint frame_rate_num = 0;
    gint frame_rate_den = 1;
    gst_util_double_to_fraction(frame_rate_, &frame_rate_num, &frame_rate_den);
    GstCaps *caps = gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "BGR",
        "width", G_TYPE_INT, param_output_size_,
        "height", G_TYPE_INT, param_output_size_,
        "framerate", GST_TYPE_FRACTION, frame_rate_num, frame_rate_den,
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
    const int rounded_frame_rate = std::max(1, static_cast<int>(std::lround(frame_rate_)));
    const int key_int = std::max(8 * rounded_frame_rate, 30);
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
            "key-int-max", rounded_frame_rate,
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
        shutdown_gstreamer();
        throw std::runtime_error("GStreamer pipeline link failed");
    }

    const GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        shutdown_gstreamer();
        throw std::runtime_error("GStreamer pipeline start failed");
    }

    RCLCPP_INFO(this->get_logger(), "GStreamer encoder ready (%s mode)", low_bitrate_mode ? "low-bitrate" : "low-latency");
}

void VideoEncoderNode::shutdown_gstreamer() {
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        appsrc_ = nullptr;
        appsink_ = nullptr;
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
        cv::Mat input = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat roi_downsample;
        cv::Mat static_removed;
        cv::Mat processed = preprocess_image(
            input,
            &roi_downsample,
            &static_removed);

        cv::Mat raw_preview;
        cv::resize(
            input,
            raw_preview,
            cv::Size(std::max(1, input.cols / 2), std::max(1, input.rows / 2)),
            0,
            0,
            cv::INTER_AREA);
        cv::Mat preview = build_preview(
            raw_preview,
            roi_downsample,
            static_removed,
            processed);
        auto preview_msg = cv_bridge::CvImage(msg->header, "bgr8", preview).toImageMsg();
        preview_pub_->publish(*preview_msg);
        if (show_) {
            preview.copyTo(preview_frame_);
        }

        push_frame_to_gstreamer(processed);
        pull_encoded_stream();
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

void VideoEncoderNode::pull_encoded_stream() {
    if (!appsink_) {
        return;
    }

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
            std_msgs::msg::UInt8MultiArray message;
            message.data.assign(map.data, map.data + map.size);
            encoded_stream_pub_->publish(std::move(message));
            gst_buffer_unmap(buffer, &map);
        }

        gst_sample_unref(sample);
    }
}

cv::Mat VideoEncoderNode::build_preview(
    const cv::Mat &raw_frame,
    const cv::Mat &roi_frame,
    const cv::Mat &static_frame,
    const cv::Mat &encoder_frame) const {
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

    cv::Mat raw_tile;
    cv::Mat roi_tile;
    cv::Mat static_tile;
    cv::Mat encoder_tile;
    render_tile(raw_frame, "Raw", raw_tile);
    render_tile(roi_frame, "ROI", roi_tile);
    render_tile(static_frame, "Static", static_tile);
    render_tile(encoder_frame, "Encoder", encoder_tile);

    cv::Mat top_row;
    cv::Mat bottom_row;
    cv::Mat dashboard;
    cv::hconcat(raw_tile, roi_tile, top_row);
    cv::hconcat(static_tile, encoder_tile, bottom_row);
    cv::vconcat(top_row, bottom_row, dashboard);
    return dashboard;
}

void VideoEncoderNode::display_callback() {
    constexpr const char *kWindowName = "Sniper Sender";

    if (!preview_frame_.empty()) {
        cv::imshow(kWindowName, preview_frame_);
    }
    cv::waitKey(1);
}

} // namespace sniper::encoder
