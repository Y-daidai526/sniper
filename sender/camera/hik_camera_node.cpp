#include "hik_camera_node.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <thread>

namespace sniper::camera {

HikCameraNode::HikCameraNode(const rclcpp::NodeOptions &options)
    : Node("hik_camera", options) {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    MV_CC_DEVICE_INFO_LIST device_list;
    nRet_ = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

    while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "No camera found!");
        RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet_);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        nRet_ = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }

    MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);

    int status = MV_CC_OpenDevice(camera_handle_);
    if (status != MV_OK) { cameraFailSuggestion(status); }

    // Get camera information
    MV_CC_GetImageInfo(camera_handle_, &img_info_);
    image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

    MVCC_ENUMVALUE stEnumValue{};
    MV_CC_GetEnumValue(camera_handle_, "PixelFormat", &stEnumValue);
    RCLCPP_INFO(this->get_logger(), "Camera support %d pixel format(s)", stEnumValue.nSupportedNum);

    MVCC_INTVALUE_EX stIntValue{};
    MV_CC_GetIntValueEx(camera_handle_, "Width", &stIntValue);
    int img_width_max = stIntValue.nCurValue;
    MV_CC_GetIntValueEx(camera_handle_, "Height", &stIntValue);
    int img_height_max = stIntValue.nCurValue;
    RCLCPP_INFO(this->get_logger(), "Image size: ( %d x %d )", img_width_max, img_height_max);

    image_msg_.data.resize(img_width_max * img_height_max * 3);

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    declareParameters();

    MV_CC_StartGrabbing(camera_handle_);

    // Load camera info (non-fatal)
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
        std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url = this->declare_parameter(
        "camera_info_url", "");
    if (!camera_info_url.empty() && camera_info_manager_->validateURL(camera_info_url)) {
        try {
            camera_info_manager_->loadCameraInfo(camera_info_url);
            camera_info_msg_ = camera_info_manager_->getCameraInfo();
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(),
                "Failed to load camera info from '%s': %s (continuing without)",
                camera_info_url.c_str(), e.what());
        }
    } else {
        RCLCPP_INFO(this->get_logger(),
            "No camera_info_url set, continuing without calibration data");
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
        MV_FRAME_OUT out_frame;

        RCLCPP_INFO(this->get_logger(), "Publishing image!");

        image_msg_.header.frame_id = "camera_optical_frame";
        image_msg_.encoding = "bgr8";

        static int frame_count = 0;

        while (rclcpp::ok()) {
            nRet_ = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
            if (MV_OK == nRet_) {
                image_msg_.header.stamp = this->now();
                image_msg_.width = out_frame.stFrameInfo.nWidth;
                image_msg_.height = out_frame.stFrameInfo.nHeight;
                image_msg_.step = image_msg_.width * 3;

                // OpenCV bayer converter is much faster than HIK SDK
                cv::Mat bayer_mat(
                    out_frame.stFrameInfo.nHeight, out_frame.stFrameInfo.nWidth,
                    CV_8UC1, out_frame.pBufAddr);

                cv::Mat rgb_mat(
                    out_frame.stFrameInfo.nHeight, out_frame.stFrameInfo.nWidth,
                    CV_8UC3, image_msg_.data.data());

                cv::cvtColor(bayer_mat, rgb_mat, cv::COLOR_BayerRG2RGB);

                if (rgb_mat.empty()) {
                    RCLCPP_ERROR(this->get_logger(), "OpenCV cvtColor failed!");
                    MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
                    continue;
                }

                frame_count++;
                if (frame_count % 2000 == 0) {
                    int center_x = image_msg_.width / 2;
                    int center_y = image_msg_.height / 2;
                    int pixel_index = center_y * image_msg_.step + center_x * 3;

                    if (pixel_index + 2 < static_cast<int>(image_msg_.data.size())) {
                        uint8_t r = image_msg_.data[pixel_index];
                        uint8_t g = image_msg_.data[pixel_index + 1];
                        uint8_t b = image_msg_.data[pixel_index + 2];

                        RCLCPP_INFO(this->get_logger(),
                            "\x1b[1;32mFrame %d - Pixel at (%d, %d): R=%d, G=%d, B=%d\x1b[0m",
                            frame_count, center_x, center_y, r, g, b);
                    } else {
                        RCLCPP_WARN(this->get_logger(),
                            "Frame %d - Pixel index out of bounds: %d (buffer size: %zu)",
                            frame_count, pixel_index, image_msg_.data.size());
                    }
                }

                camera_info_msg_.header = image_msg_.header;
                camera_pub_.publish(image_msg_, camera_info_msg_);

                MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
                fail_count_ = 0;
            } else {
                RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet_);
                MV_CC_StopGrabbing(camera_handle_);
                MV_CC_StartGrabbing(camera_handle_);
                fail_count_++;
            }

            if (fail_count_ > 5) {
                RCLCPP_FATAL(this->get_logger(), "Camera failed!");
                rclcpp::shutdown();
            }
        }
    }};
}

HikCameraNode::~HikCameraNode() {
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    if (camera_handle_) {
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
}

void HikCameraNode::cameraFailSuggestion(int error) {
    RCLCPP_WARN(
        this->get_logger(),
        "\x1b[1;31m状态码异常[%x]，相机可能没有正确启动。请务必检查相机是否被其他进程或者软件占用（例如HIK MVS）。"
        "关闭这些软件后再重试一次。\x1b[0m",
        error);
}

void HikCameraNode::declareParameters() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;

    // ========== Exposure Time ==========
    int status = MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    if (status != MV_OK) { cameraFailSuggestion(status); }

    param_desc.description = "Exposure time in microseconds";
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].from_value = f_value.fMin;
    param_desc.floating_point_range[0].to_value = f_value.fMax;
    param_desc.floating_point_range[0].step = 0.0;

    RCLCPP_INFO(this->get_logger(),
        "Exposure min/max (current): %.3f/%.1f (%.3f)", f_value.fMin, f_value.fMax, f_value.fCurValue);

    double exposure_time = this->declare_parameter("exposure_time", 1000.0, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", static_cast<float>(exposure_time));
    MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", 250.000f);

    MV_CC_SetBoolValue(camera_handle_, "ColorTransformationEnable", true);
    MV_CC_SetFloatValue(camera_handle_, "Gamma", 7.5f);
    MV_CC_SetBoolValue(camera_handle_, "CCMEnable", false);

    // ========== Gain ==========
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);

    param_desc.description = "Gain";
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].from_value = f_value.fMin;
    param_desc.floating_point_range[0].to_value = f_value.fMax;
    param_desc.floating_point_range[0].step = 0.1;

    double gain = this->declare_parameter("gain", static_cast<double>(f_value.fCurValue), param_desc);
    MV_CC_SetFloatValue(camera_handle_, "Gain", static_cast<float>(gain));
}

rcl_interfaces::msg::SetParametersResult HikCameraNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    std::string error_reason;

    for (const auto &param : parameters) {
        bool param_success = true;

        if (param.get_name() == "exposure_time") {
            int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_double());
            if (MV_OK != status) {
                param_success = false;
                error_reason = "Failed to set exposure time";
            }
        } else if (param.get_name() == "gain") {
            int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
            if (MV_OK != status) {
                param_success = false;
                error_reason = "Failed to set gain";
            }
        } else {
            param_success = false;
            error_reason = "Unknown parameter: " + param.get_name();
        }

        if (!param_success) {
            result.successful = false;
            result.reason = error_reason;
        }
    }

    return result;
}

} // namespace sniper::camera

RCLCPP_COMPONENTS_REGISTER_NODE(sniper::camera::HikCameraNode)
