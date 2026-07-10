#include "hik_camera_node.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <cstddef>
#include <functional>
#include <thread>

namespace sniper::camera {

HikCameraNode::HikCameraNode(const rclcpp::NodeOptions &options)
    : Node("hik_camera", options) {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode");

    MV_CC_DEVICE_INFO_LIST device_list;
    nRet_ = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

    while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "No camera found");
        RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet_);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        nRet_ = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }
    if (device_list.nDeviceNum == 0) {
        RCLCPP_ERROR(this->get_logger(), "Camera enumeration stopped before a device was found");
        rclcpp::shutdown();
        return;
    }

    int status = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    if (status != MV_OK) {
        cameraFailSuggestion(status);
        rclcpp::shutdown();
        return;
    }

    status = MV_CC_OpenDevice(camera_handle_);
    if (status != MV_OK) {
        cameraFailSuggestion(status);
        rclcpp::shutdown();
        return;
    }

    MV_CC_GetImageInfo(camera_handle_, &img_info_);
    image_msg_.data.reserve(static_cast<size_t>(img_info_.nHeightMax) * img_info_.nWidthMax * 3);

    MVCC_ENUMVALUE pixel_format{};
    MV_CC_GetEnumValue(camera_handle_, "PixelFormat", &pixel_format);
    RCLCPP_INFO(this->get_logger(), "Camera support %d pixel format(s)", pixel_format.nSupportedNum);

    MVCC_INTVALUE_EX int_value{};
    MV_CC_GetIntValueEx(camera_handle_, "Width", &int_value);
    const int img_width_max = int_value.nCurValue;
    MV_CC_GetIntValueEx(camera_handle_, "Height", &int_value);
    const int img_height_max = int_value.nCurValue;
    RCLCPP_INFO(this->get_logger(), "Image size: (%d x %d)", img_width_max, img_height_max);

    image_msg_.data.reserve(static_cast<size_t>(img_width_max) * img_height_max * 3);

    const bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    image_topic_ = this->declare_parameter("image_topic", std::string("image_raw"));
    frame_id_ = this->declare_parameter("frame_id", std::string("camera_optical_frame"));
    const auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, image_topic_, qos);

    declareParameters();

    status = MV_CC_StartGrabbing(camera_handle_);
    if (status != MV_OK) {
        cameraFailSuggestion(status);
        rclcpp::shutdown();
        return;
    }

    camera_name_ = this->declare_parameter("camera_name", std::string("narrow_stereo"));
    camera_info_manager_ =
        std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    const auto camera_info_url = this->declare_parameter("camera_info_url", std::string(""));
    if (!camera_info_url.empty() && camera_info_manager_->validateURL(camera_info_url)) {
        try {
            camera_info_manager_->loadCameraInfo(camera_info_url);
            camera_info_msg_ = camera_info_manager_->getCameraInfo();
        } catch (const std::exception &e) {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to load camera info from '%s': %s",
                camera_info_url.c_str(),
                e.what());
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "No camera_info_url set, continuing without calibration data");
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
        MV_FRAME_OUT out_frame{};
        int frame_count = 0;

        image_msg_.header.frame_id = frame_id_;
        image_msg_.encoding = "bgr8";
        RCLCPP_INFO(this->get_logger(), "Publishing camera images on %s", image_topic_.c_str());

        while (rclcpp::ok()) {
            nRet_ = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
            if (nRet_ == MV_OK) {
                image_msg_.header.stamp = this->now();
                image_msg_.width = out_frame.stFrameInfo.nWidth;
                image_msg_.height = out_frame.stFrameInfo.nHeight;
                image_msg_.step = image_msg_.width * 3;
                image_msg_.data.resize(static_cast<size_t>(image_msg_.step) * image_msg_.height);

                cv::Mat bayer_mat(
                    out_frame.stFrameInfo.nHeight,
                    out_frame.stFrameInfo.nWidth,
                    CV_8UC1,
                    out_frame.pBufAddr);
                cv::Mat bgr_mat(
                    out_frame.stFrameInfo.nHeight,
                    out_frame.stFrameInfo.nWidth,
                    CV_8UC3,
                    image_msg_.data.data());
                cv::cvtColor(bayer_mat, bgr_mat, cv::COLOR_BayerRG2BGR);

                if (bgr_mat.empty()) {
                    RCLCPP_ERROR(this->get_logger(), "OpenCV Bayer conversion failed");
                    MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
                    continue;
                }

                ++frame_count;
                if (frame_count % 2000 == 0) {
                    const int center_x = static_cast<int>(image_msg_.width) / 2;
                    const int center_y = static_cast<int>(image_msg_.height) / 2;
                    const int pixel_index = center_y * static_cast<int>(image_msg_.step) + center_x * 3;
                    if (pixel_index + 2 < static_cast<int>(image_msg_.data.size())) {
                        const uint8_t b = image_msg_.data[pixel_index];
                        const uint8_t g = image_msg_.data[pixel_index + 1];
                        const uint8_t r = image_msg_.data[pixel_index + 2];
                        RCLCPP_INFO(
                            this->get_logger(),
                            "Frame %d center pixel BGR=(%u,%u,%u)",
                            frame_count,
                            b,
                            g,
                            r);
                    }
                }

                camera_info_msg_.header = image_msg_.header;
                camera_info_msg_.width = image_msg_.width;
                camera_info_msg_.height = image_msg_.height;
                camera_pub_.publish(image_msg_, camera_info_msg_);

                MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
                fail_count_ = 0;
            } else {
                RCLCPP_WARN(this->get_logger(), "Get image buffer failed: [%x]", nRet_);
                MV_CC_StopGrabbing(camera_handle_);
                MV_CC_StartGrabbing(camera_handle_);
                ++fail_count_;
            }

            if (fail_count_ > 5) {
                RCLCPP_FATAL(this->get_logger(), "Camera failed repeatedly");
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
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed");
}

void HikCameraNode::cameraFailSuggestion(int error) {
    RCLCPP_WARN(
        this->get_logger(),
        "Camera SDK error [%x]. Check that the camera is connected and not opened by MVS or another process.",
        error);
}

void HikCameraNode::declareParameters() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE float_value{};

    int status = MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &float_value);
    if (status != MV_OK) {
        cameraFailSuggestion(status);
    }

    param_desc.description = "Exposure time in microseconds";
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].from_value = float_value.fMin;
    param_desc.floating_point_range[0].to_value = float_value.fMax;
    param_desc.floating_point_range[0].step = 0.0;

    RCLCPP_INFO(
        this->get_logger(),
        "Exposure min/max/current: %.3f/%.1f/%.3f",
        float_value.fMin,
        float_value.fMax,
        float_value.fCurValue);

    const double exposure_time = this->declare_parameter("exposure_time", 1000.0, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", static_cast<float>(exposure_time));

    const double acquisition_frame_rate = this->declare_parameter("acquisition_frame_rate", 250.0);
    MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", static_cast<float>(acquisition_frame_rate));

    const bool color_transformation_enable =
        this->declare_parameter("color_transformation_enable", true);
    MV_CC_SetBoolValue(camera_handle_, "ColorTransformationEnable", color_transformation_enable);

    const double gamma = this->declare_parameter("gamma", 7.5);
    MV_CC_SetFloatValue(camera_handle_, "Gamma", static_cast<float>(gamma));

    const bool ccm_enable = this->declare_parameter("ccm_enable", false);
    MV_CC_SetBoolValue(camera_handle_, "CCMEnable", ccm_enable);

    MV_CC_GetFloatValue(camera_handle_, "Gain", &float_value);
    param_desc.description = "Gain";
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].from_value = float_value.fMin;
    param_desc.floating_point_range[0].to_value = float_value.fMax;
    param_desc.floating_point_range[0].step = 0.1;

    const double gain = this->declare_parameter("gain", static_cast<double>(float_value.fCurValue), param_desc);
    MV_CC_SetFloatValue(camera_handle_, "Gain", static_cast<float>(gain));
}

rcl_interfaces::msg::SetParametersResult HikCameraNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters) {
        int status = MV_OK;
        if (param.get_name() == "exposure_time") {
            status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", static_cast<float>(param.as_double()));
        } else if (param.get_name() == "gain") {
            status = MV_CC_SetFloatValue(camera_handle_, "Gain", static_cast<float>(param.as_double()));
        } else if (param.get_name() == "acquisition_frame_rate") {
            status = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", static_cast<float>(param.as_double()));
        } else if (param.get_name() == "color_transformation_enable") {
            status = MV_CC_SetBoolValue(camera_handle_, "ColorTransformationEnable", param.as_bool());
        } else if (param.get_name() == "gamma") {
            status = MV_CC_SetFloatValue(camera_handle_, "Gamma", static_cast<float>(param.as_double()));
        } else if (param.get_name() == "ccm_enable") {
            status = MV_CC_SetBoolValue(camera_handle_, "CCMEnable", param.as_bool());
        } else {
            continue;
        }

        if (status != MV_OK) {
            result.successful = false;
            result.reason = "Failed to set camera parameter: " + param.get_name();
            break;
        }
    }

    return result;
}

} // namespace sniper::camera

RCLCPP_COMPONENTS_REGISTER_NODE(sniper::camera::HikCameraNode)
