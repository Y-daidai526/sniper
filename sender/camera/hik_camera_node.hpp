#ifndef SNIPER_CAMERA_HIK_CAMERA_NODE_HPP_
#define SNIPER_CAMERA_HIK_CAMERA_NODE_HPP_

#include <MvCameraControl.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>
#include <thread>

namespace sniper::camera {

class HikCameraNode : public rclcpp::Node {
public:
    explicit HikCameraNode(const rclcpp::NodeOptions &options);
    ~HikCameraNode() override;

private:
    void declareParameters();
    void cameraFailSuggestion(int error);
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);

    sensor_msgs::msg::Image image_msg_;
    image_transport::CameraPublisher camera_pub_;

    int nRet_ = MV_OK;
    void *camera_handle_ = nullptr;
    MV_IMAGE_BASIC_INFO img_info_{};

    std::string camera_name_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    int fail_count_ = 0;
    std::thread capture_thread_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

} // namespace sniper::camera

#endif
