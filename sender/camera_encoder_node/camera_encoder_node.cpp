#include "hik_camera_node/hik_camera_node.hpp"
#include "video_encoder_node/video_encoder_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdio>
#include <exception>
#include <memory>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<sniper::camera::HikCameraNode> camera_node;
    std::shared_ptr<sniper::encoder::VideoEncoderNode> encoder_node;

    try {
        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true);
        camera_node = std::make_shared<sniper::camera::HikCameraNode>(options);
        if (!rclcpp::ok()) {
            camera_node.reset();
            return 1;
        }
        encoder_node = std::make_shared<sniper::encoder::VideoEncoderNode>(
            camera_node->frame_rate(), options);

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(camera_node);
        executor.add_node(encoder_node);
        executor.spin();
        executor.remove_node(encoder_node);
        executor.remove_node(camera_node);
    } catch (const std::exception &error) {
        std::fprintf(stderr, "[camera_encoder] fatal error: %s\n", error.what());
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        encoder_node.reset();
        camera_node.reset();
        return 1;
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    encoder_node.reset();
    camera_node.reset();
    return 0;
}
