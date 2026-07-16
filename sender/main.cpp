#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <csignal>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>

#include "camera/hik_camera_node.hpp"
#include "encoder/video_encoder_node.hpp"
#include "serial/serial_send_worker.hpp"

namespace {
bool require_bool_parameter(const rclcpp::Node::SharedPtr &node, const std::string &name) {
    node->declare_parameter(name, rclcpp::ParameterType::PARAMETER_BOOL);
    return node->get_parameter(name).as_bool();
}

int require_int_parameter(const rclcpp::Node::SharedPtr &node, const std::string &name) {
    node->declare_parameter(name, rclcpp::ParameterType::PARAMETER_INTEGER);
    return static_cast<int>(node->get_parameter(name).as_int());
}

std::string require_string_parameter(const rclcpp::Node::SharedPtr &node, const std::string &name) {
    node->declare_parameter(name, rclcpp::ParameterType::PARAMETER_STRING);
    return node->get_parameter(name).as_string();
}

std::string local_test_dir() {
    try {
        return (std::filesystem::path(ament_index_cpp::get_package_share_directory("sender")) / "local_test").string();
    } catch (const std::exception &) {
        return (std::filesystem::current_path() / "sender" / "local_test").string();
    }
}
} // namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
#ifdef SIGPIPE
    std::signal(SIGPIPE, SIG_IGN);
#endif

    try {
        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true);

        auto runtime_node = std::make_shared<rclcpp::Node>("sender_runtime", options);
        const bool enable_serial = require_bool_parameter(runtime_node, "enable_serial");
        const bool enable_local_test = require_bool_parameter(runtime_node, "enable_local_test");
        const bool local_test_start_broker = require_bool_parameter(runtime_node, "local_test_start_broker");
        const std::string local_test_mqtt_host = require_string_parameter(runtime_node, "local_test_mqtt_host");
        const int local_test_mqtt_port = require_int_parameter(runtime_node, "local_test_mqtt_port");
        const std::string local_test_mqtt_topic = require_string_parameter(runtime_node, "local_test_mqtt_topic");
        const std::string local_test_script_dir = local_test_dir();

        auto camera_node = std::make_shared<sniper::camera::HikCameraNode>(options);
        auto encoder_node = std::make_shared<sniper::encoder::VideoEncoderNode>(options);

        if (!enable_serial) {
            RCLCPP_WARN(runtime_node->get_logger(), "serial output disabled by config");
        }
        sniper::serial::SerialSendWorker serial_worker({
            enable_serial,
            enable_local_test,
            local_test_start_broker,
            local_test_script_dir,
            local_test_mqtt_host,
            local_test_mqtt_port,
            local_test_mqtt_topic,
            encoder_node->serial_max_rate_hz(),
            encoder_node->max_tx_delay_s(),
        });
        serial_worker.start();
        encoder_node->set_serial_stream_callback(
            [&serial_worker](const uint8_t *data, size_t size) { serial_worker.enqueue(data, size); });

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(runtime_node);
        executor.add_node(camera_node);
        executor.add_node(encoder_node);

        RCLCPP_INFO(runtime_node->get_logger(), "sender started");
        executor.spin();

        serial_worker.stop();
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "[sender] fatal error: %s\n", e.what());
        rclcpp::shutdown();
        return 1;
    }
}
