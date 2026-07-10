#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <csignal>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>

#include "camera/hik_camera_node.hpp"
#include "debug/debug_bridge.hpp"
#include "encoder/video_encoder_node.hpp"
#include "protocol/serial_frame.hpp"
#include "serial/serial_writer.hpp"

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

std::string default_debug_dir() {
    try {
        return (std::filesystem::path(ament_index_cpp::get_package_share_directory("sender")) / "debug").string();
    } catch (const std::exception &) {
        return (std::filesystem::current_path() / "sender" / "debug").string();
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
    const bool enable_debug_bridge = require_bool_parameter(runtime_node, "enable_debug_bridge");
    const bool debug_start_broker = require_bool_parameter(runtime_node, "debug_start_broker");
    const std::string debug_mqtt_host = require_string_parameter(runtime_node, "debug_mqtt_host");
    const int debug_mqtt_port = require_int_parameter(runtime_node, "debug_mqtt_port");
    const std::string debug_mqtt_topic = require_string_parameter(runtime_node, "debug_mqtt_topic");
    std::string debug_script_dir = require_string_parameter(runtime_node, "debug_script_dir");
    if (debug_script_dir.empty()) {
        debug_script_dir = default_debug_dir();
    }

    auto camera_node = std::make_shared<sniper::camera::HikCameraNode>(options);
    auto encoder_node = std::make_shared<sniper::encoder::VideoEncoderNode>(options);

    sniper::serial::SerialWriter serial_writer;
    sniper::debug::DebugBridge debug_bridge;

    if (enable_serial) {
        serial_writer.start([](const std::string &device, bool connected) {
            if (connected) {
                std::fprintf(stdout, "[sender] serial connected: %s\n", device.c_str());
            } else {
                std::fprintf(stdout, "[sender] serial disconnected: %s\n", device.c_str());
            }
        });
    } else {
        RCLCPP_WARN(runtime_node->get_logger(), "serial output disabled by config");
    }

    if (enable_debug_bridge) {
        if (std::filesystem::exists(std::filesystem::path(debug_script_dir) / "main.py")) {
            if (!debug_bridge.start(
                    debug_script_dir,
                    debug_mqtt_host,
                    debug_mqtt_port,
                    debug_mqtt_topic,
                    debug_start_broker)) {
                RCLCPP_ERROR(runtime_node->get_logger(), "failed to start debug bridge");
            }
        } else {
            RCLCPP_WARN(
                runtime_node->get_logger(),
                "debug bridge disabled: %s/main.py not found",
                debug_script_dir.c_str());
        }
    }

    encoder_node->set_serial_data_callback(
        [&serial_writer, &debug_bridge, enable_serial, enable_debug_bridge](const uint8_t *data_300) {
            static uint8_t frame_seq = 0;
            const auto frame = sniper::protocol::build_frame(frame_seq++, data_300);
            if (enable_serial) {
                serial_writer.write_frame(frame.data(), frame.size());
            }
            if (enable_debug_bridge) {
                debug_bridge.write_frame(frame.data(), frame.size());
            }
        });

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(runtime_node);
    executor.add_node(camera_node);
    executor.add_node(encoder_node);

    RCLCPP_INFO(runtime_node->get_logger(), "sender started");
    executor.spin();

    debug_bridge.stop();
    serial_writer.stop();
    rclcpp::shutdown();
    return 0;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "[sender] fatal error: %s\n", e.what());
        rclcpp::shutdown();
        return 1;
    }
}
