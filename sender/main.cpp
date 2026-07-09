/**
 * sender/main.cpp — 发送端入口
 *
 * 启动流程:
 *   1. ROS2 init
 *   2. 创建 VideoEncoderNode (ComposableNode 加载)
 *   3. 注册 SerialDataCallback: 300B data → serial_writer + debug_bridge
 *   4. 启动 serial_writer (USB CDC 热插拔)
 *   5. 启动 debug_bridge (fork Python 子进程)
 *   6. ROS2 spin
 *
 * 三输出: serial_writer (USB CDC) + ROS2 VideoPacket + debug_bridge (stdin pipe)
 */

#include <rclcpp/rclcpp.hpp>
#include <csignal>
#include <memory>

#include "encoder/video_encoder_node.hpp"
#include "serial/serial_writer.hpp"
#include "debug/debug_bridge.hpp"
#include "protocol/serial_frame.hpp"

using namespace sniper;

namespace {
    volatile std::sig_atomic_t g_shutdown = 0;
}

void signal_handler(int) { g_shutdown = 1; }

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // 创建编码器节点（带相机+编码器的容器由 launch 文件管理）
    // 独立运行时，手动 create node
    rclcpp::NodeOptions options;
    auto encoder_node = std::make_shared<encoder::VideoEncoderNode>(options);

    // 串口 + debug bridge
    serial::SerialWriter serial_writer;
    debug::DebugBridge debug_bridge;

    // 启动串口热插拔监控
    serial_writer.start([](const std::string &dev, bool connected) {
        if (connected) {
            fprintf(stdout, "[main] serial device inserted: %s\n", dev.c_str());
        } else {
            fprintf(stdout, "[main] serial device removed: %s\n", dev.c_str());
        }
    });

    // 启动 debug 桥接（子进程 python3 sender/debug/main.py）
    // script_dir: 从安装路径或源路径查找
    // 简化: 假设工作目录为 colcon workspace root,
    //       sender/debug/ 在 src/sender/debug/ 或 install/sender/share/sender/debug/
    std::string debug_dir;
    // 尝试相对路径
    if (access("src/sender/debug/main.py", R_OK) == 0) {
        debug_dir = "src/sender/debug";
    } else if (access("../src/sender/debug/main.py", R_OK) == 0) {
        debug_dir = "../src/sender/debug";
    } else if (access("install/sender/share/sender/debug/main.py", R_OK) == 0) {
        debug_dir = "install/sender/share/sender/debug";
    } else {
        // 最后尝试绝对路径（硬编码常见位置）
        fprintf(stdout, "[main] WARNING: cannot find debug/main.py, "
                        "debug bridge disabled\n");
    }

    if (!debug_dir.empty()) {
        if (debug_bridge.start(debug_dir)) {
            fprintf(stdout, "[main] debug bridge started\n");
        } else {
            fprintf(stderr, "[main] debug bridge failed to start\n");
        }
    }

    // 注册回调: 300B data → 构造 309B 帧 → serial + debug
    encoder_node->set_serial_data_callback(
        [&serial_writer, &debug_bridge](const uint8_t *data_300) {
            static uint8_t frame_seq = 0;
            auto frame = protocol::build_frame(frame_seq++, data_300);

            // 写串口
            serial_writer.write_frame(frame.data(), frame.size());

            // 写 debug 桥接
            debug_bridge.write_frame(frame.data(), frame.size());
        });

    fprintf(stdout, "[main] sender started: encoder + serial + debug\n");

    // spin
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(encoder_node);
    exec.spin();

    fprintf(stdout, "[main] shutting down...\n");

    debug_bridge.stop();
    serial_writer.stop();

    rclcpp::shutdown();
    return 0;
}
