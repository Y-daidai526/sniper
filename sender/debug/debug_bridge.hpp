#ifndef SNIPER_DEBUG_DEBUG_BRIDGE_HPP_
#define SNIPER_DEBUG_DEBUG_BRIDGE_HPP_

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>

namespace sniper::debug {

// fork+exec Python 子进程，通过 stdin pipe 传递 309B 帧
class DebugBridge {
public:
    DebugBridge() = default;
    ~DebugBridge();

    // 启动子进程: python3 <script_dir>/main.py
    // script_dir: sender/debug/ 的绝对路径
    bool start(const std::string &script_dir);

    // 关闭 pipe，等待子进程退出
    void stop();

    // 写 309B 帧到子进程 stdin
    void write_frame(const uint8_t *frame, size_t len);

    bool is_running() const { return child_pid_ > 0; }

private:
    int stdin_pipe_ = -1;   // 父进程写端
    pid_t child_pid_ = 0;
};

} // namespace sniper::debug

#endif
