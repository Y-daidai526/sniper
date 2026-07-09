#include "debug_bridge.hpp"

#include <cerrno>
#include <cstring>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <signal.h>

namespace sniper::debug {

DebugBridge::~DebugBridge() { stop(); }

bool DebugBridge::start(const std::string &script_dir) {
    int pipefd[2];
    if (pipe(pipefd) != 0) {
        fprintf(stderr, "[debug_bridge] pipe() failed: %s\n", strerror(errno));
        return false;
    }

    pid_t pid = fork();
    if (pid < 0) {
        fprintf(stderr, "[debug_bridge] fork() failed: %s\n", strerror(errno));
        close(pipefd[0]);
        close(pipefd[1]);
        return false;
    }

    if (pid == 0) {
        // --- 子进程 ---
        close(pipefd[1]);  // 关闭写端

        // 重定向 stdin 到 pipe 读端
        dup2(pipefd[0], STDIN_FILENO);
        close(pipefd[0]);

        // chdir 到 script_dir
        if (chdir(script_dir.c_str()) != 0) {
            fprintf(stderr, "[debug_bridge] chdir(%s) failed: %s\n",
                script_dir.c_str(), strerror(errno));
            _exit(1);
        }

        // exec python3 main.py
        execlp("python3", "python3", "main.py", nullptr);
        fprintf(stderr, "[debug_bridge] execlp() failed: %s\n", strerror(errno));
        _exit(1);
    }

    // --- 父进程 ---
    close(pipefd[0]);  // 关闭读端
    stdin_pipe_ = pipefd[1];
    child_pid_ = pid;

    fprintf(stdout, "[debug_bridge] child PID=%d, stdin pipe fd=%d\n",
        child_pid_, stdin_pipe_);
    return true;
}

void DebugBridge::stop() {
    if (stdin_pipe_ >= 0) {
        close(stdin_pipe_);  // 关闭 pipe → 子进程 stdin EOF
        stdin_pipe_ = -1;
    }
    if (child_pid_ > 0) {
        // 等待最多 2 秒
        int status;
        for (int i = 0; i < 20; ++i) {
            pid_t ret = waitpid(child_pid_, &status, WNOHANG);
            if (ret == child_pid_) {
                fprintf(stdout, "[debug_bridge] child PID=%d exited status=%d\n",
                    child_pid_, WIFEXITED(status) ? WEXITSTATUS(status) : -1);
                break;
            }
            if (ret < 0) break;
            usleep(100000);  // 100ms
        }
        // 仍未退出则强制 kill
        if (kill(child_pid_, 0) == 0) {
            fprintf(stdout, "[debug_bridge] killing child PID=%d\n", child_pid_);
            kill(child_pid_, SIGTERM);
            waitpid(child_pid_, nullptr, 0);
        }
        child_pid_ = 0;
    }
}

void DebugBridge::write_frame(const uint8_t *frame, size_t len) {
    if (stdin_pipe_ < 0) return;

    ssize_t written = write(stdin_pipe_, frame, len);
    if (written < 0) {
        fprintf(stderr, "[debug_bridge] write error: %s\n", strerror(errno));
    } else if (static_cast<size_t>(written) != len) {
        fprintf(stderr, "[debug_bridge] short write: %zd/%zu bytes\n", written, len);
    }
}

} // namespace sniper::debug
