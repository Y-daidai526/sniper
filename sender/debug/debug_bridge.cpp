#include "debug_bridge.hpp"

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

namespace sniper::debug {

DebugBridge::~DebugBridge() {
    stop();
}

bool DebugBridge::start(
    const std::string &script_dir,
    const std::string &mqtt_host,
    int mqtt_port,
    const std::string &mqtt_topic,
    bool start_broker) {
    int pipefd[2];
    if (pipe(pipefd) != 0) {
        std::fprintf(stderr, "[debug_bridge] pipe failed: %s\n", std::strerror(errno));
        return false;
    }

    const pid_t pid = fork();
    if (pid < 0) {
        std::fprintf(stderr, "[debug_bridge] fork failed: %s\n", std::strerror(errno));
        close(pipefd[0]);
        close(pipefd[1]);
        return false;
    }

    if (pid == 0) {
        close(pipefd[1]);
        dup2(pipefd[0], STDIN_FILENO);
        close(pipefd[0]);

        if (chdir(script_dir.c_str()) != 0) {
            std::fprintf(stderr, "[debug_bridge] chdir(%s) failed: %s\n", script_dir.c_str(), std::strerror(errno));
            _exit(1);
        }

        const std::string port_arg = std::to_string(mqtt_port);
        if (start_broker) {
            execlp(
                "python3",
                "python3",
                "main.py",
                "--host",
                mqtt_host.c_str(),
                "--port",
                port_arg.c_str(),
                "--topic",
                mqtt_topic.c_str(),
                "--start-broker",
                nullptr);
        } else {
            execlp(
                "python3",
                "python3",
                "main.py",
                "--host",
                mqtt_host.c_str(),
                "--port",
                port_arg.c_str(),
                "--topic",
                mqtt_topic.c_str(),
                nullptr);
        }

        std::fprintf(stderr, "[debug_bridge] execlp failed: %s\n", std::strerror(errno));
        _exit(1);
    }

    close(pipefd[0]);
    stdin_pipe_ = pipefd[1];
    child_pid_ = pid;
    std::fprintf(stdout, "[debug_bridge] child pid=%d\n", static_cast<int>(child_pid_));
    return true;
}

void DebugBridge::stop() {
    if (stdin_pipe_ >= 0) {
        close(stdin_pipe_);
        stdin_pipe_ = -1;
    }

    if (child_pid_ <= 0) {
        return;
    }

    int status = 0;
    bool exited = false;
    for (int i = 0; i < 20; ++i) {
        const pid_t ret = waitpid(child_pid_, &status, WNOHANG);
        if (ret == child_pid_) {
            exited = true;
            std::fprintf(stdout, "[debug_bridge] child pid=%d exited\n", static_cast<int>(child_pid_));
            break;
        }
        if (ret < 0) {
            exited = true;
            break;
        }
        usleep(100000);
    }

    if (!exited && kill(child_pid_, 0) == 0) {
        std::fprintf(stdout, "[debug_bridge] terminating child pid=%d\n", static_cast<int>(child_pid_));
        kill(child_pid_, SIGTERM);
        waitpid(child_pid_, nullptr, 0);
    }

    child_pid_ = 0;
}

void DebugBridge::write_frame(const uint8_t *frame, size_t len) {
    if (stdin_pipe_ < 0) {
        return;
    }

    size_t total_written = 0;
    while (total_written < len) {
        const ssize_t written = write(stdin_pipe_, frame + total_written, len - total_written);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::fprintf(stderr, "[debug_bridge] write error: %s\n", std::strerror(errno));
            close(stdin_pipe_);
            stdin_pipe_ = -1;
            return;
        }
        if (written == 0) {
            std::fprintf(stderr, "[debug_bridge] write returned 0\n");
            close(stdin_pipe_);
            stdin_pipe_ = -1;
            return;
        }
        total_written += static_cast<size_t>(written);
    }
}

} // namespace sniper::debug
