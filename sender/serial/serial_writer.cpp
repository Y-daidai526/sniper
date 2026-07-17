#include "serial_writer.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <glob.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>

namespace sniper::serial {

SerialWriter::SerialWriter() = default;

SerialWriter::~SerialWriter() {
    stop();
}

void SerialWriter::start() {
    if (monitor_thread_.joinable()) {
        return;
    }
    running_ = true;
    monitor_thread_ = std::thread(&SerialWriter::monitor_loop, this);
}

void SerialWriter::stop() {
    running_ = false;
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
    close_current();
}

std::string SerialWriter::current_device() const {
    std::lock_guard<std::mutex> lock(write_mutex_);
    return fd_ >= 0 ? current_device_ : "";
}

std::string SerialWriter::find_device() {
    const char *patterns[] = {"/dev/ttyACM*", "/dev/ttyUSB*", nullptr};
    for (int i = 0; patterns[i] != nullptr; ++i) {
        glob_t gl{};
        if (glob(patterns[i], 0, nullptr, &gl) == 0) {
            std::string result;
            if (gl.gl_pathc > 0) {
                result = gl.gl_pathv[0];
            }
            globfree(&gl);
            if (!result.empty()) {
                return result;
            }
        }
    }
    return "";
}

bool SerialWriter::try_open(const std::string &device) {
    int fd = open(device.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::fprintf(stderr, "[serial] open(%s) failed: %s\n", device.c_str(), std::strerror(errno));
        return false;
    }

    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::fprintf(stderr, "[serial] tcgetattr(%s) failed: %s\n", device.c_str(), std::strerror(errno));
        close(fd);
        return false;
    }

    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | ISTRIP);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::fprintf(stderr, "[serial] tcsetattr(%s) failed: %s\n", device.c_str(), std::strerror(errno));
        close(fd);
        return false;
    }

    tcflush(fd, TCIOFLUSH);

    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        close_current_locked();
        fd_ = fd;
        current_device_ = device;
    }
    std::fprintf(stdout, "[serial] connected: %s @ 921600 bps\n", device.c_str());
    return true;
}

void SerialWriter::close_current_locked() {
    if (fd_ >= 0) {
        std::fprintf(stdout, "[serial] disconnected: %s\n", current_device_.c_str());
        close(fd_);
        fd_ = -1;
        current_device_.clear();
    }
}

void SerialWriter::close_current() {
    std::lock_guard<std::mutex> lock(write_mutex_);
    close_current_locked();
}

void SerialWriter::monitor_loop() {
    std::fprintf(stdout, "[serial] hotplug monitor started\n");

    while (running_) {
        const std::string device = find_device();
        const std::string connected_device = current_device();
        if (device.empty()) {
            if (!connected_device.empty()) {
                close_current();
            }
        } else if (connected_device.empty()) {
            try_open(device);
        } else if (device != connected_device || access(connected_device.c_str(), W_OK) != 0) {
            close_current();
            try_open(device);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void SerialWriter::write_frame(const uint8_t *frame, size_t len) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (fd_ < 0) {
        return;
    }

    size_t total_written = 0;
    while (total_written < len) {
        const ssize_t written = write(fd_, frame + total_written, len - total_written);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::fprintf(stderr, "[serial] write error: %s\n", std::strerror(errno));
            close_current_locked();
            return;
        }
        if (written == 0) {
            std::fprintf(stderr, "[serial] write returned 0\n");
            close_current_locked();
            return;
        }
        total_written += static_cast<size_t>(written);
    }

    while (tcdrain(fd_) != 0) {
        if (errno == EINTR) {
            continue;
        }
        std::fprintf(stderr, "[serial] tcdrain error: %s\n", std::strerror(errno));
        close_current_locked();
        return;
    }
}

} // namespace sniper::serial
