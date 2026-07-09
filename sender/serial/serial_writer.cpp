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

SerialWriter::~SerialWriter() { stop(); }

void SerialWriter::start(StatusCallback cb) {
    status_cb_ = std::move(cb);
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

std::string SerialWriter::find_device() {
    // 按优先级扫描 ttyACM* → ttyUSB*
    const char *patterns[] = {"/dev/ttyACM*", "/dev/ttyUSB*", nullptr};
    for (int i = 0; patterns[i]; ++i) {
        glob_t gl;
        if (glob(patterns[i], 0, nullptr, &gl) == 0) {
            std::string result;
            if (gl.gl_pathc > 0) {
                result = gl.gl_pathv[0];
            }
            globfree(&gl);
            if (!result.empty()) return result;
        }
    }
    return "";
}

bool SerialWriter::try_open(const std::string &device) {
    int fd = open(device.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "[serial] open(%s) failed: %s\n",
            device.c_str(), strerror(errno));
        return false;
    }

    // 设为阻塞模式
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "[serial] tcgetattr(%s) failed: %s\n",
            device.c_str(), strerror(errno));
        close(fd);
        return false;
    }

    // 921600 8N1, no flow control
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);

    tty.c_cflag &= ~PARENB;   // no parity
    tty.c_cflag &= ~CSTOPB;   // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       // 8 data bits
    tty.c_cflag &= ~CRTSCTS;  // no HW flow control
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | ISTRIP);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "[serial] tcsetattr(%s) failed: %s\n",
            device.c_str(), strerror(errno));
        close(fd);
        return false;
    }

    // flush
    tcflush(fd, TCIOFLUSH);

    fd_ = fd;
    current_device_ = device;
    fprintf(stdout, "[serial] connected: %s @ %d bps\n", device.c_str(), kSerialBaudRate);
    return true;
}

void SerialWriter::close_current() {
    if (fd_ >= 0) {
        fprintf(stdout, "[serial] disconnected: %s\n", current_device_.c_str());
        close(fd_);
        fd_ = -1;
        current_device_.clear();
    }
}

void SerialWriter::monitor_loop() {
    std::string last_device;
    bool was_connected = false;

    fprintf(stdout, "[serial] hotplug monitor started\n");

    while (running_) {
        std::string device = find_device();

        if (device != last_device) {
            if (device.empty()) {
                // 设备拔出
                if (was_connected) {
                    close_current();
                    was_connected = false;
                    if (status_cb_) status_cb_(last_device, false);
                }
            } else {
                // 新设备插入
                close_current();
                if (try_open(device)) {
                    was_connected = true;
                    if (status_cb_) status_cb_(device, true);
                }
            }
            last_device = device;
        }

        // 已有设备时检查是否仍然存在
        if (was_connected) {
            if (access(last_device.c_str(), W_OK) != 0) {
                // 设备文件消失
                close_current();
                was_connected = false;
                last_device.clear();
                if (status_cb_) status_cb_("", false);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void SerialWriter::write_frame(const uint8_t *frame, size_t len) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (fd_ < 0) return;  // 无串口，静默丢弃

    ssize_t written = write(fd_, frame, len);
    if (written < 0) {
        fprintf(stderr, "[serial] write error: %s\n", strerror(errno));
    } else if (static_cast<size_t>(written) != len) {
        fprintf(stderr, "[serial] short write: %zd/%zu bytes\n", written, len);
    }
}

} // namespace sniper::serial
