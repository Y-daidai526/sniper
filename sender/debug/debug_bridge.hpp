#ifndef SNIPER_DEBUG_DEBUG_BRIDGE_HPP_
#define SNIPER_DEBUG_DEBUG_BRIDGE_HPP_

#include <cstdint>
#include <string>

#include <sys/types.h>

namespace sniper::debug {

class DebugBridge {
public:
    DebugBridge() = default;
    ~DebugBridge();

    bool start(
        const std::string &script_dir,
        const std::string &mqtt_host,
        int mqtt_port,
        const std::string &mqtt_topic,
        bool start_broker);
    void stop();
    void write_frame(const uint8_t *frame, size_t len);

    bool is_running() const { return child_pid_ > 0; }

private:
    int stdin_pipe_ = -1;
    pid_t child_pid_ = 0;
};

} // namespace sniper::debug

#endif
