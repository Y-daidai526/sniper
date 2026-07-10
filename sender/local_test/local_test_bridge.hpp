#ifndef SNIPER_LOCAL_TEST_LOCAL_TEST_BRIDGE_HPP_
#define SNIPER_LOCAL_TEST_LOCAL_TEST_BRIDGE_HPP_

#include <cstdint>
#include <string>

#include <sys/types.h>

namespace sniper::local_test {

class LocalTestBridge {
public:
    LocalTestBridge() = default;
    ~LocalTestBridge();

    bool start(
        const std::string &script_dir,
        const std::string &mqtt_host,
        int mqtt_port,
        const std::string &mqtt_topic,
        bool start_broker);
    void stop();
    bool write_frame(const uint8_t *frame, size_t len);

    bool is_running() const { return child_pid_ > 0; }

private:
    int stdin_pipe_ = -1;
    pid_t child_pid_ = 0;
};

} // namespace sniper::local_test

#endif
