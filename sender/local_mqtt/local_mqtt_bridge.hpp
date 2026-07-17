#ifndef SNIPER_LOCAL_MQTT_LOCAL_MQTT_BRIDGE_HPP_
#define SNIPER_LOCAL_MQTT_LOCAL_MQTT_BRIDGE_HPP_

#include <cstdint>
#include <string>

#include <sys/types.h>

namespace sniper::local_mqtt {

class LocalMqttBridge {
public:
    LocalMqttBridge() = default;
    ~LocalMqttBridge();

    bool start(
        const std::string &script_dir,
        const std::string &mqtt_host,
        int mqtt_port,
        const std::string &mqtt_topic,
        bool start_broker);
    void stop();
    void write_frame(const uint8_t *frame, size_t len);

private:
    int stdin_pipe_ = -1;
    pid_t child_pid_ = 0;
};

} // namespace sniper::local_mqtt

#endif
