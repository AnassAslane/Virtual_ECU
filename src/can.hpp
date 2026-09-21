#pragma once

#include <cstdint>
#include <string>

namespace vecu {

struct EngineData {
    uint16_t rpm{0};
    uint16_t speed_kmh{0};
    uint8_t throttle_percent{0};
};

class CanBus {
public:
    CanBus() = default;
    ~CanBus();

    CanBus(const CanBus&) = delete;
    CanBus& operator=(const CanBus&) = delete;

    bool open(const std::string& interface_name);
    void close();

    bool send_engine_data(const EngineData& data);
    bool receive(uint32_t& can_id,
                 uint8_t* data,
                 uint8_t& dlc,
                 int timeout_ms);

    bool is_open() const;

private:
    int socket_fd_{-1};
    int interface_index_{-1};

    static constexpr uint32_t ENGINE_DATA_ID = 0x100;
};

} // namespace vecu