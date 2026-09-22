#pragma once

#include <cstdint>
#include <string>

namespace vecu {

struct CanFrame
{
    uint32_t id{0};
    uint8_t dlc{0};
    uint8_t data[8]{};
};

class CanBus
{
public:
    CanBus() = default;
    ~CanBus();

    CanBus(const CanBus&) = delete;
    CanBus& operator=(const CanBus&) = delete;

    bool open(const std::string& interface_name);
    void close();

    bool receive(
        CanFrame& frame,
        int timeout_ms);

    bool send(
        uint32_t can_id,
        const uint8_t* data,
        uint8_t dlc);

    bool is_open() const;

private:
    int socket_fd_{-1};
    int interface_index_{-1};
};

} // namespace vecu