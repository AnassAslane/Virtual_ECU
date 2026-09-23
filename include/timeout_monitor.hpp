#pragma once

#include <chrono>
#include <cstdint>
#include <map>

namespace vecu {

enum class TimeoutStatus {
    OK,
    TIMEOUT
};

class TimeoutMonitor {
public:
    TimeoutMonitor();

    void frame_received(uint32_t can_id);
    void update();

    TimeoutStatus status(uint32_t can_id) const;

    bool engine_timeout() const;
    bool brake_timeout() const;
    bool vehicle_timeout() const;

private:
    struct MonitoredMessage {
        std::chrono::milliseconds timeout;
        std::chrono::steady_clock::time_point last_received;
        bool received{false};
        bool timed_out{false};
    };

    std::map<uint32_t, MonitoredMessage> messages_;
};

} // namespace vecu