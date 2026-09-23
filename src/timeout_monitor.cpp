#include "timeout_monitor.hpp"

namespace vecu {

TimeoutMonitor::TimeoutMonitor()
{
    const auto now = std::chrono::steady_clock::now();

    messages_.emplace(0x100U,
        MonitoredMessage{std::chrono::milliseconds(300), now, false, false});
    messages_.emplace(0x110U,
        MonitoredMessage{std::chrono::milliseconds(150), now, false, false});
    messages_.emplace(0x120U,
        MonitoredMessage{std::chrono::milliseconds(300), now, false, false});
}

void TimeoutMonitor::frame_received(uint32_t can_id)
{
    const auto it = messages_.find(can_id);
    if (it == messages_.end()) {
        return;
    }

    it->second.last_received = std::chrono::steady_clock::now();
    it->second.received = true;
    it->second.timed_out = false;
}

void TimeoutMonitor::update()
{
    const auto now = std::chrono::steady_clock::now();

    for (auto& [id, message] : messages_) {
        (void)id;

        if (!message.received) {
            continue;
        }

        const auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - message.last_received);

        message.timed_out = elapsed > message.timeout;
    }
}

TimeoutStatus TimeoutMonitor::status(uint32_t can_id) const
{
    const auto it = messages_.find(can_id);

    if (it == messages_.end() || !it->second.received ||
        it->second.timed_out) {
        return TimeoutStatus::TIMEOUT;
    }

    return TimeoutStatus::OK;
}

bool TimeoutMonitor::engine_timeout() const
{
    return status(0x100U) == TimeoutStatus::TIMEOUT;
}

bool TimeoutMonitor::brake_timeout() const
{
    return status(0x110U) == TimeoutStatus::TIMEOUT;
}

bool TimeoutMonitor::vehicle_timeout() const
{
    return status(0x120U) == TimeoutStatus::TIMEOUT;
}

} // namespace vecu