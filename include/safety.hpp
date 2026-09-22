#pragma once

#include <cstdint>
#include <mutex>
#include <string>

namespace vecu {

enum class SafetyState {
    INIT,
    NORMAL,
    DEGRADED,
    SAFE
};

enum class FaultCode : uint16_t {
    NONE = 0x0000,

    CAN_COMMUNICATION = 0x1001,
    WATCHDOG_TIMEOUT = 0x1002,
    INVALID_ENGINE_DATA = 0x1003,
    SENSOR_TIMEOUT = 0x1004,
    OVERSPEED = 0x1005
};

class SafetyManager {
public:
    SafetyManager();

    void set_fault(FaultCode fault);

    void clear_fault(FaultCode fault);

    bool has_fault(FaultCode fault) const;

    bool has_active_faults() const;

    SafetyState get_state() const;

    void update_state();

    static std::string fault_to_string(FaultCode fault);

    static std::string state_to_string(SafetyState state);

private:
    mutable std::mutex mutex_;

    uint32_t active_faults_{0};

    SafetyState state_{SafetyState::INIT};

    static uint32_t fault_bit(FaultCode fault);
};

} // namespace vecu