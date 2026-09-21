#include "safety.hpp"

namespace vecu {

SafetyManager::SafetyManager()
{
    state_ = SafetyState::INIT;
}

uint32_t SafetyManager::fault_bit(FaultCode fault)
{
    const uint16_t value =
        static_cast<uint16_t>(fault);

    if (value == 0) {
        return 0;
    }

    return 1U << ((value - 1U) % 32U);
}

void SafetyManager::set_fault(FaultCode fault)
{
    std::lock_guard<std::mutex> lock(mutex_);

    active_faults_ |= fault_bit(fault);

    update_state();
}

void SafetyManager::clear_fault(FaultCode fault)
{
    std::lock_guard<std::mutex> lock(mutex_);

    active_faults_ &= ~fault_bit(fault);

    update_state();
}

bool SafetyManager::has_fault(FaultCode fault) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    return (active_faults_ & fault_bit(fault)) != 0;
}

bool SafetyManager::has_active_faults() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    return active_faults_ != 0;
}

SafetyState SafetyManager::get_state() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    return state_;
}

void SafetyManager::update_state()
{
    /*
     * This function is called while mutex_ is held.
     */

    if (active_faults_ == 0) {
        state_ = SafetyState::NORMAL;
        return;
    }

    /*
     * Watchdog and CAN failures are treated as
     * critical faults in this first implementation.
     */

    const bool critical =
        (active_faults_ &
         (fault_bit(FaultCode::WATCHDOG_TIMEOUT) |
          fault_bit(FaultCode::CAN_COMMUNICATION))) != 0;

    if (critical) {
        state_ = SafetyState::SAFE;
    } else {
        state_ = SafetyState::DEGRADED;
    }
}

std::string SafetyManager::fault_to_string(FaultCode fault)
{
    switch (fault) {

        case FaultCode::NONE:
            return "NONE";

        case FaultCode::CAN_COMMUNICATION:
            return "CAN_COMMUNICATION";

        case FaultCode::WATCHDOG_TIMEOUT:
            return "WATCHDOG_TIMEOUT";

        case FaultCode::INVALID_ENGINE_DATA:
            return "INVALID_ENGINE_DATA";

        case FaultCode::SENSOR_TIMEOUT:
            return "SENSOR_TIMEOUT";

        case FaultCode::OVERSPEED:
            return "OVERSPEED";
    }

    return "UNKNOWN";
}

std::string SafetyManager::state_to_string(
    SafetyState state)
{
    switch (state) {

        case SafetyState::INIT:
            return "INIT";

        case SafetyState::NORMAL:
            return "NORMAL";

        case SafetyState::DEGRADED:
            return "DEGRADED";

        case SafetyState::SAFE:
            return "SAFE";
    }

    return "UNKNOWN";
}

} // namespace vecu