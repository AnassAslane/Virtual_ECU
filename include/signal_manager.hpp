#pragma once

#include "can.hpp"

#include <cstdint>

namespace vecu {

struct VehicleSignals
{
    // Engine
    uint16_t rpm{0};
    float speed_kmh{0.0F};
    uint8_t throttle_percent{0};
    int8_t engine_temperature_c{0};
    bool engine_running{false};

    // Brakes
    uint8_t brake_percent{0};
    uint8_t brake_pressure{0};
    bool abs_active{false};
    bool brake_switch{false};

    // Vehicle
    float acceleration_mps2{0.0F};
    uint8_t gear{0};
    uint8_t vehicle_state{0};
};

class SignalManager
{
public:
    SignalManager() = default;

    bool process_frame(
        const CanFrame& frame);

    const VehicleSignals& get_signals() const;

    bool has_engine_data() const;
    bool has_brake_data() const;
    bool has_vehicle_data() const;

private:
    bool decode_engine_status(
        const CanFrame& frame);

    bool decode_brake_status(
        const CanFrame& frame);

    bool decode_vehicle_status(
        const CanFrame& frame);

    VehicleSignals signals_;

    bool engine_data_received_{false};
    bool brake_data_received_{false};
    bool vehicle_data_received_{false};

    static constexpr uint32_t ENGINE_STATUS_ID = 0x100;
    static constexpr uint32_t BRAKE_STATUS_ID = 0x110;
    static constexpr uint32_t VEHICLE_STATUS_ID = 0x120;
};

} // namespace vecu