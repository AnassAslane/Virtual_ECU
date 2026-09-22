#include "signal_manager.hpp"

#include <cstdint>

namespace vecu {

bool SignalManager::process_frame(
    const CanFrame& frame)
{
    switch (frame.id)
    {
        case ENGINE_STATUS_ID:
            return decode_engine_status(frame);

        case BRAKE_STATUS_ID:
            return decode_brake_status(frame);

        case VEHICLE_STATUS_ID:
            return decode_vehicle_status(frame);

        default:
            return false;
    }
}

bool SignalManager::decode_engine_status(
    const CanFrame& frame)
{
    if (frame.dlc != 7U)
    {
        return false;
    }

    /*
     * 0x100 ENGINE_STATUS
     *
     * Byte 0-1 : RPM
     * Byte 2-3 : Speed x10
     * Byte 4   : Throttle %
     * Byte 5   : Engine temperature
     * Byte 6   : Engine running
     */

    const uint16_t rpm =
        (static_cast<uint16_t>(frame.data[0]) << 8U) |
        static_cast<uint16_t>(frame.data[1]);

    const uint16_t speed_raw =
        (static_cast<uint16_t>(frame.data[2]) << 8U) |
        static_cast<uint16_t>(frame.data[3]);

    signals_.rpm = rpm;

    signals_.speed_kmh =
        static_cast<float>(speed_raw) / 10.0F;

    signals_.throttle_percent =
        frame.data[4];

    signals_.engine_temperature_c =
        static_cast<int8_t>(frame.data[5]);

    signals_.engine_running =
        frame.data[6] != 0U;

    engine_data_received_ = true;

    return true;
}

bool SignalManager::decode_brake_status(
    const CanFrame& frame)
{
    if (frame.dlc != 4U)
    {
        return false;
    }

    /*
     * 0x110 BRAKE_STATUS
     *
     * Byte 0 : Brake pedal %
     * Byte 1 : Brake pressure
     * Byte 2 : ABS active
     * Byte 3 : Brake switch
     */

    signals_.brake_percent =
        frame.data[0];

    signals_.brake_pressure =
        frame.data[1];

    signals_.abs_active =
        frame.data[2] != 0U;

    signals_.brake_switch =
        frame.data[3] != 0U;

    brake_data_received_ = true;

    return true;
}

bool SignalManager::decode_vehicle_status(
    const CanFrame& frame)
{
    if (frame.dlc != 6U)
    {
        return false;
    }

    /*
     * 0x120 VEHICLE_STATUS
     *
     * Byte 0-1 : Acceleration x100
     * Byte 2-3 : Speed x10
     * Byte 4   : Gear
     * Byte 5   : Vehicle state
     */

    const int16_t acceleration_raw =
        static_cast<int16_t>(
            (static_cast<uint16_t>(frame.data[0]) << 8U) |
            static_cast<uint16_t>(frame.data[1]));

    const uint16_t speed_raw =
        (static_cast<uint16_t>(frame.data[2]) << 8U) |
        static_cast<uint16_t>(frame.data[3]);

    signals_.acceleration_mps2 =
        static_cast<float>(
            acceleration_raw) / 100.0F;

    signals_.speed_kmh =
        static_cast<float>(
            speed_raw) / 10.0F;

    signals_.gear =
        frame.data[4];

    signals_.vehicle_state =
        frame.data[5];

    vehicle_data_received_ = true;

    return true;
}

const VehicleSignals&
SignalManager::get_signals() const
{
    return signals_;
}

bool SignalManager::has_engine_data() const
{
    return engine_data_received_;
}

bool SignalManager::has_brake_data() const
{
    return brake_data_received_;
}

bool SignalManager::has_vehicle_data() const
{
    return vehicle_data_received_;
}

} // namespace vecu