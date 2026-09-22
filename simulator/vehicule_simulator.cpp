#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <thread>
#include <csignal>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <unistd.h>

using namespace std::chrono_literals;

namespace {

constexpr const char* CAN_INTERFACE = "vcan0";

constexpr canid_t ENGINE_STATUS_ID = 0x100;
constexpr canid_t BRAKE_STATUS_ID  = 0x110;
constexpr canid_t VEHICLE_STATUS_ID = 0x120;

constexpr auto ENGINE_PERIOD = 100ms;
constexpr auto BRAKE_PERIOD = 50ms;
constexpr auto VEHICLE_PERIOD = 100ms;

std::atomic<bool> running{true};

void signal_handler(int)
{
    running = false;
}

class CanInterface {
public:

    CanInterface() = default;

    ~CanInterface()
    {
        close();
    }

    CanInterface(const CanInterface&) = delete;
    CanInterface& operator=(const CanInterface&) = delete;

    bool open(const char* interface_name)
    {
        socket_fd_ = socket(
            PF_CAN,
            SOCK_RAW,
            CAN_RAW);

        if (socket_fd_ < 0) {

            std::cerr
                << "[CAN] socket() failed: "
                << std::strerror(errno)
                << '\n';

            return false;
        }

        struct ifreq ifr {};

        std::strncpy(
            ifr.ifr_name,
            interface_name,
            IFNAMSIZ - 1);

        if (ioctl(
                socket_fd_,
                SIOCGIFINDEX,
                &ifr) < 0) {

            std::cerr
                << "[CAN] Cannot find interface "
                << interface_name
                << ": "
                << std::strerror(errno)
                << '\n';

            close();

            return false;
        }

        struct sockaddr_can address {};

        address.can_family = AF_CAN;
        address.can_ifindex = ifr.ifr_ifindex;

        if (bind(
                socket_fd_,
                reinterpret_cast<
                    struct sockaddr*>(&address),
                sizeof(address)) < 0) {

            std::cerr
                << "[CAN] bind() failed: "
                << std::strerror(errno)
                << '\n';

            close();

            return false;
        }

        std::cout
            << "[CAN] Connected to "
            << interface_name
            << '\n';

        return true;
    }

    bool send(const can_frame& frame)
    {
        if (socket_fd_ < 0) {
            return false;
        }

        const ssize_t result =
            write(
                socket_fd_,
                &frame,
                sizeof(frame));

        if (result != sizeof(frame)) {

            std::cerr
                << "[CAN] TX failed: "
                << std::strerror(errno)
                << '\n';

            return false;
        }

        return true;
    }

    void close()
    {
        if (socket_fd_ >= 0) {
            ::close(socket_fd_);
            socket_fd_ = -1;
        }
    }

private:
    int socket_fd_{-1};
};

/*
 * ------------------------------------------------------------
 * Vehicle model
 * ------------------------------------------------------------
 *
 * This is intentionally simple at first.
 *
 * throttle:
 *     0.0 -> 100.0 %
 *
 * speed:
 *     km/h
 *
 * rpm:
 *     engine RPM
 *
 * brake:
 *     0.0 -> 100.0 %
 *
 * The model is deterministic enough to test the ECU,
 * but it is NOT intended to be a physically accurate
 * vehicle dynamics model.
 */

struct VehicleState {

    double speed_kmh = 0.0;

    double rpm = 900.0;

    double throttle_percent = 0.0;

    double brake_percent = 0.0;

    double acceleration_mps2 = 0.0;

    double engine_temperature = 75.0;

    uint8_t gear = 1;

    bool abs_active = false;

    bool brake_switch = false;

    bool engine_running = true;
};

/*
 * ------------------------------------------------------------
 * Update vehicle physics
 * ------------------------------------------------------------
 */

void update_vehicle(
    VehicleState& vehicle,
    double dt_seconds)
{
    /*
     * Very simplified longitudinal vehicle model.
     */

    const double throttle =
        vehicle.throttle_percent / 100.0;

    const double brake =
        vehicle.brake_percent / 100.0;

    /*
     * Approximate driving acceleration.
     *
     * Maximum acceleration decreases slightly
     * at higher speeds.
     */

    double drive_force =
        3.5 * throttle;

    const double aerodynamic_drag =
        0.0025 *
        vehicle.speed_kmh *
        vehicle.speed_kmh;

    const double brake_force =
        7.0 * brake;

    vehicle.acceleration_mps2 =
        drive_force
        - aerodynamic_drag
        - brake_force;

    /*
     * Integrate acceleration.
     */

    const double speed_mps =
        vehicle.speed_kmh / 3.6;

    const double new_speed_mps =
        std::max(
            0.0,
            speed_mps +
            vehicle.acceleration_mps2 *
            dt_seconds);

    vehicle.speed_kmh =
        new_speed_mps * 3.6;

    /*
     * Very simple automatic transmission model.
     */

    if (vehicle.speed_kmh < 15.0) {
        vehicle.gear = 1;
    }
    else if (vehicle.speed_kmh < 35.0) {
        vehicle.gear = 2;
    }
    else if (vehicle.speed_kmh < 60.0) {
        vehicle.gear = 3;
    }
    else if (vehicle.speed_kmh < 90.0) {
        vehicle.gear = 4;
    }
    else {
        vehicle.gear = 5;
    }

    /*
     * Approximate RPM from vehicle speed and gear.
     */

    const double gear_ratio[] = {
        0.0,
        90.0,
        55.0,
        38.0,
        28.0,
        22.0
    };

    const double ratio =
        gear_ratio[
            std::min(
                static_cast<size_t>(vehicle.gear),
                sizeof(gear_ratio) /
                sizeof(gear_ratio[0]) - 1)];

    if (vehicle.engine_running) {

        vehicle.rpm =
            800.0 +
            vehicle.speed_kmh * ratio;

        /*
         * Throttle influences RPM slightly.
         */

        vehicle.rpm +=
            500.0 * throttle;

        vehicle.rpm =
            std::clamp(
                vehicle.rpm,
                800.0,
                6500.0);

    } else {

        vehicle.rpm = 0.0;
    }

    /*
     * Simple thermal model.
     */

    const double heat =
        throttle * 0.5;

    const double cooling =
        (vehicle.engine_temperature - 70.0)
        * 0.02;

    vehicle.engine_temperature +=
        (heat - cooling) *
        dt_seconds *
        10.0;

    vehicle.engine_temperature =
        std::clamp(
            vehicle.engine_temperature,
            60.0,
            130.0);

    /*
     * Brake logic.
     */

    vehicle.brake_switch =
        vehicle.brake_percent > 1.0;

    /*
     * ABS activates under strong braking
     * while the vehicle is moving.
     */

    vehicle.abs_active =
        vehicle.brake_percent > 80.0 &&
        vehicle.speed_kmh > 10.0;
}

/*
 * ------------------------------------------------------------
 * CAN encoding helpers
 * ------------------------------------------------------------
 */

void encode_engine_status(
    const VehicleState& vehicle,
    can_frame& frame)
{
    frame = {};

    frame.can_id = ENGINE_STATUS_ID;
    frame.can_dlc = 7;

    const uint16_t rpm =
        static_cast<uint16_t>(
            std::clamp(
                vehicle.rpm,
                0.0,
                65535.0));

    const uint16_t speed_x10 =
        static_cast<uint16_t>(
            std::clamp(
                vehicle.speed_kmh * 10.0,
                0.0,
                65535.0));

    const uint8_t throttle =
        static_cast<uint8_t>(
            std::clamp(
                vehicle.throttle_percent,
                0.0,
                100.0));

    const uint8_t temperature =
        static_cast<uint8_t>(
            std::clamp(
                vehicle.engine_temperature,
                0.0,
                255.0));

    frame.data[0] =
        static_cast<uint8_t>(
            (rpm >> 8) & 0xFF);

    frame.data[1] =
        static_cast<uint8_t>(
            rpm & 0xFF);

    frame.data[2] =
        static_cast<uint8_t>(
            (speed_x10 >> 8) & 0xFF);

    frame.data[3] =
        static_cast<uint8_t>(
            speed_x10 & 0xFF);

    frame.data[4] = throttle;

    frame.data[5] = temperature;

    frame.data[6] =
        vehicle.engine_running ? 1 : 0;
}

void encode_brake_status(
    const VehicleState& vehicle,
    can_frame& frame)
{
    frame = {};

    frame.can_id = BRAKE_STATUS_ID;
    frame.can_dlc = 4;

    frame.data[0] =
        static_cast<uint8_t>(
            std::clamp(
                vehicle.brake_percent,
                0.0,
                100.0));

    /*
     * Simplified brake pressure.
     */

    frame.data[1] =
        static_cast<uint8_t>(
            std::clamp(
                vehicle.brake_percent * 2.0,
                0.0,
                200.0));

    frame.data[2] =
        vehicle.abs_active ? 1 : 0;

    frame.data[3] =
        vehicle.brake_switch ? 1 : 0;
}

void encode_vehicle_status(
    const VehicleState& vehicle,
    can_frame& frame)
{
    frame = {};

    frame.can_id = VEHICLE_STATUS_ID;
    frame.can_dlc = 6;

    const int16_t acceleration_x100 =
        static_cast<int16_t>(
            std::clamp(
                vehicle.acceleration_mps2 * 100.0,
                -32768.0,
                32767.0));

    const uint16_t speed_x10 =
        static_cast<uint16_t>(
            std::clamp(
                vehicle.speed_kmh * 10.0,
                0.0,
                65535.0));

    frame.data[0] =
        static_cast<uint8_t>(
            (acceleration_x100 >> 8) & 0xFF);

    frame.data[1] =
        static_cast<uint8_t>(
            acceleration_x100 & 0xFF);

    frame.data[2] =
        static_cast<uint8_t>(
            (speed_x10 >> 8) & 0xFF);

    frame.data[3] =
        static_cast<uint8_t>(
            speed_x10 & 0xFF);

    frame.data[4] =
        vehicle.gear;

    /*
     * Vehicle state:
     *
     * 0 = stopped
     * 1 = driving
     */

    frame.data[5] =
        vehicle.speed_kmh > 0.5 ? 1 : 0;
}

/*
 * ------------------------------------------------------------
 * Main simulator
 * ------------------------------------------------------------
 */
}

int main()
{
    std::signal(
        SIGINT,
        signal_handler);

    std::signal(
        SIGTERM,
        signal_handler);

    std::cout
        << "========================================\n"
        << "       VECU-X Vehicle Simulator\n"
        << "========================================\n";

    CanInterface can;

    if (!can.open(CAN_INTERFACE)) {

        std::cerr
            << "[SIM] Failed to initialize CAN\n";

        return 1;
    }

    VehicleState vehicle;

    auto last_update =
        std::chrono::steady_clock::now();

    auto next_engine =
        last_update;

    auto next_brake =
        last_update;

    auto next_vehicle =
        last_update;

    /*
     * Simulation starts by accelerating.
     */

    double simulation_time = 0.0;

    while (running) {

        const auto now =
            std::chrono::steady_clock::now();

        const double dt =
            std::chrono::duration<double>(
                now - last_update).count();

        last_update = now;

        simulation_time += dt;

        /*
         * ----------------------------------------------------
         * Driver scenario
         * ----------------------------------------------------
         *
         * 0-10 sec:
         *     accelerate
         *
         * 10-15 sec:
         *     cruise
         *
         * 15-20 sec:
         *     brake
         *
         * 20+ sec:
         *     accelerate again
         */

        if (simulation_time < 10.0) {

            vehicle.throttle_percent = 40.0;
            vehicle.brake_percent = 0.0;

        } else if (simulation_time < 15.0) {

            vehicle.throttle_percent = 15.0;
            vehicle.brake_percent = 0.0;

        } else if (simulation_time < 20.0) {

            vehicle.throttle_percent = 0.0;
            vehicle.brake_percent = 60.0;

        } else {

            vehicle.throttle_percent = 35.0;
            vehicle.brake_percent = 0.0;
        }

        /*
         * Update physical state.
         */

        update_vehicle(
            vehicle,
            std::clamp(
                dt,
                0.0,
                0.1));

        /*
         * ----------------------------------------------------
         * Engine CAN message
         * ----------------------------------------------------
         */

        if (now >= next_engine) {

            can_frame frame{};

            encode_engine_status(
                vehicle,
                frame);

            if (can.send(frame)) {

                std::cout
                    << std::fixed
                    << std::setprecision(1)
                    << "[ENGINE] RPM="
                    << vehicle.rpm
                    << " Speed="
                    << vehicle.speed_kmh
                    << " km/h"
                    << " Throttle="
                    << vehicle.throttle_percent
                    << "%\n";
            }

            next_engine += ENGINE_PERIOD;
        }

        /*
         * ----------------------------------------------------
         * Brake CAN message
         * ----------------------------------------------------
         */

        if (now >= next_brake) {

            can_frame frame{};

            encode_brake_status(
                vehicle,
                frame);

            can.send(frame);

            next_brake += BRAKE_PERIOD;
        }

        /*
         * ----------------------------------------------------
         * Vehicle dynamics CAN message
         * ----------------------------------------------------
         */

        if (now >= next_vehicle) {

            can_frame frame{};

            encode_vehicle_status(
                vehicle,
                frame);

            can.send(frame);

            next_vehicle += VEHICLE_PERIOD;
        }

        /*
         * Avoid consuming an entire CPU core.
         */

        std::this_thread::sleep_for(1ms);
    }

    std::cout
        << "\n[SIM] Vehicle simulator stopped\n";

    return 0;
}
