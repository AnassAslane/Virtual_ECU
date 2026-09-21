#include "can.hpp"
#include "diagnostics.hpp"
#include "safety.hpp"
#include "scheduler.hpp"
#include "watchdog.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;

namespace {

std::atomic<bool> running{true};

void signal_handler(int)
{
    running = false;
}

} // namespace

int main()
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "====================================\n";
    std::cout << "       VECU-X Virtual ECU\n";
    std::cout << "====================================\n";

    /*
     * -------------------------------------------------------
     * Core ECU components
     * -------------------------------------------------------
     */

    vecu::CanBus can;
    vecu::SafetyManager safety;
    vecu::Diagnostics diagnostics;
    vecu::Watchdog watchdog(500);
    vecu::Scheduler scheduler;

    /*
     * -------------------------------------------------------
     * CAN initialization
     * -------------------------------------------------------
     */

    if (!can.open("vcan0")) {

        std::cerr
            << "[ECU] CAN initialization failed\n";

        safety.set_fault(
            vecu::FaultCode::CAN_COMMUNICATION);

        diagnostics.record_fault(
            vecu::FaultCode::CAN_COMMUNICATION);

    } else {

        std::cout
            << "[ECU] CAN initialized successfully\n";
    }

    /*
     * -------------------------------------------------------
     * Watchdog task registration
     * -------------------------------------------------------
     */

    watchdog.register_task("CAN_TX");
    watchdog.register_task("SAFETY");
    watchdog.register_task("DIAGNOSTICS");

    /*
     * -------------------------------------------------------
     * ECU state
     * -------------------------------------------------------
     */

    vecu::EngineData engine_data;

    engine_data.rpm = 1000;
    engine_data.speed_kmh = 0;
    engine_data.throttle_percent = 10;

    /*
     * -------------------------------------------------------
     * Task 1:
     * CAN transmission
     *
     * 100 Hz is common for many fast control signals,
     * but this demo uses 10 Hz for engine data.
     * -------------------------------------------------------
     */

    scheduler.add_task(
        "CAN_TX",
        100ms,
        [&]() {

            if (!can.is_open()) {
                safety.set_fault(
                    vecu::FaultCode::CAN_COMMUNICATION);

                diagnostics.record_fault(
                    vecu::FaultCode::CAN_COMMUNICATION);

                return;
            }

            /*
             * Simple simulated engine evolution.
             */

            if (engine_data.speed_kmh < 120) {
                ++engine_data.speed_kmh;
            }

            if (engine_data.rpm < 3500) {
                engine_data.rpm += 25;
            }

            if (engine_data.throttle_percent < 80) {
                ++engine_data.throttle_percent;
            }

            if (!can.send_engine_data(engine_data)) {

                safety.set_fault(
                    vecu::FaultCode::CAN_COMMUNICATION);

                diagnostics.record_fault(
                    vecu::FaultCode::CAN_COMMUNICATION);

            } else {

                watchdog.kick("CAN_TX");

                std::cout
                    << "[CAN TX] RPM="
                    << engine_data.rpm
                    << " SPEED="
                    << engine_data.speed_kmh
                    << " km/h THROTTLE="
                    << static_cast<int>(
                        engine_data.throttle_percent)
                    << "%\n";
            }
        });

    /*
     * -------------------------------------------------------
     * Task 2:
     * Safety monitoring
     *
     * Runs every 50 ms.
     * -------------------------------------------------------
     */

    scheduler.add_task(
        "SAFETY",
        50ms,
        [&]() {

            /*
             * Example plausibility check.
             */

            if (engine_data.throttle_percent > 100) {

                safety.set_fault(
                    vecu::FaultCode::INVALID_ENGINE_DATA);

                diagnostics.record_fault(
                    vecu::FaultCode::INVALID_ENGINE_DATA);
            }

            /*
             * Example overspeed threshold.
             *
             * This is a project-defined demonstration
             * threshold, not a vehicle safety requirement.
             */

            if (engine_data.speed_kmh > 130) {

                safety.set_fault(
                    vecu::FaultCode::OVERSPEED);

                diagnostics.record_fault(
                    vecu::FaultCode::OVERSPEED);

            } else {

                safety.clear_fault(
                    vecu::FaultCode::OVERSPEED);

                diagnostics.clear_fault(
                    vecu::FaultCode::OVERSPEED);
            }

            safety.update_state();

            watchdog.kick("SAFETY");
        });

    /*
     * -------------------------------------------------------
     * Task 3:
     * Diagnostic monitoring
     *
     * Runs every 100 ms.
     * -------------------------------------------------------
     */

    scheduler.add_task(
        "DIAGNOSTICS",
        100ms,
        [&]() {

            const auto dtcs =
                diagnostics.get_dtcs();

            if (!dtcs.empty()) {

                std::cout
                    << "[DTC] Active faults: "
                    << dtcs.size()
                    << '\n';

                for (const auto& dtc : dtcs) {

                    std::cout
                        << "       0x"
                        << std::hex
                        << std::uppercase
                        << dtc.code
                        << std::dec
                        << " "
                        << vecu::SafetyManager::
                           fault_to_string(
                               dtc.fault)
                        << '\n';
                }
            }

            watchdog.kick("DIAGNOSTICS");
        });

    /*
     * -------------------------------------------------------
     * Task 4:
     * Watchdog supervision
     *
     * The watchdog itself must run faster than the
     * monitored task timeout.
     * -------------------------------------------------------
     */

    scheduler.add_task(
        "WATCHDOG",
        100ms,
        [&]() {

            if (!watchdog.is_healthy()) {

                const std::string failed =
                    watchdog.get_failed_task();

                std::cerr
                    << "[WATCHDOG] TIMEOUT: "
                    << failed
                    << '\n';

                safety.set_fault(
                    vecu::FaultCode::WATCHDOG_TIMEOUT);

                diagnostics.record_fault(
                    vecu::FaultCode::WATCHDOG_TIMEOUT);
            }
        });

    /*
     * -------------------------------------------------------
     * Start ECU scheduler
     * -------------------------------------------------------
     */

    safety.update_state();

    std::cout
        << "[ECU] Initial safety state: "
        << vecu::SafetyManager::state_to_string(
               safety.get_state())
        << '\n';

    scheduler.start();

    /*
     * -------------------------------------------------------
     * Main supervision loop
     * -------------------------------------------------------
     */

    while (running) {

        std::this_thread::sleep_for(1s);

        std::cout
            << "[ECU] Safety state: "
            << vecu::SafetyManager::state_to_string(
                   safety.get_state())
            << '\n';
    }

    /*
     * -------------------------------------------------------
     * Shutdown
     * -------------------------------------------------------
     */

    std::cout
        << "[ECU] Shutting down...\n";

    scheduler.stop();

    can.close();

    std::cout
        << "[ECU] Shutdown complete\n";

    return 0;
}