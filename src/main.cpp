#include "can.hpp"
#include "diagnostics.hpp"
#include "safety.hpp"
#include "scheduler.hpp"
#include "signal_manager.hpp"
#include "watchdog.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

namespace {

std::atomic<bool> running{true};

void signal_handler(int signal)
{
    if (signal == SIGINT ||
        signal == SIGTERM)
    {
        running = false;
    }
}

} // namespace

int main()
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout
        << "========================================\n"
        << "       Virtual ECU - VECU\n"
        << "========================================\n";

    /*
     * ----------------------------------------------------
     * Initialize components
     * ----------------------------------------------------
     */

    vecu::CanBus can;
    vecu::SignalManager signal_manager;
    vecu::SafetyManager safety;
    vecu::Diagnostics diagnostics;
    vecu::Watchdog watchdog(500);
    vecu::Scheduler scheduler;

    /*
     * ----------------------------------------------------
     * Open CAN interface
     * ----------------------------------------------------
     */

    if (!can.open("vcan0"))
    {
        std::cerr
            << "[VECU] Failed to open vcan0\n";

        return 1;
    }

    /*
     * ----------------------------------------------------
     * Register watchdog tasks
     * ----------------------------------------------------
     */

    watchdog.register_task("CAN_RX");
    watchdog.register_task("SAFETY");
    watchdog.register_task("DIAGNOSTICS");

    /*
     * ----------------------------------------------------
     * CAN RX task
     *
     * Receives messages from the vehicle simulator.
     * ----------------------------------------------------
     */

    scheduler.add_task(
        "CAN_RX",
        std::chrono::milliseconds(10),
        [&]()
        {
            vecu::CanFrame frame;

            /*
             * We use a zero timeout because the scheduler
             * itself controls when this task executes.
             */
            while (can.receive(frame, 0))
            {
                const bool decoded =
                    signal_manager.process_frame(frame);

                if (!decoded)
                {
                    std::cout
                        << "[CAN] Unknown/invalid frame "
                        << "ID=0x"
                        << std::hex
                        << frame.id
                        << std::dec
                        << " DLC="
                        << static_cast<int>(frame.dlc)
                        << '\n';
                }
            }

            watchdog.kick("CAN_RX");
        });

    /*
     * ----------------------------------------------------
     * Safety task
     * ----------------------------------------------------
     */

    scheduler.add_task(
        "SAFETY",
        std::chrono::milliseconds(50),
        [&]()
        {
            const vecu::VehicleSignals& signals =
                signal_manager.get_signals();

            /*
             * Overspeed detection.
             */
            if (signals.speed_kmh > 130.0F)
            {
                safety.set_fault(
                    vecu::FaultCode::OVERSPEED);

                diagnostics.record_fault(
                    vecu::FaultCode::OVERSPEED);
            }
            else
            {
                safety.clear_fault(
                    vecu::FaultCode::OVERSPEED);

                diagnostics.clear_fault(
                    vecu::FaultCode::OVERSPEED);
            }

            /*
             * Basic signal validation.
             */
            if (signals.rpm > 8000U)
            {
                safety.set_fault(
                    vecu::FaultCode::INVALID_ENGINE_DATA);

                diagnostics.record_fault(
                    vecu::FaultCode::INVALID_ENGINE_DATA);
            }
            else
            {
                safety.clear_fault(
                    vecu::FaultCode::INVALID_ENGINE_DATA);

                diagnostics.clear_fault(
                    vecu::FaultCode::INVALID_ENGINE_DATA);
            }

            /*
             * Update overall safety state.
             */
            safety.update_state();

            watchdog.kick("SAFETY");
        });

    /*
     * ----------------------------------------------------
     * Diagnostics task
     * ----------------------------------------------------
     */

    scheduler.add_task(
        "DIAGNOSTICS",
        std::chrono::milliseconds(100),
        [&]()
        {
            static vecu::SafetyState previous_state =
                vecu::SafetyState::INIT;

            const vecu::SafetyState current_state =
                safety.get_state();

            if (current_state != previous_state)
            {
                std::cout
                    << "[SAFETY] State: "
                    << vecu::SafetyManager::state_to_string(
                           previous_state)
                    << " -> "
                    << vecu::SafetyManager::state_to_string(
                           current_state)
                    << '\n';

                previous_state = current_state;
            }

            const vecu::VehicleSignals& signals =
                signal_manager.get_signals();

            std::cout
                << std::fixed
                << std::setprecision(1)
                << "[VECU] "
                << "Speed="
                << signals.speed_kmh
                << " km/h | "
                << "RPM="
                << signals.rpm
                << " | "
                << "Throttle="
                << static_cast<int>(
                       signals.throttle_percent)
                << "% | "
                << "Brake="
                << static_cast<int>(
                       signals.brake_percent)
                << "% | "
                << "Gear="
                << static_cast<int>(
                       signals.gear)
                << '\n';

            watchdog.kick("DIAGNOSTICS");
        });

    /*
     * ----------------------------------------------------
     * Start scheduler
     * ----------------------------------------------------
     */

    scheduler.start();

    std::cout
        << "[VECU] Scheduler started\n"
        << "[VECU] Waiting for vehicle CAN data...\n"
        << "[VECU] Press Ctrl+C to stop\n";

    /*
     * ----------------------------------------------------
     * Main supervision loop
     * ----------------------------------------------------
     */

    while (running)
    {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(100));

        if (!watchdog.is_healthy())
        {
            const std::string failed_task =
                watchdog.get_failed_task();

            std::cerr
                << "[WATCHDOG] Task timeout: "
                << failed_task
                << '\n';

            safety.set_fault(
                vecu::FaultCode::WATCHDOG_TIMEOUT);

            diagnostics.record_fault(
                vecu::FaultCode::WATCHDOG_TIMEOUT);

            safety.update_state();
        }
    }

    /*
     * ----------------------------------------------------
     * Shutdown
     * ----------------------------------------------------
     */

    std::cout
        << "\n[VECU] Shutting down...\n";

    scheduler.stop();
    can.close();

    std::cout
        << "[VECU] Shutdown complete\n";

    return 0;
}