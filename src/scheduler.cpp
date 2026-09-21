#include "scheduler.hpp"

#include <algorithm>
#include <iostream>

namespace vecu {

Scheduler::~Scheduler()
{
    stop();
}

void Scheduler::add_task(
    const std::string& name,
    std::chrono::milliseconds period,
    std::function<void()> callback)
{
    Task task;

    task.name = name;
    task.period = period;
    task.phase = std::chrono::milliseconds(0);
    task.callback = std::move(callback);
    task.enabled = true;

    tasks_.push_back(std::move(task));
}

void Scheduler::start()
{
    if (running_) {
        return;
    }

    running_ = true;

    const auto now =
        std::chrono::steady_clock::now();

    for (auto& task : tasks_) {
        task.next_release = now + task.period;
    }

    worker_ = std::thread(&Scheduler::run, this);
}

void Scheduler::stop()
{
    running_ = false;

    if (worker_.joinable()) {
        worker_.join();
    }
}

bool Scheduler::is_running() const
{
    return running_;
}

void Scheduler::run()
{
    while (running_) {

        const auto now =
            std::chrono::steady_clock::now();

        auto next_wakeup =
            now + std::chrono::milliseconds(100);

        for (auto& task : tasks_) {

            if (!task.enabled) {
                continue;
            }

            if (now >= task.next_release) {

                try {
                    task.callback();
                }
                catch (const std::exception& e) {
                    std::cerr
                        << "[SCHEDULER] Task "
                        << task.name
                        << " exception: "
                        << e.what()
                        << '\n';
                }
                catch (...) {
                    std::cerr
                        << "[SCHEDULER] Task "
                        << task.name
                        << " unknown exception\n";
                }

                /*
                 * Deadline scheduling rather than:
                 *
                 * next_release = now + period
                 *
                 * This avoids accumulating task execution
                 * time into the task period.
                 */

                task.next_release += task.period;

                /*
                 * If a task was delayed for multiple periods,
                 * don't execute a burst of old activations.
                 */

                if (task.next_release <= now) {
                    task.next_release =
                        now + task.period;
                }
            }

            next_wakeup =
                std::min(next_wakeup,
                         task.next_release);
        }

        std::this_thread::sleep_until(next_wakeup);
    }
}

} // namespace vecu