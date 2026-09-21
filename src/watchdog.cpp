#include "watchdog.hpp"

namespace vecu {

Watchdog::Watchdog(uint32_t timeout_ms)
    : timeout_ms_(timeout_ms)
{
}

void Watchdog::register_task(const std::string& task_name)
{
    std::lock_guard<std::mutex> lock(mutex_);

    tasks_[task_name] = {
        std::chrono::steady_clock::now(),
        true
    };
}

void Watchdog::kick(const std::string& task_name)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = tasks_.find(task_name);

    if (it != tasks_.end()) {
        it->second.last_kick =
            std::chrono::steady_clock::now();
    }
}

bool Watchdog::is_healthy()
{
    std::lock_guard<std::mutex> lock(mutex_);

    const auto now =
        std::chrono::steady_clock::now();

    for (const auto& [name, task] : tasks_) {

        const auto elapsed =
            std::chrono::duration_cast<
                std::chrono::milliseconds>(
                    now - task.last_kick
                ).count();

        if (elapsed > timeout_ms_) {
            failed_task_ = name;
            return false;
        }
    }

    failed_task_.clear();

    return true;
}

std::string Watchdog::get_failed_task() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    return failed_task_;
}

} // namespace vecu