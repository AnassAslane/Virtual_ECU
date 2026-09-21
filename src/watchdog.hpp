#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>

namespace vecu {

class Watchdog {
public:
    explicit Watchdog(uint32_t timeout_ms);

    void register_task(const std::string& task_name);

    void kick(const std::string& task_name);

    bool is_healthy();

    std::string get_failed_task() const;

private:
    struct TaskState {
        std::chrono::steady_clock::time_point last_kick;
        bool registered{false};
    };

    uint32_t timeout_ms_;

    mutable std::mutex mutex_;

    std::map<std::string, TaskState> tasks_;

    std::string failed_task_;
};

} // namespace vecu