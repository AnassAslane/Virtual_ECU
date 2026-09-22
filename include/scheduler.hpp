#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace vecu {

struct Task {
    std::string name;

    std::chrono::milliseconds period;

    std::chrono::milliseconds phase;

    std::function<void()> callback;

    std::chrono::steady_clock::time_point next_release;

    bool enabled{true};
};

class Scheduler {
public:
    Scheduler() = default;
    ~Scheduler();

    void add_task(
        const std::string& name,
        std::chrono::milliseconds period,
        std::function<void()> callback);

    void start();

    void stop();

    bool is_running() const;

private:
    void run();

    std::vector<Task> tasks_;

    std::thread worker_;

    std::atomic<bool> running_{false};
};

} // namespace vecu