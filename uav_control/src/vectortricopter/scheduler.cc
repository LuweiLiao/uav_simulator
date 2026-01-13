#include <vectortricopter/scheduler.hpp>
#include <vectortricopter/world.hpp>

#include <glog/logging.h>

namespace tilt {
namespace tricopter {

Scheduler *Scheduler::globalInstance = nullptr;

Scheduler& Scheduler::GetInstance() {
    if (globalInstance == nullptr) {
        globalInstance = new Scheduler();
    }

    CHECK(globalInstance != nullptr);

    return *globalInstance;
}

void Scheduler::Init(const std::vector<Task>& tasks) {
    std::unique_lock<std::mutex> lock(_mutex);
    CHECK(!_initialized);
    _tasks = tasks;
    ScheduleTasks();
    _initialized = true;
}

void Scheduler::ScheduleTasks() {
    _main_loop = std::thread(
        std::bind(&Scheduler::MainLoop, this)
    );
}

void Scheduler::MainLoop() {
    while (1) {
        const size_t ntasks = _tasks.size();

        const double schedule_start_time = GetCurrentTimeSeconds();
        double now = schedule_start_time;
        for (size_t n = 0; n < ntasks; ++n) {
            Task& task = _tasks[n];
            
            bool should_schedule = !task.hasScheduled || task.updateRateHZ == 0;

            now = GetCurrentTimeSeconds();

            if (!should_schedule) {
                const double dt = now - task.lastScheduledTime;
                should_schedule = dt >= 1. / (double) task.updateRateHZ;
            }

            if (should_schedule) {
                task.hasScheduled = true;
                task.lastScheduledTime = now;
                task.func();
            }
        }
    }
}

}
}