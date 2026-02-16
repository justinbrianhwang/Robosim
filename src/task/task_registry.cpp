#include "robosim/task/task.h"
#include "robosim/task/tasks/all_tasks.h"

namespace robosim::task {

TaskRegistry& TaskRegistry::instance() {
    static TaskRegistry inst;
    return inst;
}

void TaskRegistry::register_task(const std::string& name, Creator creator) {
    creators_[name] = std::move(creator);
}

std::unique_ptr<Task> TaskRegistry::create(const std::string& name) const {
    auto it = creators_.find(name);
    if (it == creators_.end()) return nullptr;
    return it->second();
}

std::vector<std::string> TaskRegistry::available() const {
    std::vector<std::string> names;
    for (const auto& [name, _] : creators_) names.push_back(name);
    return names;
}

// Register built-in tasks
namespace {
struct TaskRegistrar {
    TaskRegistrar() {
        auto& r = TaskRegistry::instance();
        r.register_task("walk_forward", []() { return std::make_unique<WalkForwardTask>(); });
        r.register_task("follow_path", []() { return std::make_unique<FollowPathTask>(); });
        r.register_task("reach_target", []() { return std::make_unique<ReachTargetTask>(); });
        r.register_task("pick_and_place", []() { return std::make_unique<PickAndPlaceTask>(); });
    }
};
static TaskRegistrar _task_registrar;
}

} // namespace robosim::task
