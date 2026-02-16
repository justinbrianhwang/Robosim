#pragma once

#include "robosim/reward/reward_function.h"
#include "robosim/reward/composite_reward.h"
#include "robosim/core/robot.h"
#include "robosim/core/world.h"
#include <string>
#include <memory>
#include <random>
#include <functional>
#include <unordered_map>

namespace robosim::task {

struct TerminationResult {
    bool terminated = false;
    bool truncated = false;
    std::string reason;
};

class Task {
public:
    virtual ~Task() = default;

    virtual std::string name() const = 0;
    virtual void reset(std::mt19937& rng, core::World& world, core::Robot& robot) = 0;
    virtual Eigen::VectorXd get_task_observation(const core::Robot& robot,
                                                   const core::World& world) const = 0;
    virtual int task_obs_dim() const = 0;
    virtual void populate_reward_context(reward::RewardContext& ctx,
                                          const core::Robot& robot,
                                          const core::World& world) const = 0;
    virtual TerminationResult check_termination(const core::Robot& robot,
                                                  const core::World& world,
                                                  int step_count) const = 0;
    virtual std::unordered_map<std::string, double> get_info(
        const core::Robot& robot, const core::World& world) const { return {}; }

    reward::CompositeReward& reward_function() { return reward_; }
    const reward::CompositeReward& reward_function() const { return reward_; }
    int max_episode_steps() const { return max_steps_; }
    void set_max_episode_steps(int s) { max_steps_ = s; }

protected:
    reward::CompositeReward reward_;
    int max_steps_ = 1000;
};

class TaskRegistry {
public:
    static TaskRegistry& instance();
    using Creator = std::function<std::unique_ptr<Task>()>;
    void register_task(const std::string& name, Creator creator);
    std::unique_ptr<Task> create(const std::string& name) const;
    std::vector<std::string> available() const;

private:
    std::unordered_map<std::string, Creator> creators_;
};

#define REGISTER_TASK(cls, task_name) \
    namespace { static bool _reg_task_##cls = [] { \
        TaskRegistry::instance().register_task(task_name, \
            []() { return std::make_unique<cls>(); }); \
        return true; \
    }(); }

} // namespace robosim::task
