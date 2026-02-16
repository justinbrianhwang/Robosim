#include "robosim/reward/reward_function.h"
#include "robosim/reward/rewards/locomotion_rewards.h"

namespace robosim::reward {

RewardRegistry& RewardRegistry::instance() {
    static RewardRegistry inst;
    return inst;
}

void RewardRegistry::register_reward(const std::string& name, Creator creator) {
    creators_[name] = std::move(creator);
}

std::shared_ptr<RewardFunction> RewardRegistry::create(const std::string& name) const {
    auto it = creators_.find(name);
    if (it == creators_.end()) return nullptr;
    return it->second();
}

std::vector<std::string> RewardRegistry::available() const {
    std::vector<std::string> names;
    for (const auto& [name, _] : creators_) names.push_back(name);
    return names;
}

// Register built-in rewards
namespace {
struct RewardRegistrar {
    RewardRegistrar() {
        auto& r = RewardRegistry::instance();
        r.register_reward("linear_velocity", []() { return std::make_shared<LinearVelocityReward>(); });
        r.register_reward("angular_velocity", []() { return std::make_shared<AngularVelocityReward>(); });
        r.register_reward("orientation", []() { return std::make_shared<OrientationReward>(); });
        r.register_reward("base_height", []() { return std::make_shared<BaseHeightReward>(); });
        r.register_reward("torque_penalty", []() { return std::make_shared<TorquePenalty>(); });
        r.register_reward("action_rate", []() { return std::make_shared<ActionRatePenalty>(); });
        r.register_reward("joint_velocity_penalty", []() { return std::make_shared<JointVelocityPenalty>(); });
        r.register_reward("joint_acceleration_penalty", []() { return std::make_shared<JointAccelerationPenalty>(); });
        r.register_reward("foot_clearance", []() { return std::make_shared<FootClearanceReward>(); });
        r.register_reward("base_velocity_penalty", []() { return std::make_shared<BaseVelocityPenalty>(); });
    }
};
static RewardRegistrar _reward_registrar;
}

} // namespace robosim::reward
