#pragma once

#include "robosim/reward/reward_function.h"
#include <vector>

namespace robosim::reward {

class CompositeReward : public RewardFunction {
public:
    void add_term(const std::string& term_name, std::shared_ptr<RewardFunction> func, double weight) {
        terms_.push_back({term_name, std::move(func), weight});
    }

    void set_weight(const std::string& term_name, double weight) {
        for (auto& t : terms_) {
            if (t.name == term_name) { t.weight = weight; return; }
        }
    }

    RewardInfo compute(const RewardContext& ctx) const override {
        RewardInfo info;
        info.total_reward = 0.0;
        for (const auto& t : terms_) {
            auto sub = t.func->compute(ctx);
            double weighted = sub.total_reward * t.weight;
            info.total_reward += weighted;
            info.components[t.name] = weighted;
        }
        return info;
    }

    std::string name() const override { return "composite"; }

private:
    struct WeightedTerm {
        std::string name;
        std::shared_ptr<RewardFunction> func;
        double weight;
    };
    std::vector<WeightedTerm> terms_;
};

} // namespace robosim::reward
