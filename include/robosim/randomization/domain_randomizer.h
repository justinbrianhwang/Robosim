#pragma once

#include "robosim/core/world.h"
#include "robosim/core/robot.h"
#include <random>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>

namespace robosim::randomization {

struct ParameterDistribution {
    std::string name;
    enum class Type { Uniform, Normal, LogUniform } type = Type::Uniform;
    double low = 0.0, high = 1.0;
    double mean = 0.0, std = 1.0;
    double default_value = 1.0;

    double sample(std::mt19937& rng) const {
        switch (type) {
        case Type::Uniform: {
            std::uniform_real_distribution<double> d(low, high);
            return d(rng);
        }
        case Type::Normal: {
            std::normal_distribution<double> d(mean, std);
            return d(rng);
        }
        case Type::LogUniform: {
            std::uniform_real_distribution<double> d(std::log(low), std::log(high));
            return std::exp(d(rng));
        }
        }
        return default_value;
    }
};

struct RandomizationSample {
    std::unordered_map<std::string, double> params;
    unsigned int seed = 0;
};

class DomainRandomizer {
public:
    DomainRandomizer() = default;

    void add_parameter(const ParameterDistribution& dist) { distributions_.push_back(dist); }

    RandomizationSample sample(std::mt19937& rng) const {
        RandomizationSample s;
        for (const auto& d : distributions_) {
            s.params[d.name] = d.sample(rng);
        }
        return s;
    }

    void apply(const RandomizationSample& sample, core::World& world, core::Robot& robot) const {
        for (const auto& [name, value] : sample.params) {
            auto it = apply_funcs_.find(name);
            if (it != apply_funcs_.end()) {
                it->second(value, world, robot);
            }
        }
    }

    // Register how to apply a named parameter
    using ApplyFunc = std::function<void(double, core::World&, core::Robot&)>;
    void register_apply(const std::string& name, ApplyFunc func) {
        apply_funcs_[name] = std::move(func);
    }

    // Default randomization setup for locomotion
    void setup_locomotion_defaults() {
        add_parameter({"ground_friction", ParameterDistribution::Type::Uniform, 0.3, 2.0, 0, 0, 1.0});
        register_apply("ground_friction", [](double v, core::World& w, core::Robot&) {
            w.set_ground_friction(v);
        });

        add_parameter({"gravity_z", ParameterDistribution::Type::Uniform, -10.5, -9.0, 0, 0, -9.81});
        register_apply("gravity_z", [](double v, core::World& w, core::Robot&) {
            w.set_gravity(Eigen::Vector3d(0, 0, v));
        });
    }

    const std::vector<ParameterDistribution>& distributions() const { return distributions_; }

private:
    std::vector<ParameterDistribution> distributions_;
    std::unordered_map<std::string, ApplyFunc> apply_funcs_;
};

} // namespace robosim::randomization
