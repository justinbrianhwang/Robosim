#pragma once

#include <Eigen/Dense>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include <functional>
#include "robosim/core/contact.h"

namespace robosim::reward {

struct RewardContext {
    // Robot base state
    Eigen::Vector3d base_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond base_orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d base_linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d base_angular_velocity = Eigen::Vector3d::Zero();

    // Joint state
    Eigen::VectorXd joint_positions;
    Eigen::VectorXd joint_velocities;
    Eigen::VectorXd joint_torques;

    // Actions
    Eigen::VectorXd previous_actions;
    Eigen::VectorXd current_actions;

    // Target commands
    Eigen::Vector3d target_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond target_orientation = Eigen::Quaterniond::Identity();
    double target_yaw_rate = 0.0;

    // Contacts
    std::vector<core::ContactInfo> contacts;
    Eigen::VectorXd foot_contact_forces;

    // Time
    double simulation_time = 0.0;
    int step_count = 0;
    double dt = 0.002;

    // Terrain
    double terrain_height_at_base = 0.0;
};

struct RewardInfo {
    double total_reward = 0.0;
    std::unordered_map<std::string, double> components;
};

class RewardFunction {
public:
    virtual ~RewardFunction() = default;
    virtual RewardInfo compute(const RewardContext& ctx) const = 0;
    virtual std::string name() const = 0;
};

// Registry
class RewardRegistry {
public:
    static RewardRegistry& instance();
    using Creator = std::function<std::shared_ptr<RewardFunction>()>;
    void register_reward(const std::string& name, Creator creator);
    std::shared_ptr<RewardFunction> create(const std::string& name) const;
    std::vector<std::string> available() const;

private:
    std::unordered_map<std::string, Creator> creators_;
};

#define REGISTER_REWARD(cls, reward_name) \
    namespace { static bool _reg_##cls = [] { \
        RewardRegistry::instance().register_reward(reward_name, \
            []() { return std::make_shared<cls>(); }); \
        return true; \
    }(); }

} // namespace robosim::reward
