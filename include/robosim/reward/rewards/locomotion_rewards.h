#pragma once

#include "robosim/reward/reward_function.h"
#include <cmath>

namespace robosim::reward {

// Rewards forward velocity tracking (exp(-error^2 / sigma^2))
class LinearVelocityReward : public RewardFunction {
public:
    RewardInfo compute(const RewardContext& ctx) const override {
        double error = (ctx.base_linear_velocity - ctx.target_velocity).squaredNorm();
        double r = std::exp(-error / sigma_sq_);
        return {r, {{"linear_velocity", r}}};
    }
    std::string name() const override { return "linear_velocity"; }
    void set_sigma(double s) { sigma_sq_ = s * s; }
private:
    double sigma_sq_ = 0.25 * 0.25;
};

// Rewards angular velocity tracking (yaw rate)
class AngularVelocityReward : public RewardFunction {
public:
    RewardInfo compute(const RewardContext& ctx) const override {
        double error = std::pow(ctx.base_angular_velocity.z() - ctx.target_yaw_rate, 2);
        double r = std::exp(-error / sigma_sq_);
        return {r, {{"angular_velocity", r}}};
    }
    std::string name() const override { return "angular_velocity"; }
private:
    double sigma_sq_ = 0.25 * 0.25;
};

// Rewards upright orientation
class OrientationReward : public RewardFunction {
public:
    RewardInfo compute(const RewardContext& ctx) const override {
        Eigen::Vector3d up = ctx.base_orientation * Eigen::Vector3d::UnitZ();
        double r = up.z(); // 1.0 when perfectly upright, <0 when upside down
        return {std::max(0.0, r), {{"orientation", std::max(0.0, r)}}};
    }
    std::string name() const override { return "orientation"; }
};

// Penalizes base height deviation from nominal
class BaseHeightReward : public RewardFunction {
public:
    RewardInfo compute(const RewardContext& ctx) const override {
        double error = std::pow(ctx.base_position.z() - ctx.terrain_height_at_base - target_height_, 2);
        double r = std::exp(-error / (0.05 * 0.05));
        return {r, {{"base_height", r}}};
    }
    std::string name() const override { return "base_height"; }
    void set_target_height(double h) { target_height_ = h; }
private:
    double target_height_ = 0.35;
};

// Penalizes torque (energy efficiency)
class TorquePenalty : public RewardFunction {
public:
    RewardInfo compute(const RewardContext& ctx) const override {
        double penalty = ctx.joint_torques.squaredNorm();
        return {penalty, {{"torque_penalty", penalty}}};
    }
    std::string name() const override { return "torque_penalty"; }
};

// Penalizes action rate (smooth motion)
class ActionRatePenalty : public RewardFunction {
public:
    RewardInfo compute(const RewardContext& ctx) const override {
        if (ctx.previous_actions.size() != ctx.current_actions.size()) return {0, {}};
        double penalty = (ctx.current_actions - ctx.previous_actions).squaredNorm();
        return {penalty, {{"action_rate", penalty}}};
    }
    std::string name() const override { return "action_rate"; }
};

// Penalizes joint velocities
class JointVelocityPenalty : public RewardFunction {
public:
    RewardInfo compute(const RewardContext& ctx) const override {
        double penalty = ctx.joint_velocities.squaredNorm();
        return {penalty, {{"joint_velocity", penalty}}};
    }
    std::string name() const override { return "joint_velocity_penalty"; }
};

// Penalizes joint acceleration (jerk-like)
class JointAccelerationPenalty : public RewardFunction {
public:
    RewardInfo compute(const RewardContext& ctx) const override {
        // Approximate acceleration from velocity change / dt
        double penalty = 0.0;
        if (ctx.dt > 0.0 && ctx.joint_velocities.size() > 0) {
            penalty = ctx.joint_velocities.squaredNorm();
        }
        return {penalty, {{"joint_acceleration", penalty}}};
    }
    std::string name() const override { return "joint_acceleration_penalty"; }
};

// Rewards foot clearance during swing phase
class FootClearanceReward : public RewardFunction {
public:
    RewardInfo compute(const RewardContext& ctx) const override {
        // Reward foot contact forces being binary (either fully on ground or in air)
        double r = 0.0;
        if (ctx.foot_contact_forces.size() > 0) {
            for (int i = 0; i < ctx.foot_contact_forces.size(); i++) {
                double f = ctx.foot_contact_forces(i);
                // Encourage either full contact or no contact
                r += std::exp(-std::pow(f - 0.0, 2) / 100.0) +
                     std::exp(-std::pow(f - 100.0, 2) / 100.0);
            }
            r /= ctx.foot_contact_forces.size();
        }
        return {r, {{"foot_clearance", r}}};
    }
    std::string name() const override { return "foot_clearance"; }
};

// Penalizes lateral/unwanted base velocity
class BaseVelocityPenalty : public RewardFunction {
public:
    RewardInfo compute(const RewardContext& ctx) const override {
        // Penalize z-velocity (bouncing)
        double penalty = std::pow(ctx.base_linear_velocity.z(), 2);
        return {penalty, {{"base_velocity_z", penalty}}};
    }
    std::string name() const override { return "base_velocity_penalty"; }
};

} // namespace robosim::reward
