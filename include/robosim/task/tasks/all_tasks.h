#pragma once

#include "robosim/task/task.h"
#include "robosim/reward/rewards/locomotion_rewards.h"

namespace robosim::task {

// ============================================================================
// Walk Forward Task - basic locomotion
// ============================================================================
class WalkForwardTask : public Task {
public:
    WalkForwardTask() {
        target_velocity_ = Eigen::Vector3d(1.0, 0.0, 0.0);
        // Setup default reward
        reward_.add_term("linear_velocity", std::make_shared<reward::LinearVelocityReward>(), 1.0);
        reward_.add_term("orientation", std::make_shared<reward::OrientationReward>(), 0.5);
        reward_.add_term("torque", std::make_shared<reward::TorquePenalty>(), -0.0001);
        reward_.add_term("action_rate", std::make_shared<reward::ActionRatePenalty>(), -0.01);
        reward_.add_term("joint_velocity", std::make_shared<reward::JointVelocityPenalty>(), -0.001);
    }

    std::string name() const override { return "walk_forward"; }

    void reset(std::mt19937& rng, core::World&, core::Robot& robot) override {
        std::uniform_real_distribution<double> vel_dist(0.5, 2.0);
        target_velocity_ = Eigen::Vector3d(vel_dist(rng), 0.0, 0.0);
    }

    Eigen::VectorXd get_task_observation(const core::Robot& robot, const core::World&) const override {
        Eigen::VectorXd obs(3);
        obs = robot.body_frame_rotation().transpose() * target_velocity_;
        return obs;
    }

    int task_obs_dim() const override { return 3; }

    void populate_reward_context(reward::RewardContext& ctx,
                                  const core::Robot& robot,
                                  const core::World&) const override {
        ctx.target_velocity = target_velocity_;
    }

    TerminationResult check_termination(const core::Robot& robot,
                                          const core::World&,
                                          int step_count) const override {
        TerminationResult result;
        if (robot.base_height() < fall_threshold_) {
            result.terminated = true;
            result.reason = "Robot fell";
        }
        Eigen::Vector3d up = robot.base_orientation() * Eigen::Vector3d::UnitZ();
        if (up.z() < 0.3) {
            result.terminated = true;
            result.reason = "Robot tipped over";
        }
        if (step_count >= max_steps_) {
            result.truncated = true;
            result.reason = "Max steps reached";
        }
        return result;
    }

private:
    Eigen::Vector3d target_velocity_;
    double fall_threshold_ = 0.15;
};

// ============================================================================
// Follow Path Task
// ============================================================================
class FollowPathTask : public Task {
public:
    FollowPathTask() {
        reward_.add_term("linear_velocity", std::make_shared<reward::LinearVelocityReward>(), 1.0);
        reward_.add_term("orientation", std::make_shared<reward::OrientationReward>(), 0.3);
        reward_.add_term("torque", std::make_shared<reward::TorquePenalty>(), -0.0001);
    }

    std::string name() const override { return "follow_path"; }

    void reset(std::mt19937& rng, core::World&, core::Robot& robot) override {
        waypoints_.clear();
        std::uniform_real_distribution<double> pos_dist(-3.0, 3.0);
        for (int i = 0; i < 5; i++) {
            waypoints_.push_back(Eigen::Vector3d(pos_dist(rng), pos_dist(rng), 0.0));
        }
        current_wp_ = 0;
    }

    Eigen::VectorXd get_task_observation(const core::Robot& robot, const core::World&) const override {
        Eigen::VectorXd obs(3);
        if (current_wp_ < static_cast<int>(waypoints_.size())) {
            Eigen::Vector3d rel = waypoints_[current_wp_] - robot.base_position();
            obs = robot.body_frame_rotation().transpose() * rel;
        } else {
            obs.setZero();
        }
        return obs;
    }

    int task_obs_dim() const override { return 3; }

    void populate_reward_context(reward::RewardContext& ctx,
                                  const core::Robot& robot,
                                  const core::World&) const override {
        if (current_wp_ < static_cast<int>(waypoints_.size())) {
            Eigen::Vector3d diff = waypoints_[current_wp_] - robot.base_position();
            double dist = diff.norm();
            ctx.target_velocity = (dist > 1e-6) ? (diff / dist) * 1.0 : Eigen::Vector3d::Zero();
            ctx.target_position = waypoints_[current_wp_];
        }
    }

    TerminationResult check_termination(const core::Robot& robot,
                                          const core::World&,
                                          int step_count) const override {
        TerminationResult result;
        if (robot.base_height() < 0.15) {
            result.terminated = true;
            result.reason = "Robot fell";
        }
        if (current_wp_ >= static_cast<int>(waypoints_.size())) {
            result.terminated = true;
            result.reason = "Path completed";
        }
        if (step_count >= max_steps_) {
            result.truncated = true;
            result.reason = "Max steps reached";
        }
        return result;
    }

private:
    std::vector<Eigen::Vector3d> waypoints_;
    int current_wp_ = 0;
};

// ============================================================================
// Reach Target Task (for robot arms)
// ============================================================================
class ReachTargetTask : public Task {
public:
    ReachTargetTask() {
        max_steps_ = 500;
    }

    std::string name() const override { return "reach_target"; }

    void reset(std::mt19937& rng, core::World&, core::Robot&) override {
        std::uniform_real_distribution<double> dist(-0.5, 0.5);
        target_position_ = Eigen::Vector3d(0.4 + dist(rng) * 0.3,
                                             dist(rng) * 0.3,
                                             0.3 + dist(rng) * 0.2);
    }

    Eigen::VectorXd get_task_observation(const core::Robot& robot, const core::World&) const override {
        Eigen::VectorXd obs(3);
        obs = target_position_ - robot.base_position(); // simplified: should use end-effector
        return obs;
    }

    int task_obs_dim() const override { return 3; }

    void populate_reward_context(reward::RewardContext& ctx,
                                  const core::Robot&,
                                  const core::World&) const override {
        ctx.target_position = target_position_;
    }

    TerminationResult check_termination(const core::Robot& robot,
                                          const core::World&,
                                          int step_count) const override {
        TerminationResult result;
        // Check if end-effector is near target
        double dist = (robot.base_position() - target_position_).norm(); // simplified
        if (dist < 0.02) {
            result.terminated = true;
            result.reason = "Target reached";
        }
        if (step_count >= max_steps_) {
            result.truncated = true;
            result.reason = "Max steps reached";
        }
        return result;
    }

private:
    Eigen::Vector3d target_position_;
};

// ============================================================================
// Pick and Place Task
// ============================================================================
class PickAndPlaceTask : public Task {
public:
    PickAndPlaceTask() {
        max_steps_ = 1000;
    }

    std::string name() const override { return "pick_and_place"; }

    void reset(std::mt19937& rng, core::World&, core::Robot&) override {
        std::uniform_real_distribution<double> dist(-0.2, 0.2);
        pick_position_ = Eigen::Vector3d(0.4 + dist(rng), dist(rng), 0.05);
        place_position_ = Eigen::Vector3d(0.4 + dist(rng), dist(rng), 0.2);
        phase_ = Phase::Approach;
    }

    Eigen::VectorXd get_task_observation(const core::Robot& robot, const core::World&) const override {
        Eigen::VectorXd obs(7);
        obs.head<3>() = pick_position_ - robot.base_position();
        obs.segment<3>(3) = place_position_ - robot.base_position();
        obs(6) = static_cast<double>(phase_);
        return obs;
    }

    int task_obs_dim() const override { return 7; }

    void populate_reward_context(reward::RewardContext& ctx,
                                  const core::Robot&,
                                  const core::World&) const override {
        ctx.target_position = (phase_ == Phase::Approach || phase_ == Phase::Grasp)
                              ? pick_position_ : place_position_;
    }

    TerminationResult check_termination(const core::Robot&,
                                          const core::World&,
                                          int step_count) const override {
        TerminationResult result;
        if (phase_ == Phase::Done) {
            result.terminated = true;
            result.reason = "Task completed";
        }
        if (step_count >= max_steps_) {
            result.truncated = true;
            result.reason = "Max steps reached";
        }
        return result;
    }

private:
    enum class Phase { Approach, Grasp, Lift, Place, Done };
    Phase phase_ = Phase::Approach;
    Eigen::Vector3d pick_position_;
    Eigen::Vector3d place_position_;
};

} // namespace robosim::task
