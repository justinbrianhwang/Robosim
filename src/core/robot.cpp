#include "robosim/core/robot.h"
#include <spdlog/spdlog.h>

namespace robosim::core {

Robot::Robot(const RobotDef& def) : def_(def) {
    // Initialize joint states
    int num_movable = 0;
    for (const auto& j : def_.joints) {
        if (j.type != JointType::Fixed) {
            joint_name_map_[j.name] = num_movable;
            num_movable++;
        }
    }
    joint_states_.resize(num_movable);

    // Initialize actuators
    actuators_.reserve(def_.actuators.size());
    for (const auto& ap : def_.actuators) {
        actuators_.emplace_back(ap);
    }
    // Pad if needed
    while (static_cast<int>(actuators_.size()) < num_movable) {
        actuators_.emplace_back(ActuatorParams{});
    }

    // Set default joint positions
    if (!def_.default_joint_positions.empty()) {
        for (size_t i = 0; i < def_.default_joint_positions.size() && i < joint_states_.size(); i++) {
            joint_states_[i].position = def_.default_joint_positions[i];
            joint_states_[i].target_position = def_.default_joint_positions[i];
        }
    }

    base_position_ = def_.spawn_position;
    base_orientation_ = def_.spawn_orientation;

    spdlog::info("Robot '{}' created with {} joints, {} links",
                 def_.name, num_movable, def_.links.size());
}

void Robot::set_base_state(const Eigen::Vector3d& pos, const Eigen::Quaterniond& ori,
                            const Eigen::Vector3d& lin_vel, const Eigen::Vector3d& ang_vel) {
    base_position_ = pos;
    base_orientation_ = ori;
    base_linear_vel_ = lin_vel;
    base_angular_vel_ = ang_vel;
}

void Robot::update_joint_caches() const {
    if (!joint_cache_dirty_) return;
    int n = num_joints();
    if (cached_positions_.size() != n) {
        cached_positions_.resize(n);
        cached_velocities_.resize(n);
        cached_torques_.resize(n);
    }
    for (int i = 0; i < n; i++) {
        cached_positions_(i) = joint_states_[i].position;
        cached_velocities_(i) = joint_states_[i].velocity;
        cached_torques_(i) = joint_states_[i].torque;
    }
    joint_cache_dirty_ = false;
}

const Eigen::VectorXd& Robot::joint_positions() const {
    update_joint_caches();
    return cached_positions_;
}

const Eigen::VectorXd& Robot::joint_velocities() const {
    update_joint_caches();
    return cached_velocities_;
}

const Eigen::VectorXd& Robot::joint_torques() const {
    update_joint_caches();
    return cached_torques_;
}

void Robot::set_joint_positions(const Eigen::VectorXd& q) {
    for (int i = 0; i < std::min(num_joints(), static_cast<int>(q.size())); i++) {
        joint_states_[i].position = q(i);
    }
    joint_cache_dirty_ = true;
}

void Robot::set_joint_velocities(const Eigen::VectorXd& qd) {
    for (int i = 0; i < std::min(num_joints(), static_cast<int>(qd.size())); i++) {
        joint_states_[i].velocity = qd(i);
    }
    joint_cache_dirty_ = true;
}

void Robot::set_target_positions(const Eigen::VectorXd& targets) {
    for (int i = 0; i < std::min(num_joints(), static_cast<int>(targets.size())); i++) {
        joint_states_[i].target_position = targets(i);
    }
}

void Robot::set_target_velocities(const Eigen::VectorXd& targets) {
    for (int i = 0; i < std::min(num_joints(), static_cast<int>(targets.size())); i++) {
        joint_states_[i].target_velocity = targets(i);
    }
}

Eigen::VectorXd Robot::compute_torques(double dt) {
    int n = num_joints();
    Eigen::VectorXd torques(n);
    for (int i = 0; i < n; i++) {
        auto& s = joint_states_[i];
        torques(i) = actuators_[i].compute_torque(
            s.position, s.velocity, s.target_position, s.target_velocity, dt);
        s.torque = torques(i);
    }
    joint_cache_dirty_ = true;
    return torques;
}

void Robot::apply_action(const Eigen::VectorXd& action, double dt) {
    // Actions are interpreted as target joint positions (normalized [-1,1] to joint limits)
    int n = std::min(num_joints(), static_cast<int>(action.size()));
    int joint_idx = 0;
    for (const auto& jd : def_.joints) {
        if (jd.type == JointType::Fixed) continue;
        if (joint_idx >= n) break;
        // Map [-1, 1] to [lower_limit, upper_limit]
        double range = (jd.upper_limit - jd.lower_limit) * 0.5;
        double mid = (jd.upper_limit + jd.lower_limit) * 0.5;
        joint_states_[joint_idx].target_position = mid + action(joint_idx) * range;
        joint_idx++;
    }
    compute_torques(dt);
}

void Robot::reset() {
    base_position_ = def_.spawn_position;
    base_orientation_ = def_.spawn_orientation;
    base_linear_vel_.setZero();
    base_angular_vel_.setZero();

    for (int i = 0; i < num_joints(); i++) {
        joint_states_[i] = JointState{};
        actuators_[i].reset();
    }
    if (!def_.default_joint_positions.empty()) {
        for (size_t i = 0; i < def_.default_joint_positions.size() && i < joint_states_.size(); i++) {
            joint_states_[i].position = def_.default_joint_positions[i];
            joint_states_[i].target_position = def_.default_joint_positions[i];
        }
    }
    contacts_.clear();
    joint_cache_dirty_ = true;
}

void Robot::reset_to(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel) {
    set_joint_positions(qpos);
    set_joint_velocities(qvel);
    for (auto& a : actuators_) a.reset();
}

Eigen::VectorXd Robot::foot_contact_forces() const {
    // Aggregate contact forces by link
    Eigen::VectorXd forces(contacts_.size());
    for (size_t i = 0; i < contacts_.size(); i++) {
        forces(i) = contacts_[i].normal_force;
    }
    return forces;
}

int Robot::joint_index(const std::string& name) const {
    auto it = joint_name_map_.find(name);
    return (it != joint_name_map_.end()) ? it->second : -1;
}

} // namespace robosim::core
