#pragma once

#include "robosim/core/joint.h"
#include "robosim/core/link.h"
#include "robosim/core/actuator.h"
#include "robosim/core/contact.h"
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

namespace robosim::core {

enum class RobotCategory {
    Quadruped,
    Humanoid,
    Arm,
    Drone,
    MobileRobot
};

struct RobotDef {
    std::string name;
    std::string description;
    RobotCategory category;
    std::vector<LinkDef> links;
    std::vector<JointDef> joints;
    std::vector<ActuatorParams> actuators; // one per movable joint

    // Default spawn state
    Eigen::Vector3d spawn_position = Eigen::Vector3d(0, 0, 0.5);
    Eigen::Quaterniond spawn_orientation = Eigen::Quaterniond::Identity();
    std::vector<double> default_joint_positions;

    // Physical properties
    double total_mass = 0.0;
    double nominal_height = 0.5; // nominal standing height
};

class Robot {
public:
    Robot() = default;
    explicit Robot(const RobotDef& def);

    // State access
    const std::string& name() const { return def_.name; }
    RobotCategory category() const { return def_.category; }
    int num_joints() const { return static_cast<int>(joint_states_.size()); }
    int num_links() const { return static_cast<int>(def_.links.size()); }

    // Base state
    Eigen::Vector3d base_position() const { return base_position_; }
    Eigen::Quaterniond base_orientation() const { return base_orientation_; }
    Eigen::Vector3d base_linear_velocity() const { return base_linear_vel_; }
    Eigen::Vector3d base_angular_velocity() const { return base_angular_vel_; }
    double base_height() const { return base_position_.z(); }

    Eigen::Matrix3d body_frame_rotation() const { return base_orientation_.toRotationMatrix(); }

    void set_base_state(const Eigen::Vector3d& pos, const Eigen::Quaterniond& ori,
                        const Eigen::Vector3d& lin_vel, const Eigen::Vector3d& ang_vel);

    // Joint state
    const JointState& joint_state(int idx) const { return joint_states_[idx]; }
    JointState& joint_state(int idx) { joint_cache_dirty_ = true; return joint_states_[idx]; }

    const Eigen::VectorXd& joint_positions() const;
    const Eigen::VectorXd& joint_velocities() const;
    const Eigen::VectorXd& joint_torques() const;
    void update_joint_caches() const;

    void set_joint_positions(const Eigen::VectorXd& q);
    void set_joint_velocities(const Eigen::VectorXd& qd);

    // Control
    void set_target_positions(const Eigen::VectorXd& targets);
    void set_target_velocities(const Eigen::VectorXd& targets);
    Eigen::VectorXd compute_torques(double dt);
    void apply_action(const Eigen::VectorXd& action, double dt);

    // Reset
    void reset();
    void reset_to(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel);

    // Contacts
    void set_contacts(const std::vector<ContactInfo>& contacts) { contacts_ = contacts; }
    const std::vector<ContactInfo>& contacts() const { return contacts_; }
    Eigen::VectorXd foot_contact_forces() const;

    // Definition access
    const RobotDef& definition() const { return def_; }

    // Joint name lookup
    int joint_index(const std::string& name) const;

private:
    RobotDef def_;
    std::vector<JointState> joint_states_;
    std::vector<Actuator> actuators_;
    std::vector<ContactInfo> contacts_;

    Eigen::Vector3d base_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond base_orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d base_linear_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d base_angular_vel_ = Eigen::Vector3d::Zero();

    std::unordered_map<std::string, int> joint_name_map_;

    // Cached joint state vectors (mutable for lazy update from const methods)
    mutable Eigen::VectorXd cached_positions_;
    mutable Eigen::VectorXd cached_velocities_;
    mutable Eigen::VectorXd cached_torques_;
    mutable bool joint_cache_dirty_ = true;
};

} // namespace robosim::core
