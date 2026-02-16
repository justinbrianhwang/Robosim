#pragma once

#include <Eigen/Dense>
#include <string>
#include <limits>

namespace robosim::core {

enum class JointType {
    Fixed,
    Revolute,
    Prismatic,
    Continuous,
    Floating,
    Spherical
};

struct JointDef {
    std::string name;
    JointType type = JointType::Revolute;
    std::string parent_link;
    std::string child_link;
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d origin_xyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d origin_rpy = Eigen::Vector3d::Zero();
    double lower_limit = -3.14159;
    double upper_limit = 3.14159;
    double max_velocity = 10.0;
    double max_effort = 100.0;
    double damping = 0.1;
    double friction = 0.0;
};

struct JointState {
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
    double torque = 0.0;
    double target_position = 0.0;
    double target_velocity = 0.0;
};

} // namespace robosim::core
