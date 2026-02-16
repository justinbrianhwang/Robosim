#pragma once

#include <Eigen/Dense>
#include <string>

namespace robosim::core {

struct ContactInfo {
    std::string body_a;
    std::string body_b;
    int link_index_a = -1;
    int link_index_b = -1;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    double depth = 0.0;
    double normal_force = 0.0;
    Eigen::Vector3d friction_force = Eigen::Vector3d::Zero();
};

} // namespace robosim::core
