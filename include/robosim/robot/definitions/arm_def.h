#pragma once

#include "robosim/core/robot.h"

namespace robosim::robot::definitions {

struct ArmParams {
    std::string name;
    std::string description;

    // Number of joints (DOF)
    int dof = 7;

    // Link lengths (from base to end-effector)
    std::vector<double> link_lengths;
    std::vector<double> link_radii;
    std::vector<double> link_masses;

    // Joint types and axes
    std::vector<Eigen::Vector3d> joint_axes;

    // Joint limits
    std::vector<double> joint_lower;
    std::vector<double> joint_upper;
    std::vector<double> max_torques;
    std::vector<double> max_velocities;

    // Base
    double base_height = 0.10;
    double base_radius = 0.08;
    double base_mass = 5.0;

    // End-effector
    double ee_length = 0.10;
    double ee_radius = 0.03;
    double ee_mass = 0.5;
    bool has_gripper = false;
    double gripper_width = 0.08;

    // Actuator
    double kp = 100.0;
    double kd = 10.0;

    // Visual
    Eigen::Vector4d body_color = Eigen::Vector4d(0.9, 0.9, 0.9, 1.0);
    Eigen::Vector4d joint_color = Eigen::Vector4d(0.2, 0.2, 0.2, 1.0);
};

core::RobotDef build_arm(const ArmParams& p);

// Franka Emika Panda
inline core::RobotDef create_panda() {
    ArmParams p;
    p.name = "franka_panda";
    p.description = "Franka Emika Panda - 7-DOF collaborative robot arm for research";
    p.dof = 7;
    p.link_lengths = {0.333, 0.0, 0.316, 0.0825, 0.384, 0.0, 0.107};
    p.link_radii = {0.06, 0.06, 0.05, 0.05, 0.04, 0.04, 0.035};
    p.link_masses = {3.5, 3.5, 2.5, 2.5, 1.8, 1.5, 0.5};
    p.joint_axes = {
        Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitZ()
    };
    p.joint_lower = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    p.joint_upper = { 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973};
    p.max_torques = {87, 87, 87, 87, 12, 12, 12};
    p.max_velocities = {2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61};
    p.base_height = 0.08;
    p.base_radius = 0.10;
    p.base_mass = 8.0;
    p.ee_length = 0.10;
    p.ee_radius = 0.03;
    p.ee_mass = 0.73;
    p.has_gripper = true;
    p.gripper_width = 0.08;
    p.kp = 150.0;
    p.kd = 15.0;
    p.body_color = Eigen::Vector4d(0.95, 0.95, 0.95, 1.0);
    p.joint_color = Eigen::Vector4d(0.15, 0.15, 0.15, 1.0);
    return build_arm(p);
}

// Universal Robots UR5e
inline core::RobotDef create_ur5e() {
    ArmParams p;
    p.name = "universal_robots_ur5e";
    p.description = "Universal Robots UR5e - 6-DOF collaborative industrial robot";
    p.dof = 6;
    p.link_lengths = {0.1625, 0.4250, 0.3922, 0.1333, 0.0997, 0.0996};
    p.link_radii = {0.06, 0.05, 0.045, 0.04, 0.04, 0.035};
    p.link_masses = {3.761, 8.058, 2.846, 1.37, 1.37, 0.365};
    p.joint_axes = {
        Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY()
    };
    p.joint_lower = {-6.2832, -6.2832, -3.1416, -6.2832, -6.2832, -6.2832};
    p.joint_upper = { 6.2832,  6.2832,  3.1416,  6.2832,  6.2832,  6.2832};
    p.max_torques = {150, 150, 150, 28, 28, 28};
    p.max_velocities = {3.14, 3.14, 3.14, 6.28, 6.28, 6.28};
    p.base_height = 0.089;
    p.base_radius = 0.075;
    p.base_mass = 4.0;
    p.kp = 200.0;
    p.kd = 20.0;
    p.body_color = Eigen::Vector4d(0.3, 0.5, 0.8, 1.0);
    p.joint_color = Eigen::Vector4d(0.2, 0.2, 0.2, 1.0);
    return build_arm(p);
}

// KUKA iiwa 14
inline core::RobotDef create_iiwa() {
    ArmParams p;
    p.name = "kuka_iiwa14";
    p.description = "KUKA LBR iiwa 14 R820 - 7-DOF lightweight industrial robot";
    p.dof = 7;
    p.link_lengths = {0.36, 0.0, 0.42, 0.0, 0.40, 0.0, 0.126};
    p.link_radii = {0.07, 0.07, 0.06, 0.06, 0.05, 0.05, 0.04};
    p.link_masses = {5.76, 6.35, 3.5, 3.5, 2.1, 2.1, 0.3};
    p.joint_axes = {
        Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitZ()
    };
    p.joint_lower = {-2.9671, -2.0944, -2.9671, -2.0944, -2.9671, -2.0944, -3.0543};
    p.joint_upper = { 2.9671,  2.0944,  2.9671,  2.0944,  2.9671,  2.0944,  3.0543};
    p.max_torques = {320, 320, 176, 176, 110, 40, 40};
    p.max_velocities = {1.48, 1.48, 1.75, 1.31, 2.27, 2.36, 2.36};
    p.base_height = 0.10;
    p.base_radius = 0.10;
    p.base_mass = 10.0;
    p.kp = 250.0;
    p.kd = 25.0;
    p.body_color = Eigen::Vector4d(0.95, 0.55, 0.1, 1.0);
    p.joint_color = Eigen::Vector4d(0.2, 0.2, 0.2, 1.0);
    return build_arm(p);
}

} // namespace robosim::robot::definitions
