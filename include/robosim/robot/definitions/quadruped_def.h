#pragma once

#include "robosim/core/robot.h"
#include <cmath>

namespace robosim::robot::definitions {

// ============================================================================
// Quadruped Robot Parametric Definition
// ============================================================================
struct QuadrupedParams {
    std::string name;
    std::string description;

    // Body dimensions
    double body_length = 0.4;   // m
    double body_width = 0.2;
    double body_height = 0.1;
    double body_mass = 10.0;    // kg

    // Hip dimensions
    double hip_length = 0.08;
    double hip_radius = 0.04;
    double hip_mass = 0.5;

    // Upper leg (thigh)
    double thigh_length = 0.25;
    double thigh_radius = 0.025;
    double thigh_mass = 1.0;

    // Lower leg (calf)
    double calf_length = 0.25;
    double calf_radius = 0.02;
    double calf_mass = 0.3;

    // Foot
    double foot_radius = 0.02;
    double foot_mass = 0.05;

    // Joint limits
    double hip_abduction_range = 0.8;  // rad
    double hip_flexion_range = 3.14;
    double knee_range = 2.7;

    // Actuator
    double hip_torque = 23.7;  // Nm
    double knee_torque = 35.5;
    double kp = 40.0;
    double kd = 2.0;

    // Spawn
    double nominal_height = 0.35;

    // Visual color RGBA
    Eigen::Vector4d body_color = Eigen::Vector4d(0.2, 0.2, 0.2, 1.0);
    Eigen::Vector4d leg_color = Eigen::Vector4d(0.3, 0.3, 0.3, 1.0);
};

core::RobotDef build_quadruped(const QuadrupedParams& p);

// ============================================================================
// Pre-defined Quadruped Robots
// ============================================================================

// Unitree Go2
inline core::RobotDef create_go2() {
    QuadrupedParams p;
    p.name = "unitree_go2";
    p.description = "Unitree Go2 - Compact quadruped robot for research and education";
    p.body_length = 0.3762;
    p.body_width = 0.0935;
    p.body_height = 0.114;
    p.body_mass = 8.0;
    p.hip_length = 0.0955;
    p.hip_radius = 0.04;
    p.hip_mass = 0.678;
    p.thigh_length = 0.213;
    p.thigh_radius = 0.03;
    p.thigh_mass = 1.152;
    p.calf_length = 0.213;
    p.calf_radius = 0.02;
    p.calf_mass = 0.154;
    p.foot_radius = 0.02;
    p.foot_mass = 0.06;
    p.hip_abduction_range = 0.863;
    p.hip_flexion_range = 3.14;
    p.knee_range = 2.72;
    p.hip_torque = 23.7;
    p.knee_torque = 35.55;
    p.kp = 40.0;
    p.kd = 2.0;
    p.nominal_height = 0.35;
    p.body_color = Eigen::Vector4d(0.15, 0.15, 0.15, 1.0);
    p.leg_color = Eigen::Vector4d(0.2, 0.2, 0.25, 1.0);
    return build_quadruped(p);
}

// Boston Dynamics Spot
inline core::RobotDef create_spot() {
    QuadrupedParams p;
    p.name = "boston_dynamics_spot";
    p.description = "Boston Dynamics Spot - Industrial inspection and exploration robot";
    p.body_length = 0.55;
    p.body_width = 0.17;
    p.body_height = 0.16;
    p.body_mass = 25.0;
    p.hip_length = 0.11;
    p.hip_radius = 0.055;
    p.hip_mass = 1.5;
    p.thigh_length = 0.32;
    p.thigh_radius = 0.04;
    p.thigh_mass = 2.5;
    p.calf_length = 0.32;
    p.calf_radius = 0.03;
    p.calf_mass = 0.8;
    p.foot_radius = 0.025;
    p.foot_mass = 0.1;
    p.hip_abduction_range = 0.8;
    p.hip_flexion_range = 3.14;
    p.knee_range = 2.8;
    p.hip_torque = 45.0;
    p.knee_torque = 60.0;
    p.kp = 80.0;
    p.kd = 4.0;
    p.nominal_height = 0.55;
    p.body_color = Eigen::Vector4d(0.85, 0.75, 0.1, 1.0);
    p.leg_color = Eigen::Vector4d(0.2, 0.2, 0.2, 1.0);
    return build_quadruped(p);
}

// ANYmal C
inline core::RobotDef create_anymal() {
    QuadrupedParams p;
    p.name = "anybotics_anymal_c";
    p.description = "ANYbotics ANYmal C - Industrial inspection quadruped";
    p.body_length = 0.53;
    p.body_width = 0.165;
    p.body_height = 0.15;
    p.body_mass = 30.0;
    p.hip_length = 0.125;
    p.hip_radius = 0.06;
    p.hip_mass = 2.0;
    p.thigh_length = 0.285;
    p.thigh_radius = 0.04;
    p.thigh_mass = 2.8;
    p.calf_length = 0.335;
    p.calf_radius = 0.035;
    p.calf_mass = 1.2;
    p.foot_radius = 0.03;
    p.foot_mass = 0.15;
    p.hip_abduction_range = 0.75;
    p.hip_flexion_range = 3.14;
    p.knee_range = 2.7;
    p.hip_torque = 40.0;
    p.knee_torque = 80.0;
    p.kp = 80.0;
    p.kd = 4.0;
    p.nominal_height = 0.54;
    p.body_color = Eigen::Vector4d(0.9, 0.3, 0.1, 1.0);
    p.leg_color = Eigen::Vector4d(0.3, 0.3, 0.3, 1.0);
    return build_quadruped(p);
}

// Xiaomi CyberDog 2
inline core::RobotDef create_cyberdog() {
    QuadrupedParams p;
    p.name = "xiaomi_cyberdog2";
    p.description = "Xiaomi CyberDog 2 - Consumer quadruped robot";
    p.body_length = 0.37;
    p.body_width = 0.10;
    p.body_height = 0.10;
    p.body_mass = 8.9;
    p.hip_length = 0.08;
    p.hip_radius = 0.035;
    p.hip_mass = 0.6;
    p.thigh_length = 0.20;
    p.thigh_radius = 0.025;
    p.thigh_mass = 1.0;
    p.calf_length = 0.20;
    p.calf_radius = 0.02;
    p.calf_mass = 0.3;
    p.foot_radius = 0.018;
    p.foot_mass = 0.05;
    p.hip_abduction_range = 0.8;
    p.hip_flexion_range = 3.14;
    p.knee_range = 2.5;
    p.hip_torque = 20.0;
    p.knee_torque = 32.0;
    p.kp = 35.0;
    p.kd = 1.8;
    p.nominal_height = 0.33;
    p.body_color = Eigen::Vector4d(0.1, 0.1, 0.1, 1.0);
    p.leg_color = Eigen::Vector4d(0.15, 0.15, 0.15, 1.0);
    return build_quadruped(p);
}

} // namespace robosim::robot::definitions
