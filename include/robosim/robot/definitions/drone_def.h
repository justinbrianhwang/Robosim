#pragma once

#include "robosim/core/robot.h"

namespace robosim::robot::definitions {

struct DroneParams {
    std::string name;
    std::string description;

    int num_rotors = 4;  // 4 or 6
    double arm_length = 0.25;  // center to motor
    double arm_radius = 0.01;
    double arm_mass = 0.05;

    // Body
    double body_radius = 0.10;
    double body_height = 0.05;
    double body_mass = 1.0;

    // Rotor
    double rotor_radius = 0.12;
    double rotor_height = 0.01;
    double rotor_mass = 0.02;
    double max_thrust = 10.0; // N per motor
    double max_torque_z = 0.1; // Nm per motor (yaw torque from spinning)
    double thrust_coefficient = 1.0e-5;
    double torque_coefficient = 1.0e-7;

    // Spawn
    double nominal_height = 1.0;

    // Visual
    Eigen::Vector4d body_color = Eigen::Vector4d(0.2, 0.2, 0.2, 1.0);
    Eigen::Vector4d arm_color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0);
    Eigen::Vector4d rotor_color = Eigen::Vector4d(0.1, 0.8, 0.2, 1.0);
};

core::RobotDef build_drone(const DroneParams& p);

// Generic Quadrotor
inline core::RobotDef create_quadrotor() {
    DroneParams p;
    p.name = "generic_quadrotor";
    p.description = "Generic Quadrotor - Standard X-configuration research drone";
    p.num_rotors = 4;
    p.arm_length = 0.22;
    p.body_radius = 0.08;
    p.body_height = 0.04;
    p.body_mass = 0.8;
    p.rotor_radius = 0.10;
    p.max_thrust = 6.0;
    p.nominal_height = 1.0;
    p.body_color = Eigen::Vector4d(0.1, 0.1, 0.1, 1.0);
    p.arm_color = Eigen::Vector4d(0.4, 0.4, 0.4, 1.0);
    p.rotor_color = Eigen::Vector4d(0.2, 0.8, 0.3, 1.0);
    return build_drone(p);
}

// DJI Matrice-like Hexarotor
inline core::RobotDef create_hexarotor() {
    DroneParams p;
    p.name = "dji_hexarotor";
    p.description = "DJI Matrice-like Hexarotor - Heavy-lift industrial drone";
    p.num_rotors = 6;
    p.arm_length = 0.35;
    p.arm_radius = 0.015;
    p.arm_mass = 0.08;
    p.body_radius = 0.15;
    p.body_height = 0.07;
    p.body_mass = 3.5;
    p.rotor_radius = 0.16;
    p.rotor_mass = 0.05;
    p.max_thrust = 15.0;
    p.nominal_height = 1.5;
    p.body_color = Eigen::Vector4d(0.2, 0.2, 0.2, 1.0);
    p.arm_color = Eigen::Vector4d(0.3, 0.3, 0.3, 1.0);
    p.rotor_color = Eigen::Vector4d(0.8, 0.2, 0.1, 1.0);
    return build_drone(p);
}

} // namespace robosim::robot::definitions
