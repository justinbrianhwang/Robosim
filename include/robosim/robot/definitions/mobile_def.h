#pragma once

#include "robosim/core/robot.h"

namespace robosim::robot::definitions {

struct MobileRobotParams {
    std::string name;
    std::string description;

    enum class DriveType { Differential, Ackermann };
    DriveType drive_type = DriveType::Differential;

    // Body
    double body_length = 0.50;
    double body_width = 0.40;
    double body_height = 0.15;
    double body_mass = 15.0;

    // Wheels
    double wheel_radius = 0.08;
    double wheel_width = 0.04;
    double wheel_mass = 0.5;
    int num_wheels = 4; // 2 drive + 2 caster for differential, 4 for ackermann

    // Drive params
    double max_wheel_torque = 10.0;
    double max_wheel_velocity = 20.0; // rad/s
    double wheelbase = 0.40; // distance between front and rear axle (ackermann)
    double track_width = 0.35; // distance between left and right wheels
    double max_steering_angle = 0.52; // rad (ackermann only)

    // Caster (differential only)
    double caster_radius = 0.03;
    double caster_mass = 0.1;

    // Actuator
    double kp = 10.0;
    double kd = 0.5;

    double nominal_height = 0.20;

    // Visual
    Eigen::Vector4d body_color = Eigen::Vector4d(0.3, 0.3, 0.7, 1.0);
    Eigen::Vector4d wheel_color = Eigen::Vector4d(0.1, 0.1, 0.1, 1.0);
};

core::RobotDef build_mobile_robot(const MobileRobotParams& p);

// Differential Drive Robot (TurtleBot-like)
inline core::RobotDef create_differential_drive() {
    MobileRobotParams p;
    p.name = "differential_drive";
    p.description = "Differential Drive Robot - TurtleBot-style mobile platform for SLAM and navigation";
    p.drive_type = MobileRobotParams::DriveType::Differential;
    p.body_length = 0.35;
    p.body_width = 0.35;
    p.body_height = 0.12;
    p.body_mass = 6.0;
    p.wheel_radius = 0.066;
    p.wheel_width = 0.03;
    p.wheel_mass = 0.3;
    p.num_wheels = 2;
    p.max_wheel_torque = 5.0;
    p.max_wheel_velocity = 30.0;
    p.track_width = 0.30;
    p.kp = 8.0;
    p.kd = 0.3;
    p.nominal_height = 0.15;
    p.body_color = Eigen::Vector4d(0.2, 0.6, 0.3, 1.0);
    p.wheel_color = Eigen::Vector4d(0.1, 0.1, 0.1, 1.0);
    return build_mobile_robot(p);
}

// Ackermann Steering Robot
inline core::RobotDef create_ackermann_robot() {
    MobileRobotParams p;
    p.name = "ackermann_robot";
    p.description = "Ackermann Steering Robot - Car-like mobile platform for autonomous driving";
    p.drive_type = MobileRobotParams::DriveType::Ackermann;
    p.body_length = 0.60;
    p.body_width = 0.40;
    p.body_height = 0.15;
    p.body_mass = 12.0;
    p.wheel_radius = 0.08;
    p.wheel_width = 0.05;
    p.wheel_mass = 0.5;
    p.num_wheels = 4;
    p.max_wheel_torque = 15.0;
    p.max_wheel_velocity = 25.0;
    p.wheelbase = 0.45;
    p.track_width = 0.35;
    p.max_steering_angle = 0.52;
    p.kp = 12.0;
    p.kd = 0.5;
    p.nominal_height = 0.18;
    p.body_color = Eigen::Vector4d(0.7, 0.2, 0.2, 1.0);
    p.wheel_color = Eigen::Vector4d(0.1, 0.1, 0.1, 1.0);
    return build_mobile_robot(p);
}

} // namespace robosim::robot::definitions
