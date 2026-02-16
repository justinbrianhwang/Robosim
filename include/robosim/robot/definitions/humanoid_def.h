#pragma once

#include "robosim/core/robot.h"

namespace robosim::robot::definitions {

struct HumanoidParams {
    std::string name;
    std::string description;

    // Torso
    double torso_width = 0.3;
    double torso_depth = 0.2;
    double torso_height = 0.4;
    double torso_mass = 20.0;

    // Pelvis
    double pelvis_width = 0.25;
    double pelvis_depth = 0.18;
    double pelvis_height = 0.15;
    double pelvis_mass = 8.0;

    // Upper leg
    double upper_leg_length = 0.40;
    double upper_leg_radius = 0.06;
    double upper_leg_mass = 5.0;

    // Lower leg
    double lower_leg_length = 0.40;
    double lower_leg_radius = 0.04;
    double lower_leg_mass = 3.0;

    // Foot
    double foot_length = 0.22;
    double foot_width = 0.10;
    double foot_height = 0.05;
    double foot_mass = 0.8;

    // Upper arm
    double upper_arm_length = 0.30;
    double upper_arm_radius = 0.04;
    double upper_arm_mass = 2.5;

    // Lower arm
    double lower_arm_length = 0.28;
    double lower_arm_radius = 0.035;
    double lower_arm_mass = 1.5;

    // Hand
    double hand_length = 0.10;
    double hand_width = 0.08;
    double hand_height = 0.03;
    double hand_mass = 0.4;

    // Head
    double head_radius = 0.10;
    double head_mass = 3.0;

    // Joint ranges
    double hip_flexion_range = 2.5;
    double hip_abduction_range = 0.8;
    double hip_rotation_range = 0.8;
    double knee_range = 2.5;
    double ankle_range = 1.0;
    double shoulder_flexion_range = 3.14;
    double shoulder_abduction_range = 2.0;
    double elbow_range = 2.5;

    // Actuator
    double leg_torque = 150.0;
    double arm_torque = 50.0;
    double kp = 200.0;
    double kd = 10.0;

    // Spawn
    double nominal_height = 1.0;

    // Visual
    Eigen::Vector4d body_color = Eigen::Vector4d(0.3, 0.3, 0.35, 1.0);
    Eigen::Vector4d limb_color = Eigen::Vector4d(0.25, 0.25, 0.3, 1.0);

    bool has_arms = true;
    bool has_hands = false;
};

core::RobotDef build_humanoid(const HumanoidParams& p);

// Boston Dynamics Atlas
inline core::RobotDef create_atlas() {
    HumanoidParams p;
    p.name = "boston_dynamics_atlas";
    p.description = "Boston Dynamics Atlas - Advanced bipedal humanoid for dynamic locomotion";
    p.torso_width = 0.40;
    p.torso_depth = 0.25;
    p.torso_height = 0.50;
    p.torso_mass = 35.0;
    p.pelvis_width = 0.30;
    p.pelvis_depth = 0.20;
    p.pelvis_height = 0.18;
    p.pelvis_mass = 12.0;
    p.upper_leg_length = 0.45;
    p.upper_leg_radius = 0.07;
    p.upper_leg_mass = 8.0;
    p.lower_leg_length = 0.42;
    p.lower_leg_radius = 0.055;
    p.lower_leg_mass = 5.0;
    p.foot_length = 0.26;
    p.foot_width = 0.14;
    p.foot_height = 0.06;
    p.foot_mass = 1.5;
    p.upper_arm_length = 0.36;
    p.upper_arm_radius = 0.055;
    p.upper_arm_mass = 4.0;
    p.lower_arm_length = 0.32;
    p.lower_arm_radius = 0.04;
    p.lower_arm_mass = 2.5;
    p.hand_mass = 0.8;
    p.head_radius = 0.12;
    p.head_mass = 4.0;
    p.leg_torque = 250.0;
    p.arm_torque = 80.0;
    p.kp = 300.0;
    p.kd = 15.0;
    p.nominal_height = 1.5;
    p.body_color = Eigen::Vector4d(0.2, 0.3, 0.6, 1.0);
    p.limb_color = Eigen::Vector4d(0.25, 0.25, 0.3, 1.0);
    p.has_arms = true;
    p.has_hands = true;
    return build_humanoid(p);
}

// Agility Robotics Digit
inline core::RobotDef create_digit() {
    HumanoidParams p;
    p.name = "agility_digit";
    p.description = "Agility Robotics Digit - Bipedal robot for logistics and warehouse";
    p.torso_width = 0.32;
    p.torso_depth = 0.20;
    p.torso_height = 0.40;
    p.torso_mass = 20.0;
    p.pelvis_width = 0.24;
    p.pelvis_height = 0.12;
    p.pelvis_mass = 6.0;
    p.upper_leg_length = 0.40;
    p.upper_leg_radius = 0.05;
    p.upper_leg_mass = 4.5;
    p.lower_leg_length = 0.45;
    p.lower_leg_radius = 0.04;
    p.lower_leg_mass = 2.5;
    p.foot_length = 0.20;
    p.foot_width = 0.10;
    p.foot_height = 0.04;
    p.foot_mass = 0.6;
    p.upper_arm_length = 0.30;
    p.upper_arm_radius = 0.04;
    p.upper_arm_mass = 2.0;
    p.lower_arm_length = 0.28;
    p.lower_arm_radius = 0.03;
    p.lower_arm_mass = 1.2;
    p.leg_torque = 180.0;
    p.arm_torque = 40.0;
    p.kp = 200.0;
    p.kd = 10.0;
    p.nominal_height = 1.35;
    p.body_color = Eigen::Vector4d(0.15, 0.15, 0.15, 1.0);
    p.limb_color = Eigen::Vector4d(0.2, 0.35, 0.5, 1.0);
    p.has_arms = true;
    return build_humanoid(p);
}

// Tesla Optimus Gen 2
inline core::RobotDef create_optimus() {
    HumanoidParams p;
    p.name = "tesla_optimus";
    p.description = "Tesla Optimus Gen 2 - General-purpose humanoid for manufacturing";
    p.torso_width = 0.38;
    p.torso_depth = 0.22;
    p.torso_height = 0.45;
    p.torso_mass = 25.0;
    p.pelvis_width = 0.28;
    p.pelvis_height = 0.15;
    p.pelvis_mass = 8.0;
    p.upper_leg_length = 0.42;
    p.upper_leg_radius = 0.06;
    p.upper_leg_mass = 5.5;
    p.lower_leg_length = 0.42;
    p.lower_leg_radius = 0.045;
    p.lower_leg_mass = 3.5;
    p.foot_length = 0.24;
    p.foot_width = 0.12;
    p.foot_height = 0.05;
    p.foot_mass = 1.0;
    p.upper_arm_length = 0.32;
    p.upper_arm_radius = 0.045;
    p.upper_arm_mass = 3.0;
    p.lower_arm_length = 0.30;
    p.lower_arm_radius = 0.035;
    p.lower_arm_mass = 1.8;
    p.hand_length = 0.12;
    p.hand_width = 0.10;
    p.hand_mass = 0.6;
    p.leg_torque = 200.0;
    p.arm_torque = 60.0;
    p.kp = 250.0;
    p.kd = 12.0;
    p.nominal_height = 1.73;
    p.body_color = Eigen::Vector4d(0.9, 0.9, 0.9, 1.0);
    p.limb_color = Eigen::Vector4d(0.1, 0.1, 0.1, 1.0);
    p.has_arms = true;
    p.has_hands = true;
    return build_humanoid(p);
}

// Figure 01
inline core::RobotDef create_figure01() {
    HumanoidParams p;
    p.name = "figure_01";
    p.description = "Figure 01 - AI-powered humanoid for general tasks";
    p.torso_width = 0.35;
    p.torso_depth = 0.20;
    p.torso_height = 0.42;
    p.torso_mass = 22.0;
    p.pelvis_width = 0.26;
    p.pelvis_height = 0.14;
    p.pelvis_mass = 7.0;
    p.upper_leg_length = 0.40;
    p.upper_leg_radius = 0.055;
    p.upper_leg_mass = 5.0;
    p.lower_leg_length = 0.40;
    p.lower_leg_radius = 0.04;
    p.lower_leg_mass = 3.0;
    p.foot_length = 0.22;
    p.foot_width = 0.11;
    p.foot_height = 0.05;
    p.foot_mass = 0.9;
    p.upper_arm_length = 0.30;
    p.upper_arm_radius = 0.04;
    p.upper_arm_mass = 2.5;
    p.lower_arm_length = 0.28;
    p.lower_arm_radius = 0.035;
    p.lower_arm_mass = 1.5;
    p.hand_length = 0.11;
    p.hand_width = 0.09;
    p.hand_mass = 0.5;
    p.leg_torque = 190.0;
    p.arm_torque = 55.0;
    p.kp = 230.0;
    p.kd = 11.0;
    p.nominal_height = 1.68;
    p.body_color = Eigen::Vector4d(0.85, 0.85, 0.85, 1.0);
    p.limb_color = Eigen::Vector4d(0.2, 0.2, 0.2, 1.0);
    p.has_arms = true;
    p.has_hands = true;
    return build_humanoid(p);
}

} // namespace robosim::robot::definitions
