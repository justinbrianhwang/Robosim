#include "robosim/robot/definitions/quadruped_def.h"
#include <cmath>

namespace robosim::robot::definitions {

namespace {

// Compute inertia tensor for a box
Eigen::Matrix3d box_inertia(double mass, double lx, double ly, double lz) {
    Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
    I(0,0) = mass / 12.0 * (ly*ly + lz*lz);
    I(1,1) = mass / 12.0 * (lx*lx + lz*lz);
    I(2,2) = mass / 12.0 * (lx*lx + ly*ly);
    return I;
}

// Compute inertia tensor for a cylinder (aligned along Z)
Eigen::Matrix3d cylinder_inertia(double mass, double radius, double height) {
    Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
    I(0,0) = mass / 12.0 * (3*radius*radius + height*height);
    I(1,1) = I(0,0);
    I(2,2) = mass / 2.0 * radius * radius;
    return I;
}

// Compute inertia tensor for a sphere
Eigen::Matrix3d sphere_inertia(double mass, double radius) {
    double i = 2.0/5.0 * mass * radius * radius;
    return Eigen::Matrix3d::Identity() * i;
}

core::LinkDef make_body_link(const QuadrupedParams& p) {
    core::LinkDef link;
    link.name = "base_link";
    link.inertial.mass = p.body_mass;
    link.inertial.inertia = box_inertia(p.body_mass, p.body_length, p.body_width, p.body_height);

    core::CollisionShape col;
    col.type = core::ShapeType::Box;
    col.dimensions = Eigen::Vector3d(p.body_length, p.body_width, p.body_height);
    link.collision_shapes.push_back(col);

    core::VisualShape vis;
    vis.type = core::ShapeType::Box;
    vis.dimensions = Eigen::Vector3d(p.body_length, p.body_width, p.body_height);
    vis.color = p.body_color;
    link.visual_shapes.push_back(vis);

    return link;
}

struct LegLinks {
    core::LinkDef hip;
    core::LinkDef thigh;
    core::LinkDef calf;
    core::LinkDef foot;
    core::JointDef hip_abd_joint;
    core::JointDef hip_flex_joint;
    core::JointDef knee_joint;
};

LegLinks make_leg(const QuadrupedParams& p, const std::string& prefix,
                  double x_offset, double y_sign) {
    LegLinks leg;
    double y_offset = y_sign * (p.body_width / 2.0 + p.hip_length / 2.0);

    // Hip link
    leg.hip.name = prefix + "_hip";
    leg.hip.inertial.mass = p.hip_mass;
    leg.hip.inertial.inertia = cylinder_inertia(p.hip_mass, p.hip_radius, p.hip_length);
    {
        core::CollisionShape col;
        col.type = core::ShapeType::Cylinder;
        col.dimensions = Eigen::Vector3d(p.hip_radius, 0, p.hip_length);
        col.origin_rpy = Eigen::Vector3d(M_PI/2.0, 0, 0); // align Y
        leg.hip.collision_shapes.push_back(col);
        core::VisualShape vis;
        vis.type = core::ShapeType::Cylinder;
        vis.dimensions = col.dimensions;
        vis.origin_rpy = col.origin_rpy;
        vis.color = p.leg_color;
        leg.hip.visual_shapes.push_back(vis);
    }

    // Thigh link
    leg.thigh.name = prefix + "_thigh";
    leg.thigh.inertial.mass = p.thigh_mass;
    leg.thigh.inertial.inertia = cylinder_inertia(p.thigh_mass, p.thigh_radius, p.thigh_length);
    leg.thigh.inertial.com = Eigen::Vector3d(0, 0, -p.thigh_length / 2.0);
    {
        core::CollisionShape col;
        col.type = core::ShapeType::Capsule;
        col.dimensions = Eigen::Vector3d(p.thigh_radius, 0, p.thigh_length);
        col.origin_xyz = Eigen::Vector3d(0, 0, -p.thigh_length / 2.0);
        leg.thigh.collision_shapes.push_back(col);
        core::VisualShape vis;
        vis.type = core::ShapeType::Capsule;
        vis.dimensions = col.dimensions;
        vis.origin_xyz = col.origin_xyz;
        vis.color = p.leg_color;
        leg.thigh.visual_shapes.push_back(vis);
    }

    // Calf link
    leg.calf.name = prefix + "_calf";
    leg.calf.inertial.mass = p.calf_mass;
    leg.calf.inertial.inertia = cylinder_inertia(p.calf_mass, p.calf_radius, p.calf_length);
    leg.calf.inertial.com = Eigen::Vector3d(0, 0, -p.calf_length / 2.0);
    {
        core::CollisionShape col;
        col.type = core::ShapeType::Capsule;
        col.dimensions = Eigen::Vector3d(p.calf_radius, 0, p.calf_length);
        col.origin_xyz = Eigen::Vector3d(0, 0, -p.calf_length / 2.0);
        leg.calf.collision_shapes.push_back(col);
        core::VisualShape vis;
        vis.type = core::ShapeType::Capsule;
        vis.dimensions = col.dimensions;
        vis.origin_xyz = col.origin_xyz;
        vis.color = p.leg_color;
        leg.calf.visual_shapes.push_back(vis);
    }

    // Foot (sphere)
    leg.foot.name = prefix + "_foot";
    leg.foot.inertial.mass = p.foot_mass;
    leg.foot.inertial.inertia = sphere_inertia(p.foot_mass, p.foot_radius);
    {
        core::CollisionShape col;
        col.type = core::ShapeType::Sphere;
        col.dimensions = Eigen::Vector3d(p.foot_radius, 0, 0);
        leg.foot.collision_shapes.push_back(col);
        core::VisualShape vis;
        vis.type = core::ShapeType::Sphere;
        vis.dimensions = col.dimensions;
        vis.color = Eigen::Vector4d(0.6, 0.6, 0.6, 1.0);
        leg.foot.visual_shapes.push_back(vis);
    }

    // Hip abduction joint (base_link -> hip)
    leg.hip_abd_joint.name = prefix + "_hip_abduction";
    leg.hip_abd_joint.type = core::JointType::Revolute;
    leg.hip_abd_joint.parent_link = "base_link";
    leg.hip_abd_joint.child_link = leg.hip.name;
    leg.hip_abd_joint.axis = Eigen::Vector3d::UnitX();
    leg.hip_abd_joint.origin_xyz = Eigen::Vector3d(x_offset, y_offset, 0);
    leg.hip_abd_joint.lower_limit = -p.hip_abduction_range;
    leg.hip_abd_joint.upper_limit = p.hip_abduction_range;
    leg.hip_abd_joint.max_effort = p.hip_torque;
    leg.hip_abd_joint.damping = 0.5;

    // Hip flexion joint (hip -> thigh)
    leg.hip_flex_joint.name = prefix + "_hip_flexion";
    leg.hip_flex_joint.type = core::JointType::Revolute;
    leg.hip_flex_joint.parent_link = leg.hip.name;
    leg.hip_flex_joint.child_link = leg.thigh.name;
    leg.hip_flex_joint.axis = Eigen::Vector3d::UnitY();
    leg.hip_flex_joint.origin_xyz = Eigen::Vector3d(0, y_sign * p.hip_length / 2.0, 0);
    leg.hip_flex_joint.lower_limit = -p.hip_flexion_range / 2.0;
    leg.hip_flex_joint.upper_limit = p.hip_flexion_range / 2.0;
    leg.hip_flex_joint.max_effort = p.hip_torque;
    leg.hip_flex_joint.damping = 0.5;

    // Knee joint (thigh -> calf)
    leg.knee_joint.name = prefix + "_knee";
    leg.knee_joint.type = core::JointType::Revolute;
    leg.knee_joint.parent_link = leg.thigh.name;
    leg.knee_joint.child_link = leg.calf.name;
    leg.knee_joint.axis = Eigen::Vector3d::UnitY();
    leg.knee_joint.origin_xyz = Eigen::Vector3d(0, 0, -p.thigh_length);
    leg.knee_joint.lower_limit = -p.knee_range;
    leg.knee_joint.upper_limit = -0.1; // knees bend backward
    leg.knee_joint.max_effort = p.knee_torque;
    leg.knee_joint.damping = 0.3;

    return leg;
}

} // anonymous namespace

core::RobotDef build_quadruped(const QuadrupedParams& p) {
    core::RobotDef def;
    def.name = p.name;
    def.description = p.description;
    def.category = core::RobotCategory::Quadruped;

    // Base link
    def.links.push_back(make_body_link(p));

    // Leg prefixes and offsets
    struct LegConfig {
        std::string prefix;
        double x_offset;
        double y_sign;
    };
    std::vector<LegConfig> legs = {
        {"FL", p.body_length / 2.0, 1.0},   // Front Left
        {"FR", p.body_length / 2.0, -1.0},  // Front Right
        {"RL", -p.body_length / 2.0, 1.0},  // Rear Left
        {"RR", -p.body_length / 2.0, -1.0}, // Rear Right
    };

    // Default joint positions (slight crouch)
    std::vector<double> default_q;

    for (auto& lc : legs) {
        LegLinks leg = make_leg(p, lc.prefix, lc.x_offset, lc.y_sign);

        def.links.push_back(leg.hip);
        def.links.push_back(leg.thigh);
        def.links.push_back(leg.calf);
        def.links.push_back(leg.foot);

        def.joints.push_back(leg.hip_abd_joint);
        def.joints.push_back(leg.hip_flex_joint);
        def.joints.push_back(leg.knee_joint);

        // Fixed joint: calf -> foot
        core::JointDef foot_joint;
        foot_joint.name = lc.prefix + "_foot_fixed";
        foot_joint.type = core::JointType::Fixed;
        foot_joint.parent_link = leg.calf.name;
        foot_joint.child_link = leg.foot.name;
        foot_joint.origin_xyz = Eigen::Vector3d(0, 0, -p.calf_length);
        def.joints.push_back(foot_joint);

        // Actuators for movable joints (3 per leg)
        core::ActuatorParams abd_act;
        abd_act.type = core::ActuatorType::PositionPD;
        abd_act.kp = p.kp;
        abd_act.kd = p.kd;
        abd_act.max_torque = p.hip_torque;
        def.actuators.push_back(abd_act);

        core::ActuatorParams flex_act;
        flex_act.type = core::ActuatorType::PositionPD;
        flex_act.kp = p.kp;
        flex_act.kd = p.kd;
        flex_act.max_torque = p.hip_torque;
        def.actuators.push_back(flex_act);

        core::ActuatorParams knee_act;
        knee_act.type = core::ActuatorType::PositionPD;
        knee_act.kp = p.kp * 1.2;
        knee_act.kd = p.kd * 1.2;
        knee_act.max_torque = p.knee_torque;
        def.actuators.push_back(knee_act);

        // Default positions: 0 abduction, slight hip flexion, bent knee
        default_q.push_back(0.0);    // hip abduction
        default_q.push_back(0.6);    // hip flexion
        default_q.push_back(-1.2);   // knee
    }

    def.default_joint_positions = default_q;
    def.spawn_position = Eigen::Vector3d(0, 0, p.nominal_height);
    def.nominal_height = p.nominal_height;

    // Compute total mass
    double total = 0;
    for (auto& l : def.links) total += l.inertial.mass;
    def.total_mass = total;

    return def;
}

} // namespace robosim::robot::definitions
