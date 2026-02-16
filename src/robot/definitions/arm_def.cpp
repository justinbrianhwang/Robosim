#include "robosim/robot/definitions/arm_def.h"
#include <cmath>

namespace robosim::robot::definitions {

namespace {

Eigen::Matrix3d cylinder_inertia(double mass, double radius, double height) {
    Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
    I(0,0) = mass / 12.0 * (3*radius*radius + height*height);
    I(1,1) = I(0,0);
    I(2,2) = mass / 2.0 * radius * radius;
    return I;
}

Eigen::Matrix3d box_inertia(double mass, double lx, double ly, double lz) {
    Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
    I(0,0) = mass / 12.0 * (ly*ly + lz*lz);
    I(1,1) = mass / 12.0 * (lx*lx + lz*lz);
    I(2,2) = mass / 12.0 * (lx*lx + ly*ly);
    return I;
}

} // anonymous namespace

core::RobotDef build_arm(const ArmParams& p) {
    core::RobotDef def;
    def.name = p.name;
    def.description = p.description;
    def.category = core::RobotCategory::Arm;

    // Base link (fixed to world conceptually)
    {
        core::LinkDef base;
        base.name = "base_link";
        base.inertial.mass = p.base_mass;
        base.inertial.inertia = cylinder_inertia(p.base_mass, p.base_radius, p.base_height);
        core::CollisionShape col;
        col.type = core::ShapeType::Cylinder;
        col.dimensions = Eigen::Vector3d(p.base_radius, 0, p.base_height);
        base.collision_shapes.push_back(col);
        core::VisualShape vis;
        vis.type = core::ShapeType::Cylinder;
        vis.dimensions = col.dimensions;
        vis.color = p.joint_color;
        base.visual_shapes.push_back(vis);
        def.links.push_back(base);
    }

    std::vector<double> default_q;
    std::string prev_link = "base_link";
    double cumulative_height = p.base_height;

    for (int i = 0; i < p.dof; i++) {
        std::string link_name = "link_" + std::to_string(i + 1);
        double length = p.link_lengths[i];
        double radius = p.link_radii[i];
        double mass = p.link_masses[i];
        bool is_vertical = (length > 0.05);

        // Create link
        core::LinkDef link;
        link.name = link_name;
        link.inertial.mass = mass;

        if (is_vertical) {
            link.inertial.inertia = cylinder_inertia(mass, radius, length);
            link.inertial.com = Eigen::Vector3d(0, 0, length / 2.0);

            core::CollisionShape col;
            col.type = core::ShapeType::Capsule;
            col.dimensions = Eigen::Vector3d(radius, 0, length);
            col.origin_xyz = Eigen::Vector3d(0, 0, length / 2.0);
            link.collision_shapes.push_back(col);

            core::VisualShape vis;
            vis.type = core::ShapeType::Capsule;
            vis.dimensions = col.dimensions;
            vis.origin_xyz = col.origin_xyz;
            vis.color = (i % 2 == 0) ? p.body_color : p.joint_color;
            link.visual_shapes.push_back(vis);
        } else {
            // Short connector link - use sphere at joint
            double r = radius * 1.2;
            link.inertial.inertia = cylinder_inertia(mass, r, r * 2);
            core::CollisionShape col;
            col.type = core::ShapeType::Sphere;
            col.dimensions = Eigen::Vector3d(r, 0, 0);
            link.collision_shapes.push_back(col);
            core::VisualShape vis;
            vis.type = core::ShapeType::Sphere;
            vis.dimensions = col.dimensions;
            vis.color = p.joint_color;
            link.visual_shapes.push_back(vis);
        }
        def.links.push_back(link);

        // Create joint
        core::JointDef joint;
        joint.name = "joint_" + std::to_string(i + 1);
        joint.type = core::JointType::Revolute;
        joint.parent_link = prev_link;
        joint.child_link = link_name;
        joint.axis = p.joint_axes[i];
        joint.origin_xyz = Eigen::Vector3d(0, 0, (prev_link == "base_link") ? p.base_height : p.link_lengths[i-1]);
        joint.lower_limit = p.joint_lower[i];
        joint.upper_limit = p.joint_upper[i];
        joint.max_effort = p.max_torques[i];
        joint.max_velocity = p.max_velocities[i];
        joint.damping = 0.5;
        def.joints.push_back(joint);

        // Actuator
        core::ActuatorParams act;
        act.type = core::ActuatorType::PositionPD;
        act.kp = p.kp;
        act.kd = p.kd;
        act.max_torque = p.max_torques[i];
        def.actuators.push_back(act);

        default_q.push_back(0.0);
        prev_link = link_name;
    }

    // End-effector
    {
        core::LinkDef ee;
        ee.name = "end_effector";
        ee.inertial.mass = p.ee_mass;
        ee.inertial.inertia = cylinder_inertia(p.ee_mass, p.ee_radius, p.ee_length);
        core::CollisionShape col;
        col.type = core::ShapeType::Cylinder;
        col.dimensions = Eigen::Vector3d(p.ee_radius, 0, p.ee_length);
        col.origin_xyz = Eigen::Vector3d(0, 0, p.ee_length / 2.0);
        ee.collision_shapes.push_back(col);
        core::VisualShape vis;
        vis.type = core::ShapeType::Cylinder;
        vis.dimensions = col.dimensions;
        vis.origin_xyz = col.origin_xyz;
        vis.color = Eigen::Vector4d(0.8, 0.2, 0.2, 1.0);
        ee.visual_shapes.push_back(vis);
        def.links.push_back(ee);
    }

    // Fixed joint to end-effector
    core::JointDef ee_joint;
    ee_joint.name = "ee_fixed";
    ee_joint.type = core::JointType::Fixed;
    ee_joint.parent_link = prev_link;
    ee_joint.child_link = "end_effector";
    ee_joint.origin_xyz = Eigen::Vector3d(0, 0, p.link_lengths.back());
    def.joints.push_back(ee_joint);

    // Gripper (optional)
    if (p.has_gripper) {
        // Left finger
        core::LinkDef lf;
        lf.name = "gripper_left";
        lf.inertial.mass = 0.05;
        lf.inertial.inertia = box_inertia(0.05, 0.01, 0.005, 0.04);
        core::CollisionShape col;
        col.type = core::ShapeType::Box;
        col.dimensions = Eigen::Vector3d(0.01, 0.005, 0.04);
        lf.collision_shapes.push_back(col);
        core::VisualShape vis;
        vis.type = core::ShapeType::Box;
        vis.dimensions = col.dimensions;
        vis.color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0);
        lf.visual_shapes.push_back(vis);
        def.links.push_back(lf);

        core::JointDef lf_joint;
        lf_joint.name = "gripper_left_joint";
        lf_joint.type = core::JointType::Prismatic;
        lf_joint.parent_link = "end_effector";
        lf_joint.child_link = "gripper_left";
        lf_joint.axis = Eigen::Vector3d::UnitY();
        lf_joint.origin_xyz = Eigen::Vector3d(0, p.gripper_width / 4.0, p.ee_length);
        lf_joint.lower_limit = 0.0;
        lf_joint.upper_limit = p.gripper_width / 2.0;
        lf_joint.max_effort = 20.0;
        def.joints.push_back(lf_joint);

        core::ActuatorParams grip_act;
        grip_act.type = core::ActuatorType::PositionPD;
        grip_act.kp = 100.0;
        grip_act.kd = 5.0;
        grip_act.max_torque = 20.0;
        def.actuators.push_back(grip_act);
        default_q.push_back(p.gripper_width / 4.0);

        // Right finger (mirror)
        core::LinkDef rf;
        rf.name = "gripper_right";
        rf.inertial = lf.inertial;
        rf.collision_shapes = lf.collision_shapes;
        rf.visual_shapes = lf.visual_shapes;
        def.links.push_back(rf);

        core::JointDef rf_joint = lf_joint;
        rf_joint.name = "gripper_right_joint";
        rf_joint.child_link = "gripper_right";
        rf_joint.origin_xyz = Eigen::Vector3d(0, -p.gripper_width / 4.0, p.ee_length);
        rf_joint.axis = -Eigen::Vector3d::UnitY();
        def.joints.push_back(rf_joint);
        def.actuators.push_back(grip_act);
        default_q.push_back(p.gripper_width / 4.0);
    }

    def.default_joint_positions = default_q;
    def.spawn_position = Eigen::Vector3d(0, 0, 0); // arms sit on table/ground
    def.nominal_height = 0.0;

    double total = 0;
    for (auto& l : def.links) total += l.inertial.mass;
    def.total_mass = total;

    return def;
}

} // namespace robosim::robot::definitions
