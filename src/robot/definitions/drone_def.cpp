#include "robosim/robot/definitions/drone_def.h"
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

core::RobotDef build_drone(const DroneParams& p) {
    core::RobotDef def;
    def.name = p.name;
    def.description = p.description;
    def.category = core::RobotCategory::Drone;

    // Central body
    {
        core::LinkDef body;
        body.name = "base_link";
        body.inertial.mass = p.body_mass;
        body.inertial.inertia = cylinder_inertia(p.body_mass, p.body_radius, p.body_height);
        core::CollisionShape col;
        col.type = core::ShapeType::Cylinder;
        col.dimensions = Eigen::Vector3d(p.body_radius, 0, p.body_height);
        body.collision_shapes.push_back(col);
        core::VisualShape vis;
        vis.type = core::ShapeType::Cylinder;
        vis.dimensions = col.dimensions;
        vis.color = p.body_color;
        body.visual_shapes.push_back(vis);
        def.links.push_back(body);
    }

    // Arms and rotors
    for (int i = 0; i < p.num_rotors; i++) {
        double angle = 2.0 * M_PI * i / p.num_rotors;
        // Offset for X-configuration (45 degrees) for quadrotor
        if (p.num_rotors == 4) {
            angle += M_PI / 4.0;
        }

        double cx = p.arm_length * std::cos(angle);
        double cy = p.arm_length * std::sin(angle);
        std::string idx = std::to_string(i);

        // Arm link
        {
            core::LinkDef arm;
            arm.name = "arm_" + idx;
            arm.inertial.mass = p.arm_mass;
            arm.inertial.inertia = box_inertia(p.arm_mass,
                p.arm_length, p.arm_radius * 2, p.arm_radius * 2);
            arm.inertial.com = Eigen::Vector3d(cx / 2.0, cy / 2.0, 0);

            core::CollisionShape col;
            col.type = core::ShapeType::Box;
            col.dimensions = Eigen::Vector3d(p.arm_length, p.arm_radius * 2, p.arm_radius * 2);
            col.origin_xyz = Eigen::Vector3d(cx / 2.0, cy / 2.0, 0);
            col.origin_rpy = Eigen::Vector3d(0, 0, angle);
            arm.collision_shapes.push_back(col);

            core::VisualShape vis;
            vis.type = core::ShapeType::Box;
            vis.dimensions = col.dimensions;
            vis.origin_xyz = col.origin_xyz;
            vis.origin_rpy = col.origin_rpy;
            vis.color = p.arm_color;
            arm.visual_shapes.push_back(vis);
            def.links.push_back(arm);
        }

        // Fixed joint: body -> arm
        {
            core::JointDef j;
            j.name = "arm_joint_" + idx;
            j.type = core::JointType::Fixed;
            j.parent_link = "base_link";
            j.child_link = "arm_" + idx;
            j.origin_xyz = Eigen::Vector3d::Zero();
            def.joints.push_back(j);
        }

        // Rotor link
        {
            core::LinkDef rotor;
            rotor.name = "rotor_" + idx;
            rotor.inertial.mass = p.rotor_mass;
            rotor.inertial.inertia = cylinder_inertia(p.rotor_mass, p.rotor_radius, p.rotor_height);

            core::CollisionShape col;
            col.type = core::ShapeType::Cylinder;
            col.dimensions = Eigen::Vector3d(p.rotor_radius, 0, p.rotor_height);
            rotor.collision_shapes.push_back(col);

            core::VisualShape vis;
            vis.type = core::ShapeType::Cylinder;
            vis.dimensions = col.dimensions;
            vis.color = p.rotor_color;
            rotor.visual_shapes.push_back(vis);
            def.links.push_back(rotor);
        }

        // Continuous joint: arm -> rotor (spinning rotor)
        {
            core::JointDef j;
            j.name = "rotor_joint_" + idx;
            j.type = core::JointType::Continuous;
            j.parent_link = "arm_" + idx;
            j.child_link = "rotor_" + idx;
            j.axis = Eigen::Vector3d::UnitZ();
            j.origin_xyz = Eigen::Vector3d(cx, cy, p.body_height / 2.0 + p.rotor_height / 2.0);
            j.max_effort = p.max_thrust;
            def.joints.push_back(j);

            // Actuator: use Torque type for direct thrust control
            core::ActuatorParams act;
            act.type = core::ActuatorType::Torque;
            act.max_torque = p.max_thrust;
            def.actuators.push_back(act);
        }
    }

    // Default joint positions (all zeros - rotors start at rest)
    def.default_joint_positions.resize(p.num_rotors, 0.0);
    def.spawn_position = Eigen::Vector3d(0, 0, p.nominal_height);
    def.nominal_height = p.nominal_height;

    double total = 0;
    for (auto& l : def.links) total += l.inertial.mass;
    def.total_mass = total;

    return def;
}

} // namespace robosim::robot::definitions
