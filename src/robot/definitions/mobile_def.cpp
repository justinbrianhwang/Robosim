#include "robosim/robot/definitions/mobile_def.h"
#include <cmath>

namespace robosim::robot::definitions {

namespace {

Eigen::Matrix3d box_inertia(double mass, double lx, double ly, double lz) {
    Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
    I(0,0) = mass / 12.0 * (ly*ly + lz*lz);
    I(1,1) = mass / 12.0 * (lx*lx + lz*lz);
    I(2,2) = mass / 12.0 * (lx*lx + ly*ly);
    return I;
}

Eigen::Matrix3d cylinder_inertia(double mass, double radius, double height) {
    Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
    I(0,0) = mass / 12.0 * (3*radius*radius + height*height);
    I(1,1) = I(0,0);
    I(2,2) = mass / 2.0 * radius * radius;
    return I;
}

Eigen::Matrix3d sphere_inertia(double mass, double radius) {
    double i = 2.0/5.0 * mass * radius * radius;
    return Eigen::Matrix3d::Identity() * i;
}

} // anonymous namespace

core::RobotDef build_mobile_robot(const MobileRobotParams& p) {
    core::RobotDef def;
    def.name = p.name;
    def.description = p.description;
    def.category = core::RobotCategory::MobileRobot;

    // Body
    {
        core::LinkDef body;
        body.name = "base_link";
        body.inertial.mass = p.body_mass;
        body.inertial.inertia = box_inertia(p.body_mass, p.body_length, p.body_width, p.body_height);

        core::CollisionShape col;
        col.type = core::ShapeType::Box;
        col.dimensions = Eigen::Vector3d(p.body_length, p.body_width, p.body_height);
        body.collision_shapes.push_back(col);

        core::VisualShape vis;
        vis.type = core::ShapeType::Box;
        vis.dimensions = Eigen::Vector3d(p.body_length, p.body_width, p.body_height);
        vis.color = p.body_color;
        body.visual_shapes.push_back(vis);
        def.links.push_back(body);
    }

    std::vector<double> default_q;

    if (p.drive_type == MobileRobotParams::DriveType::Differential) {
        // Two drive wheels (left and right)
        for (int side = 0; side < 2; side++) {
            double y_sign = (side == 0) ? 1.0 : -1.0;
            std::string name = (side == 0) ? "left_wheel" : "right_wheel";

            core::LinkDef wheel;
            wheel.name = name;
            wheel.inertial.mass = p.wheel_mass;
            wheel.inertial.inertia = cylinder_inertia(p.wheel_mass, p.wheel_radius, p.wheel_width);

            core::CollisionShape col;
            col.type = core::ShapeType::Cylinder;
            col.dimensions = Eigen::Vector3d(p.wheel_radius, 0, p.wheel_width);
            col.origin_rpy = Eigen::Vector3d(M_PI/2.0, 0, 0);
            wheel.collision_shapes.push_back(col);

            core::VisualShape vis;
            vis.type = core::ShapeType::Cylinder;
            vis.dimensions = col.dimensions;
            vis.origin_rpy = col.origin_rpy;
            vis.color = p.wheel_color;
            wheel.visual_shapes.push_back(vis);
            def.links.push_back(wheel);

            core::JointDef joint;
            joint.name = name + "_joint";
            joint.type = core::JointType::Continuous;
            joint.parent_link = "base_link";
            joint.child_link = name;
            joint.axis = Eigen::Vector3d::UnitY();
            joint.origin_xyz = Eigen::Vector3d(0, y_sign * p.track_width / 2.0,
                                                -p.body_height / 2.0);
            joint.max_effort = p.max_wheel_torque;
            joint.max_velocity = p.max_wheel_velocity;
            def.joints.push_back(joint);

            core::ActuatorParams act;
            act.type = core::ActuatorType::VelocityPI;
            act.kp = p.kp;
            act.ki = 0.1;
            act.max_torque = p.max_wheel_torque;
            def.actuators.push_back(act);
            default_q.push_back(0.0);
        }

        // Front caster wheel
        {
            core::LinkDef caster;
            caster.name = "caster_wheel";
            caster.inertial.mass = p.caster_mass;
            caster.inertial.inertia = sphere_inertia(p.caster_mass, p.caster_radius);

            core::CollisionShape col;
            col.type = core::ShapeType::Sphere;
            col.dimensions = Eigen::Vector3d(p.caster_radius, 0, 0);
            caster.collision_shapes.push_back(col);

            core::VisualShape vis;
            vis.type = core::ShapeType::Sphere;
            vis.dimensions = col.dimensions;
            vis.color = Eigen::Vector4d(0.4, 0.4, 0.4, 1.0);
            caster.visual_shapes.push_back(vis);
            def.links.push_back(caster);

            core::JointDef j;
            j.name = "caster_joint";
            j.type = core::JointType::Spherical;
            j.parent_link = "base_link";
            j.child_link = "caster_wheel";
            j.origin_xyz = Eigen::Vector3d(p.body_length / 2.0 - 0.03, 0,
                                            -p.body_height / 2.0 - (p.wheel_radius - p.caster_radius));
            def.joints.push_back(j);
        }

    } else {
        // Ackermann: 4 wheels with front steering
        struct WheelConfig {
            std::string name;
            double x_off;
            double y_sign;
            bool steered;
        };
        std::vector<WheelConfig> wheels = {
            {"front_left_wheel",  p.wheelbase / 2.0,  1.0, true},
            {"front_right_wheel", p.wheelbase / 2.0, -1.0, true},
            {"rear_left_wheel",  -p.wheelbase / 2.0,  1.0, false},
            {"rear_right_wheel", -p.wheelbase / 2.0, -1.0, false},
        };

        for (auto& wc : wheels) {
            std::string steer_name = wc.name + "_steer";

            if (wc.steered) {
                // Steering link (virtual)
                core::LinkDef steer_link;
                steer_link.name = steer_name;
                steer_link.inertial.mass = 0.1;
                steer_link.inertial.inertia = Eigen::Matrix3d::Identity() * 1e-4;
                def.links.push_back(steer_link);

                core::JointDef steer_joint;
                steer_joint.name = steer_name + "_joint";
                steer_joint.type = core::JointType::Revolute;
                steer_joint.parent_link = "base_link";
                steer_joint.child_link = steer_name;
                steer_joint.axis = Eigen::Vector3d::UnitZ();
                steer_joint.origin_xyz = Eigen::Vector3d(wc.x_off, wc.y_sign * p.track_width / 2.0,
                                                          -p.body_height / 2.0);
                steer_joint.lower_limit = -p.max_steering_angle;
                steer_joint.upper_limit = p.max_steering_angle;
                steer_joint.max_effort = 50.0;
                def.joints.push_back(steer_joint);

                core::ActuatorParams steer_act;
                steer_act.type = core::ActuatorType::PositionPD;
                steer_act.kp = p.kp * 2.0;
                steer_act.kd = p.kd;
                steer_act.max_torque = 50.0;
                def.actuators.push_back(steer_act);
                default_q.push_back(0.0);
            }

            // Wheel
            core::LinkDef wheel;
            wheel.name = wc.name;
            wheel.inertial.mass = p.wheel_mass;
            wheel.inertial.inertia = cylinder_inertia(p.wheel_mass, p.wheel_radius, p.wheel_width);

            core::CollisionShape col;
            col.type = core::ShapeType::Cylinder;
            col.dimensions = Eigen::Vector3d(p.wheel_radius, 0, p.wheel_width);
            col.origin_rpy = Eigen::Vector3d(M_PI/2.0, 0, 0);
            wheel.collision_shapes.push_back(col);

            core::VisualShape vis;
            vis.type = core::ShapeType::Cylinder;
            vis.dimensions = col.dimensions;
            vis.origin_rpy = col.origin_rpy;
            vis.color = p.wheel_color;
            wheel.visual_shapes.push_back(vis);
            def.links.push_back(wheel);

            core::JointDef wheel_joint;
            wheel_joint.name = wc.name + "_joint";
            wheel_joint.type = core::JointType::Continuous;
            wheel_joint.parent_link = wc.steered ? steer_name : "base_link";
            wheel_joint.child_link = wc.name;
            wheel_joint.axis = Eigen::Vector3d::UnitY();
            if (!wc.steered) {
                wheel_joint.origin_xyz = Eigen::Vector3d(wc.x_off, wc.y_sign * p.track_width / 2.0,
                                                          -p.body_height / 2.0);
            }
            wheel_joint.max_effort = p.max_wheel_torque;
            wheel_joint.max_velocity = p.max_wheel_velocity;
            def.joints.push_back(wheel_joint);

            core::ActuatorParams wheel_act;
            wheel_act.type = core::ActuatorType::VelocityPI;
            wheel_act.kp = p.kp;
            wheel_act.ki = 0.1;
            wheel_act.max_torque = p.max_wheel_torque;
            def.actuators.push_back(wheel_act);
            default_q.push_back(0.0);
        }
    }

    def.default_joint_positions = default_q;
    def.spawn_position = Eigen::Vector3d(0, 0, p.nominal_height);
    def.nominal_height = p.nominal_height;

    double total = 0;
    for (auto& l : def.links) total += l.inertial.mass;
    def.total_mass = total;

    return def;
}

} // namespace robosim::robot::definitions
