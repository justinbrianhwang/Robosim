#include "robosim/robot/definitions/humanoid_def.h"
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

void add_box_link(core::RobotDef& def, const std::string& name,
                  double mass, double lx, double ly, double lz,
                  const Eigen::Vector4d& color,
                  const Eigen::Vector3d& com = Eigen::Vector3d::Zero()) {
    core::LinkDef link;
    link.name = name;
    link.inertial.mass = mass;
    link.inertial.inertia = box_inertia(mass, lx, ly, lz);
    link.inertial.com = com;
    core::CollisionShape col;
    col.type = core::ShapeType::Box;
    col.dimensions = Eigen::Vector3d(lx, ly, lz);
    col.origin_xyz = com;
    link.collision_shapes.push_back(col);
    core::VisualShape vis;
    vis.type = core::ShapeType::Box;
    vis.dimensions = Eigen::Vector3d(lx, ly, lz);
    vis.origin_xyz = com;
    vis.color = color;
    link.visual_shapes.push_back(vis);
    def.links.push_back(link);
}

void add_capsule_link(core::RobotDef& def, const std::string& name,
                      double mass, double radius, double length,
                      const Eigen::Vector4d& color,
                      const Eigen::Vector3d& com = Eigen::Vector3d::Zero()) {
    core::LinkDef link;
    link.name = name;
    link.inertial.mass = mass;
    link.inertial.inertia = cylinder_inertia(mass, radius, length);
    link.inertial.com = com;
    core::CollisionShape col;
    col.type = core::ShapeType::Capsule;
    col.dimensions = Eigen::Vector3d(radius, 0, length);
    col.origin_xyz = com;
    link.collision_shapes.push_back(col);
    core::VisualShape vis;
    vis.type = core::ShapeType::Capsule;
    vis.dimensions = Eigen::Vector3d(radius, 0, length);
    vis.origin_xyz = com;
    vis.color = color;
    link.visual_shapes.push_back(vis);
    def.links.push_back(link);
}

void add_revolute_joint(core::RobotDef& def, const std::string& name,
                        const std::string& parent, const std::string& child,
                        const Eigen::Vector3d& axis, const Eigen::Vector3d& origin,
                        double lower, double upper, double max_effort) {
    core::JointDef j;
    j.name = name;
    j.type = core::JointType::Revolute;
    j.parent_link = parent;
    j.child_link = child;
    j.axis = axis;
    j.origin_xyz = origin;
    j.lower_limit = lower;
    j.upper_limit = upper;
    j.max_effort = max_effort;
    j.damping = 0.5;
    def.joints.push_back(j);
}

void add_fixed_joint(core::RobotDef& def, const std::string& name,
                     const std::string& parent, const std::string& child,
                     const Eigen::Vector3d& origin) {
    core::JointDef j;
    j.name = name;
    j.type = core::JointType::Fixed;
    j.parent_link = parent;
    j.child_link = child;
    j.origin_xyz = origin;
    def.joints.push_back(j);
}

core::ActuatorParams make_pd_actuator(double kp, double kd, double max_torque) {
    core::ActuatorParams a;
    a.type = core::ActuatorType::PositionPD;
    a.kp = kp;
    a.kd = kd;
    a.max_torque = max_torque;
    return a;
}

void add_leg(core::RobotDef& def, const HumanoidParams& p,
             const std::string& side, double y_sign,
             std::vector<double>& default_q) {
    std::string S = side;
    double y_off = y_sign * p.pelvis_width / 2.0;

    // Upper leg
    add_capsule_link(def, S + "_upper_leg", p.upper_leg_mass,
                     p.upper_leg_radius, p.upper_leg_length, p.limb_color,
                     Eigen::Vector3d(0, 0, -p.upper_leg_length / 2.0));

    // Lower leg
    add_capsule_link(def, S + "_lower_leg", p.lower_leg_mass,
                     p.lower_leg_radius, p.lower_leg_length, p.limb_color,
                     Eigen::Vector3d(0, 0, -p.lower_leg_length / 2.0));

    // Foot
    add_box_link(def, S + "_foot", p.foot_mass,
                 p.foot_length, p.foot_width, p.foot_height, p.limb_color,
                 Eigen::Vector3d(p.foot_length * 0.2, 0, -p.foot_height / 2.0));

    // Hip flexion (pelvis -> upper_leg)
    add_revolute_joint(def, S + "_hip_flexion", "pelvis", S + "_upper_leg",
                       Eigen::Vector3d::UnitY(),
                       Eigen::Vector3d(0, y_off, -p.pelvis_height / 2.0),
                       -p.hip_flexion_range / 2.0, p.hip_flexion_range / 2.0,
                       p.leg_torque);
    def.actuators.push_back(make_pd_actuator(p.kp, p.kd, p.leg_torque));
    default_q.push_back(0.0);

    // Hip abduction
    add_revolute_joint(def, S + "_hip_abduction", S + "_upper_leg", S + "_upper_leg",
                       Eigen::Vector3d::UnitX(),
                       Eigen::Vector3d::Zero(),
                       -p.hip_abduction_range, p.hip_abduction_range,
                       p.leg_torque);
    // Note: this creates a virtual joint on same link - in real impl
    // we'd use a 3-DOF ball joint. For simplicity we chain revolutes.

    // Knee (upper_leg -> lower_leg)
    add_revolute_joint(def, S + "_knee", S + "_upper_leg", S + "_lower_leg",
                       Eigen::Vector3d::UnitY(),
                       Eigen::Vector3d(0, 0, -p.upper_leg_length),
                       -p.knee_range, -0.05, p.leg_torque);
    def.actuators.push_back(make_pd_actuator(p.kp, p.kd, p.leg_torque));
    default_q.push_back(-0.4); // slight knee bend

    // Ankle (lower_leg -> foot)
    add_revolute_joint(def, S + "_ankle", S + "_lower_leg", S + "_foot",
                       Eigen::Vector3d::UnitY(),
                       Eigen::Vector3d(0, 0, -p.lower_leg_length),
                       -p.ankle_range, p.ankle_range, p.leg_torque * 0.5);
    def.actuators.push_back(make_pd_actuator(p.kp * 0.5, p.kd * 0.5, p.leg_torque * 0.5));
    default_q.push_back(0.0);
}

void add_arm(core::RobotDef& def, const HumanoidParams& p,
             const std::string& side, double y_sign,
             std::vector<double>& default_q) {
    std::string S = side;
    double y_off = y_sign * (p.torso_width / 2.0 + 0.02);

    // Upper arm
    add_capsule_link(def, S + "_upper_arm", p.upper_arm_mass,
                     p.upper_arm_radius, p.upper_arm_length, p.limb_color,
                     Eigen::Vector3d(0, 0, -p.upper_arm_length / 2.0));

    // Lower arm
    add_capsule_link(def, S + "_lower_arm", p.lower_arm_mass,
                     p.lower_arm_radius, p.lower_arm_length, p.limb_color,
                     Eigen::Vector3d(0, 0, -p.lower_arm_length / 2.0));

    // Shoulder flexion (torso -> upper_arm)
    add_revolute_joint(def, S + "_shoulder_flexion", "torso", S + "_upper_arm",
                       Eigen::Vector3d::UnitY(),
                       Eigen::Vector3d(0, y_off, p.torso_height / 2.0 - 0.05),
                       -p.shoulder_flexion_range / 2.0, p.shoulder_flexion_range / 2.0,
                       p.arm_torque);
    def.actuators.push_back(make_pd_actuator(p.kp * 0.5, p.kd * 0.5, p.arm_torque));
    default_q.push_back(0.0);

    // Elbow (upper_arm -> lower_arm)
    add_revolute_joint(def, S + "_elbow", S + "_upper_arm", S + "_lower_arm",
                       Eigen::Vector3d::UnitY(),
                       Eigen::Vector3d(0, 0, -p.upper_arm_length),
                       -p.elbow_range, 0.0, p.arm_torque);
    def.actuators.push_back(make_pd_actuator(p.kp * 0.4, p.kd * 0.4, p.arm_torque));
    default_q.push_back(-0.3); // slightly bent

    // Hand (optional)
    if (p.has_hands) {
        add_box_link(def, S + "_hand", p.hand_mass,
                     p.hand_length, p.hand_width, p.hand_height, p.limb_color);
        add_fixed_joint(def, S + "_wrist", S + "_lower_arm", S + "_hand",
                        Eigen::Vector3d(0, 0, -p.lower_arm_length));
    }
}

} // anonymous namespace

core::RobotDef build_humanoid(const HumanoidParams& p) {
    core::RobotDef def;
    def.name = p.name;
    def.description = p.description;
    def.category = core::RobotCategory::Humanoid;

    std::vector<double> default_q;

    // Pelvis (base link)
    add_box_link(def, "pelvis", p.pelvis_mass,
                 p.pelvis_depth, p.pelvis_width, p.pelvis_height, p.body_color);

    // Torso
    add_box_link(def, "torso", p.torso_mass,
                 p.torso_depth, p.torso_width, p.torso_height, p.body_color,
                 Eigen::Vector3d(0, 0, p.torso_height / 2.0));

    // Pelvis -> Torso (waist joint)
    add_revolute_joint(def, "waist", "pelvis", "torso",
                       Eigen::Vector3d::UnitY(),
                       Eigen::Vector3d(0, 0, p.pelvis_height / 2.0),
                       -0.5, 0.5, p.leg_torque);
    def.actuators.push_back(make_pd_actuator(p.kp, p.kd, p.leg_torque));
    default_q.push_back(0.0);

    // Head
    {
        core::LinkDef head;
        head.name = "head";
        head.inertial.mass = p.head_mass;
        head.inertial.inertia = sphere_inertia(p.head_mass, p.head_radius);
        core::CollisionShape col;
        col.type = core::ShapeType::Sphere;
        col.dimensions = Eigen::Vector3d(p.head_radius, 0, 0);
        col.origin_xyz = Eigen::Vector3d(0, 0, p.head_radius);
        head.collision_shapes.push_back(col);
        core::VisualShape vis;
        vis.type = core::ShapeType::Sphere;
        vis.dimensions = col.dimensions;
        vis.origin_xyz = col.origin_xyz;
        vis.color = p.body_color;
        head.visual_shapes.push_back(vis);
        def.links.push_back(head);
    }
    add_fixed_joint(def, "neck", "torso", "head",
                    Eigen::Vector3d(0, 0, p.torso_height));

    // Legs
    add_leg(def, p, "L", 1.0, default_q);
    add_leg(def, p, "R", -1.0, default_q);

    // Arms
    if (p.has_arms) {
        add_arm(def, p, "L", 1.0, default_q);
        add_arm(def, p, "R", -1.0, default_q);
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
