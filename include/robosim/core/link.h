#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace robosim::core {

enum class ShapeType {
    Box,
    Cylinder,
    Sphere,
    Capsule,
    Mesh
};

struct CollisionShape {
    ShapeType type = ShapeType::Box;
    Eigen::Vector3d dimensions = Eigen::Vector3d(0.1, 0.1, 0.1); // box: xyz, cyl: r,0,h, sphere: r,0,0, capsule: r,0,h
    Eigen::Vector3d origin_xyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d origin_rpy = Eigen::Vector3d::Zero();
    std::string mesh_path; // for Mesh type
};

struct VisualShape {
    ShapeType type = ShapeType::Box;
    Eigen::Vector3d dimensions = Eigen::Vector3d(0.1, 0.1, 0.1);
    Eigen::Vector3d origin_xyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d origin_rpy = Eigen::Vector3d::Zero();
    Eigen::Vector4d color = Eigen::Vector4d(0.7, 0.7, 0.7, 1.0); // RGBA
    std::string mesh_path;
};

struct InertialProperties {
    double mass = 1.0;
    Eigen::Vector3d com = Eigen::Vector3d::Zero(); // center of mass offset
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity() * 0.001;
};

struct LinkDef {
    std::string name;
    InertialProperties inertial;
    std::vector<CollisionShape> collision_shapes;
    std::vector<VisualShape> visual_shapes;
};

} // namespace robosim::core
