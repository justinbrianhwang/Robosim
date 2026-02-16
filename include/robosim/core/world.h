#pragma once

#include "robosim/core/robot.h"
#include "robosim/core/contact.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace robosim::core {

struct WorldConfig {
    double timestep = 0.002;         // 500Hz physics
    Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.81);
    int max_substeps = 4;
    double ground_friction = 1.0;
    double ground_restitution = 0.0;
};

class World {
public:
    World() = default;
    explicit World(const WorldConfig& config);
    virtual ~World() = default;

    virtual void initialize() = 0;
    virtual void step() = 0;
    virtual void add_robot(std::shared_ptr<Robot> robot) = 0;
    virtual void remove_robot(const std::string& name) = 0;
    virtual void set_terrain_heightfield(const std::vector<float>& heights,
                                          int rows, int cols,
                                          double resolution,
                                          double min_h, double max_h) = 0;
    virtual std::vector<ContactInfo> get_contacts() const = 0;

    // Raycast for LiDAR
    virtual bool raycast(const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                         Eigen::Vector3d& hit_point, Eigen::Vector3d& hit_normal) const = 0;

    const WorldConfig& config() const { return config_; }
    double simulation_time() const { return sim_time_; }
    void set_gravity(const Eigen::Vector3d& g) { config_.gravity = g; }
    void set_ground_friction(double f) { config_.ground_friction = f; }

protected:
    WorldConfig config_;
    double sim_time_ = 0.0;
    std::vector<std::shared_ptr<Robot>> robots_;
};

} // namespace robosim::core
