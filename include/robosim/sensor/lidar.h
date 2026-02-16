#pragma once

#include "robosim/sensor/sensor.h"
#include "robosim/core/world.h"

namespace robosim::sensor {

class LiDAR : public Sensor {
public:
    LiDAR(const core::World* world = nullptr, int num_rays = 360,
           double min_range = 0.1, double max_range = 10.0,
           double fov_horizontal = 2.0 * M_PI, double fov_vertical = 0.0)
        : world_(world), num_rays_(num_rays),
          min_range_(min_range), max_range_(max_range),
          fov_h_(fov_horizontal), fov_v_(fov_vertical) {}

    std::string name() const override { return "lidar"; }
    int data_dim() const override { return num_rays_; }

    Eigen::VectorXd read(std::mt19937& rng) const override {
        Eigen::VectorXd ranges(num_rays_);
        ranges.setConstant(max_range_);
        if (!world_) return ranges;

        for (int i = 0; i < num_rays_; i++) {
            double angle = (num_rays_ > 1) ? (-fov_h_ / 2.0 + fov_h_ * i / (num_rays_ - 1)) : 0.0;
            Eigen::Vector3d dir(std::cos(angle), std::sin(angle), 0);
            // Transform by sensor pose
            dir = sensor_orientation_ * dir;
            Eigen::Vector3d from = sensor_position_ + dir * min_range_;
            Eigen::Vector3d to = sensor_position_ + dir * max_range_;
            Eigen::Vector3d hit, normal;
            if (world_->raycast(from, to, hit, normal)) {
                ranges(i) = add_noise((hit - sensor_position_).norm(), rng);
            }
        }
        return ranges;
    }

    void update(double) override {}
    void reset() override {}

    void set_world(const core::World* w) { world_ = w; }
    void set_pose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& ori) {
        sensor_position_ = pos;
        sensor_orientation_ = ori;
    }

private:
    const core::World* world_ = nullptr;
    int num_rays_;
    double min_range_, max_range_;
    double fov_h_, fov_v_;
    Eigen::Vector3d sensor_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond sensor_orientation_ = Eigen::Quaterniond::Identity();
};

} // namespace robosim::sensor
