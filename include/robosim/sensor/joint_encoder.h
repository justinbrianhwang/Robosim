#pragma once

#include "robosim/sensor/sensor.h"
#include "robosim/core/robot.h"

namespace robosim::sensor {

class JointEncoder : public Sensor {
public:
    explicit JointEncoder(const core::Robot* robot = nullptr) : robot_(robot) {}

    std::string name() const override { return "joint_encoder"; }
    int data_dim() const override { return robot_ ? robot_->num_joints() * 2 : 0; }

    Eigen::VectorXd read(std::mt19937& rng) const override {
        if (!robot_) return Eigen::VectorXd();
        int n = robot_->num_joints();
        Eigen::VectorXd data(n * 2);
        for (int i = 0; i < n; i++) {
            const auto& s = robot_->joint_state(i);
            data(i) = add_noise(s.position, rng);
            data(n + i) = add_noise(s.velocity, rng);
        }
        return data;
    }

    void update(double) override {}
    void reset() override {}
    void set_robot(const core::Robot* robot) { robot_ = robot; }

private:
    const core::Robot* robot_ = nullptr;
};

} // namespace robosim::sensor
