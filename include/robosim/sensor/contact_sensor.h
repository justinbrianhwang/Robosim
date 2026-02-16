#pragma once

#include "robosim/sensor/sensor.h"
#include "robosim/core/robot.h"

namespace robosim::sensor {

class ContactSensor : public Sensor {
public:
    explicit ContactSensor(const core::Robot* robot = nullptr, int num_contact_points = 4)
        : robot_(robot), num_points_(num_contact_points) {}

    std::string name() const override { return "contact_sensor"; }
    int data_dim() const override { return num_points_; } // binary contact flags

    Eigen::VectorXd read(std::mt19937& rng) const override {
        Eigen::VectorXd data = Eigen::VectorXd::Zero(num_points_);
        if (!robot_) return data;
        const auto& contacts = robot_->contacts();
        // Map contacts to foot indices (simplified: mark as 1 if contact force > threshold)
        int idx = 0;
        for (const auto& c : contacts) {
            if (idx >= num_points_) break;
            if (c.normal_force > contact_threshold_) {
                data(idx) = add_noise(1.0, rng);
            }
            idx++;
        }
        return data;
    }

    void update(double) override {}
    void reset() override {}
    void set_robot(const core::Robot* robot) { robot_ = robot; }
    void set_contact_threshold(double t) { contact_threshold_ = t; }

private:
    const core::Robot* robot_ = nullptr;
    int num_points_ = 4;
    double contact_threshold_ = 1.0; // N
};

} // namespace robosim::sensor
