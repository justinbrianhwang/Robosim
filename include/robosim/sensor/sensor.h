#pragma once

#include <Eigen/Dense>
#include <string>
#include <random>

namespace robosim::sensor {

class Sensor {
public:
    virtual ~Sensor() = default;
    virtual std::string name() const = 0;
    virtual int data_dim() const = 0;
    virtual Eigen::VectorXd read(std::mt19937& rng) const = 0;
    virtual void update(double dt) = 0;
    virtual void reset() = 0;

    void set_noise_std(double std) { noise_std_ = std; }
    double noise_std() const { return noise_std_; }

protected:
    double add_noise(double val, std::mt19937& rng) const {
        if (noise_std_ <= 0.0) return val;
        std::normal_distribution<double> dist(0.0, noise_std_);
        return val + dist(rng);
    }

    Eigen::VectorXd add_noise_vec(const Eigen::VectorXd& v, std::mt19937& rng) const {
        if (noise_std_ <= 0.0) return v;
        Eigen::VectorXd result(v.size());
        std::normal_distribution<double> dist(0.0, noise_std_);
        for (int i = 0; i < v.size(); i++) {
            result(i) = v(i) + dist(rng);
        }
        return result;
    }

    double noise_std_ = 0.0;
};

} // namespace robosim::sensor
