#pragma once

#include "robosim/sensor/sensor.h"

namespace robosim::sensor {

class IMU : public Sensor {
public:
    std::string name() const override { return "imu"; }
    int data_dim() const override { return 10; } // quat(4) + gyro(3) + accel(3)

    Eigen::VectorXd read(std::mt19937& rng) const override {
        Eigen::VectorXd data(10);
        // Quaternion (w,x,y,z)
        data(0) = orientation_.w();
        data(1) = orientation_.x();
        data(2) = orientation_.y();
        data(3) = orientation_.z();
        // Gyroscope (body frame angular velocity)
        Eigen::Vector3d gyro = add_noise_vec(angular_velocity_, rng);
        data.segment<3>(4) = gyro + gyro_bias_;
        // Accelerometer (body frame linear acceleration + gravity)
        Eigen::Vector3d accel = add_noise_vec(linear_acceleration_, rng);
        data.segment<3>(7) = accel + accel_bias_;
        return data;
    }

    void update(double /*dt*/) override {}
    void reset() override {
        orientation_ = Eigen::Quaterniond::Identity();
        angular_velocity_.setZero();
        linear_acceleration_ = Eigen::Vector3d(0, 0, 9.81);
        gyro_bias_.setZero();
        accel_bias_.setZero();
    }

    void set_state(const Eigen::Quaterniond& ori,
                   const Eigen::Vector3d& ang_vel,
                   const Eigen::Vector3d& lin_acc) {
        orientation_ = ori;
        angular_velocity_ = ang_vel;
        linear_acceleration_ = lin_acc;
    }

    void set_gyro_bias(const Eigen::Vector3d& b) { gyro_bias_ = b; }
    void set_accel_bias(const Eigen::Vector3d& b) { accel_bias_ = b; }

private:
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_acceleration_ = Eigen::Vector3d(0, 0, 9.81);
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
};

} // namespace robosim::sensor
