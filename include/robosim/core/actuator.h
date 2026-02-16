#pragma once

#include <string>
#include <cmath>
#include <algorithm>

namespace robosim::core {

enum class ActuatorType {
    PositionPD,    // PD position controller
    VelocityPI,    // PI velocity controller
    Torque,        // Direct torque control
    SEA,           // Series Elastic Actuator
    Motor          // DC motor model
};

struct ActuatorParams {
    ActuatorType type = ActuatorType::PositionPD;
    double kp = 50.0;           // Position gain
    double kd = 2.0;            // Derivative gain
    double ki = 0.0;            // Integral gain
    double max_torque = 100.0;  // Nm
    double gear_ratio = 1.0;
    double motor_inertia = 0.0;
    double friction = 0.0;
    double damping = 0.0;
    // Motor model params
    double kt = 0.0;           // Torque constant
    double resistance = 0.0;
    double max_voltage = 0.0;
    // SEA params
    double spring_stiffness = 0.0;
};

class Actuator {
public:
    Actuator() = default;
    explicit Actuator(const ActuatorParams& params) : params_(params) {}

    double compute_torque(double pos, double vel, double target_pos,
                          double target_vel, double dt) {
        double error = target_pos - pos;
        double derror = target_vel - vel;

        switch (params_.type) {
        case ActuatorType::PositionPD: {
            double torque = params_.kp * error + params_.kd * derror;
            integral_error_ += error * dt;
            torque += params_.ki * integral_error_;
            return clamp_torque(torque);
        }
        case ActuatorType::VelocityPI: {
            double vel_error = target_vel - vel;
            integral_error_ += vel_error * dt;
            double torque = params_.kp * vel_error + params_.ki * integral_error_;
            return clamp_torque(torque);
        }
        case ActuatorType::Torque:
            return clamp_torque(target_pos); // target_pos used as torque command
        case ActuatorType::Motor: {
            double back_emf = params_.kt * vel * params_.gear_ratio;
            double voltage = std::clamp(target_pos * params_.max_voltage, -params_.max_voltage, params_.max_voltage);
            double current = (voltage - back_emf) / std::max(params_.resistance, 1e-6);
            double motor_torque = params_.kt * current;
            return clamp_torque(motor_torque * params_.gear_ratio);
        }
        case ActuatorType::SEA: {
            double spring_torque = params_.spring_stiffness * error;
            double damping_torque = params_.damping * derror;
            return clamp_torque(spring_torque + damping_torque);
        }
        default:
            return 0.0;
        }
    }

    void reset() { integral_error_ = 0.0; }
    const ActuatorParams& params() const { return params_; }
    void set_params(const ActuatorParams& p) { params_ = p; }

private:
    double clamp_torque(double t) const {
        return std::clamp(t, -params_.max_torque, params_.max_torque);
    }

    ActuatorParams params_;
    double integral_error_ = 0.0;
};

} // namespace robosim::core
