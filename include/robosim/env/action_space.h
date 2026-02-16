#pragma once

#include <Eigen/Dense>

namespace robosim::env {

struct ActionSpace {
    Eigen::VectorXd low;
    Eigen::VectorXd high;
    int dim() const { return static_cast<int>(low.size()); }

    static ActionSpace create(int dim, double low_val = -1.0, double high_val = 1.0) {
        ActionSpace s;
        s.low = Eigen::VectorXd::Constant(dim, low_val);
        s.high = Eigen::VectorXd::Constant(dim, high_val);
        return s;
    }

    // Clip action to bounds
    Eigen::VectorXd clip(const Eigen::VectorXd& action) const {
        return action.cwiseMax(low).cwiseMin(high);
    }

    // Scale from [-1,1] to [low,high]
    Eigen::VectorXd scale(const Eigen::VectorXd& normalized_action) const {
        return (low.array() + (normalized_action.array() + 1.0) * 0.5 * (high - low).array()).matrix();
    }
};

} // namespace robosim::env
