#pragma once

#include <Eigen/Dense>
#include <string>

namespace robosim::env {

struct BoxSpace {
    Eigen::VectorXd low;
    Eigen::VectorXd high;
    int dim() const { return static_cast<int>(low.size()); }

    static BoxSpace create(int dim, double low_val = -1e6, double high_val = 1e6) {
        BoxSpace s;
        s.low = Eigen::VectorXd::Constant(dim, low_val);
        s.high = Eigen::VectorXd::Constant(dim, high_val);
        return s;
    }
};

} // namespace robosim::env
