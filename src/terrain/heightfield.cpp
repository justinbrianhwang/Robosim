#include "robosim/terrain/heightfield.h"
#include <algorithm>
#include <cmath>

namespace robosim::terrain {

Heightfield::Heightfield(int rows, int cols, double resolution)
    : rows_(rows), cols_(cols), resolution_(resolution) {
    data_.resize(rows * cols, 0.0);
}

double Heightfield::height_at(double x, double y) const {
    if (rows_ == 0 || cols_ == 0) return 0.0;

    // Convert world coords to grid coords
    double gx = (x - origin_x()) / resolution_;
    double gy = (y - origin_y()) / resolution_;

    int c0 = static_cast<int>(std::floor(gx));
    int r0 = static_cast<int>(std::floor(gy));
    int c1 = c0 + 1;
    int r1 = r0 + 1;

    c0 = std::clamp(c0, 0, cols_ - 1);
    c1 = std::clamp(c1, 0, cols_ - 1);
    r0 = std::clamp(r0, 0, rows_ - 1);
    r1 = std::clamp(r1, 0, rows_ - 1);

    double tx = gx - std::floor(gx);
    double ty = gy - std::floor(gy);

    double h00 = data_[r0 * cols_ + c0];
    double h01 = data_[r0 * cols_ + c1];
    double h10 = data_[r1 * cols_ + c0];
    double h11 = data_[r1 * cols_ + c1];

    return (1 - ty) * ((1 - tx) * h00 + tx * h01) +
           ty * ((1 - tx) * h10 + tx * h11);
}

Eigen::Vector3d Heightfield::normal_at(double x, double y) const {
    double eps = resolution_ * 0.5;
    double hx0 = height_at(x - eps, y);
    double hx1 = height_at(x + eps, y);
    double hy0 = height_at(x, y - eps);
    double hy1 = height_at(x, y + eps);

    Eigen::Vector3d n(-(hx1 - hx0) / (2.0 * eps),
                       -(hy1 - hy0) / (2.0 * eps),
                       1.0);
    return n.normalized();
}

std::vector<float> Heightfield::data_float() const {
    std::vector<float> result(data_.size());
    for (size_t i = 0; i < data_.size(); i++) {
        result[i] = static_cast<float>(data_[i]);
    }
    return result;
}

double Heightfield::min_height() const {
    if (data_.empty()) return 0.0;
    auto [mi, ma] = std::minmax_element(data_.begin(), data_.end());
    return *mi;
}

double Heightfield::max_height() const {
    if (data_.empty()) return 0.0;
    auto [mi, ma] = std::minmax_element(data_.begin(), data_.end());
    return *ma;
}

} // namespace robosim::terrain
