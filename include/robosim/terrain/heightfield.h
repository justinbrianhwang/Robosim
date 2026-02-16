#pragma once

#include <Eigen/Dense>
#include <vector>

namespace robosim::terrain {

class Heightfield {
public:
    Heightfield() = default;
    Heightfield(int rows, int cols, double resolution);

    double& at(int row, int col) { return data_[row * cols_ + col]; }
    double at(int row, int col) const { return data_[row * cols_ + col]; }

    // Bilinear interpolation at world coordinates
    double height_at(double x, double y) const;
    Eigen::Vector3d normal_at(double x, double y) const;

    int rows() const { return rows_; }
    int cols() const { return cols_; }
    double resolution() const { return resolution_; }
    double width() const { return cols_ * resolution_; }
    double length() const { return rows_ * resolution_; }

    // For physics engine upload
    const std::vector<double>& data() const { return data_; }
    std::vector<float> data_float() const;
    double min_height() const;
    double max_height() const;

    // Center the heightfield origin
    double origin_x() const { return -width() / 2.0; }
    double origin_y() const { return -length() / 2.0; }

private:
    int rows_ = 0, cols_ = 0;
    double resolution_ = 0.02;
    std::vector<double> data_;
};

} // namespace robosim::terrain
