#pragma once

#include "robosim/terrain/terrain.h"
#include <cmath>

namespace robosim::terrain {

class FlatTerrain : public Terrain {
public:
    Heightfield generate(std::mt19937&) const override {
        Heightfield hf(rows_, cols_, resolution_);
        return hf; // all zeros
    }
    std::string name() const override { return "flat"; }
private:
    int rows_ = 200, cols_ = 200;
    double resolution_ = 0.05;
};

class StairsTerrain : public Terrain {
public:
    Heightfield generate(std::mt19937&) const override {
        Heightfield hf(rows_, cols_, resolution_);
        int steps_per_stair = static_cast<int>(step_depth_ / resolution_);
        if (steps_per_stair < 1) steps_per_stair = 1;
        for (int r = 0; r < rows_; r++) {
            int stair_idx = r / steps_per_stair;
            double height = stair_idx * step_height_;
            for (int c = 0; c < cols_; c++) {
                hf.at(r, c) = height;
            }
        }
        return hf;
    }
    std::string name() const override { return "stairs"; }
    void set_step_height(double h) { step_height_ = h; }
    void set_step_depth(double d) { step_depth_ = d; }
private:
    double step_height_ = 0.15;
    double step_depth_ = 0.30;
    int rows_ = 400, cols_ = 200;
    double resolution_ = 0.02;
};

class SlopeTerrain : public Terrain {
public:
    Heightfield generate(std::mt19937&) const override {
        Heightfield hf(rows_, cols_, resolution_);
        double slope = std::tan(angle_rad_);
        for (int r = 0; r < rows_; r++) {
            double height = r * resolution_ * slope;
            for (int c = 0; c < cols_; c++) {
                hf.at(r, c) = height;
            }
        }
        return hf;
    }
    std::string name() const override { return "slope"; }
    void set_angle_deg(double deg) { angle_rad_ = deg * M_PI / 180.0; }
private:
    double angle_rad_ = 10.0 * M_PI / 180.0;
    int rows_ = 400, cols_ = 200;
    double resolution_ = 0.02;
};

class RoughTerrain : public Terrain {
public:
    Heightfield generate(std::mt19937& rng) const override {
        Heightfield hf(rows_, cols_, resolution_);
        // Simple Perlin-like noise using multiple octaves of random values
        std::uniform_real_distribution<double> dist(-1.0, 1.0);

        // Generate base random grid at coarse resolution
        int coarse_r = rows_ / 8 + 2;
        int coarse_c = cols_ / 8 + 2;
        std::vector<double> coarse(coarse_r * coarse_c);
        for (auto& v : coarse) v = dist(rng) * amplitude_;

        // Bilinear upsample
        for (int r = 0; r < rows_; r++) {
            for (int c = 0; c < cols_; c++) {
                double fr = r * (coarse_r - 1.0) / rows_;
                double fc = c * (coarse_c - 1.0) / cols_;
                int r0 = static_cast<int>(fr);
                int c0 = static_cast<int>(fc);
                int r1 = std::min(r0 + 1, coarse_r - 1);
                int c1 = std::min(c0 + 1, coarse_c - 1);
                double tr = fr - r0;
                double tc = fc - c0;

                double v00 = coarse[r0 * coarse_c + c0];
                double v01 = coarse[r0 * coarse_c + c1];
                double v10 = coarse[r1 * coarse_c + c0];
                double v11 = coarse[r1 * coarse_c + c1];
                hf.at(r, c) = (1-tr)*(1-tc)*v00 + (1-tr)*tc*v01 +
                               tr*(1-tc)*v10 + tr*tc*v11;
            }
        }

        // Add fine noise
        std::normal_distribution<double> fine_noise(0, amplitude_ * 0.2);
        for (int r = 0; r < rows_; r++) {
            for (int c = 0; c < cols_; c++) {
                hf.at(r, c) += fine_noise(rng);
            }
        }
        return hf;
    }
    std::string name() const override { return "rough"; }
    void set_amplitude(double a) { amplitude_ = a; }
private:
    double amplitude_ = 0.08;
    int rows_ = 400, cols_ = 400;
    double resolution_ = 0.02;
};

class SteppingStoneTerrain : public Terrain {
public:
    Heightfield generate(std::mt19937& rng) const override {
        Heightfield hf(rows_, cols_, resolution_);
        // Start with a gap (negative height = void, approximated as very low)
        for (int r = 0; r < rows_; r++) {
            for (int c = 0; c < cols_; c++) {
                hf.at(r, c) = -1.0; // "void"
            }
        }
        // Place stepping stones in a grid with some randomness
        std::uniform_real_distribution<double> offset_dist(-0.02, 0.02);
        std::uniform_real_distribution<double> height_dist(0.0, 0.1);

        int stone_cells = static_cast<int>(stone_size_ / resolution_);
        int gap_cells = static_cast<int>(gap_size_ / resolution_);
        int stride = stone_cells + gap_cells;

        for (int sr = 0; sr < rows_; sr += stride) {
            for (int sc = 0; sc < cols_; sc += stride) {
                double h = height_dist(rng);
                for (int r = sr; r < std::min(sr + stone_cells, rows_); r++) {
                    for (int c = sc; c < std::min(sc + stone_cells, cols_); c++) {
                        hf.at(r, c) = h + offset_dist(rng);
                    }
                }
            }
        }
        return hf;
    }
    std::string name() const override { return "stepping_stones"; }
private:
    double stone_size_ = 0.3;
    double gap_size_ = 0.15;
    int rows_ = 400, cols_ = 400;
    double resolution_ = 0.02;
};

} // namespace robosim::terrain
