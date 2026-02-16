#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <memory>

namespace robosim::ai {

class OnnxInference {
public:
    OnnxInference() = default;
    ~OnnxInference();

    bool load_model(const std::string& model_path);
    Eigen::VectorXd infer(const Eigen::VectorXd& input) const;
    bool is_loaded() const { return loaded_; }

    // Model info
    int input_dim() const { return input_dim_; }
    int output_dim() const { return output_dim_; }
    std::string model_path() const { return model_path_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    bool loaded_ = false;
    int input_dim_ = 0;
    int output_dim_ = 0;
    std::string model_path_;
};

} // namespace robosim::ai
