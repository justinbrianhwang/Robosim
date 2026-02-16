#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <fstream>
#include <unordered_map>

namespace robosim::checkpoint {

struct StateSnapshot {
    double simulation_time = 0.0;
    int step_count = 0;
    unsigned int rng_seed = 0;

    // Robot base state
    Eigen::Vector3d base_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond base_orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d base_linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d base_angular_velocity = Eigen::Vector3d::Zero();

    // Joint state
    Eigen::VectorXd joint_positions;
    Eigen::VectorXd joint_velocities;

    // Task state
    std::string task_name;
    std::unordered_map<std::string, double> task_state;

    // Domain randomization params
    std::unordered_map<std::string, double> randomization_params;

    // Serialization
    bool save(const std::string& filepath) const;
    bool load(const std::string& filepath);
};

class CheckpointManager {
public:
    explicit CheckpointManager(const std::string& directory = "checkpoints");

    std::string save(const StateSnapshot& snapshot, const std::string& tag = "");
    bool load(StateSnapshot& snapshot, const std::string& filepath);
    bool load_latest(StateSnapshot& snapshot);
    std::vector<std::string> list_checkpoints() const;
    void set_max_checkpoints(int max) { max_checkpoints_ = max; }

private:
    std::string directory_;
    int max_checkpoints_ = 100;
    int counter_ = 0;
    void enforce_max();
};

} // namespace robosim::checkpoint
