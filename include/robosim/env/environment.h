#pragma once

#include "robosim/core/simulator.h"
#include "robosim/core/robot.h"
#include "robosim/task/task.h"
#include "robosim/terrain/terrain.h"
#include "robosim/terrain/heightfield.h"
#include "robosim/reward/reward_function.h"
#include "robosim/reward/composite_reward.h"
#include "robosim/sensor/imu.h"
#include "robosim/sensor/joint_encoder.h"
#include "robosim/sensor/contact_sensor.h"
#include "robosim/env/observation_space.h"
#include "robosim/env/action_space.h"
#include <memory>
#include <random>
#include <tuple>
#include <unordered_map>

namespace robosim::env {

struct StepResult {
    Eigen::VectorXd observation;
    double reward;
    bool terminated;
    bool truncated;
    std::unordered_map<std::string, double> info;
};

struct ResetResult {
    Eigen::VectorXd observation;
    std::unordered_map<std::string, double> info;
};

class Environment {
public:
    Environment() = default;
    Environment(const std::string& robot_name, const std::string& task_name,
                bool headless = true);
    ~Environment() = default;

    // Gymnasium-compatible interface
    ResetResult reset(std::optional<unsigned int> seed = std::nullopt);
    StepResult step(const Eigen::VectorXd& action);
    void close();

    // Space info
    BoxSpace observation_space() const { return obs_space_; }
    ActionSpace action_space() const { return act_space_; }
    int observation_dim() const { return obs_space_.dim(); }
    int action_dim() const { return act_space_.dim(); }

    // Access
    core::Robot& robot() { return sim_.robot(); }
    const core::Robot& robot() const { return sim_.robot(); }
    core::World& world() { return sim_.world(); }
    task::Task& task() { return *task_; }

    // Configuration
    void set_terrain(std::unique_ptr<terrain::Terrain> terrain) { terrain_ = std::move(terrain); }
    void set_task(std::unique_ptr<task::Task> task);
    void set_control_dt(double dt) { control_dt_ = dt; }

    int step_count() const { return step_count_; }
    double simulation_time() const { return sim_.simulation_time(); }

private:
    Eigen::VectorXd build_observation();
    void compute_spaces();

    core::Simulator sim_;
    std::unique_ptr<task::Task> task_;
    std::unique_ptr<terrain::Terrain> terrain_;
    terrain::Heightfield heightfield_;

    // Sensors
    sensor::IMU imu_;
    sensor::JointEncoder joint_encoder_;
    sensor::ContactSensor contact_sensor_;

    // Spaces
    BoxSpace obs_space_;
    ActionSpace act_space_;

    // State
    int step_count_ = 0;
    Eigen::VectorXd previous_action_;
    double control_dt_ = 0.02; // 50Hz control
    std::mt19937 rng_;

    // History for observation
    Eigen::VectorXd last_action_;
};

} // namespace robosim::env
