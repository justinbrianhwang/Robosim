#pragma once

#include "robosim/parallel/thread_pool.h"
#include "robosim/env/environment.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace robosim::parallel {

struct BatchedState {
    Eigen::MatrixXd observations;  // [num_envs x obs_dim]
    Eigen::VectorXd rewards;       // [num_envs]
    Eigen::VectorXi dones;         // [num_envs]
    Eigen::VectorXi truncateds;    // [num_envs]
};

class EnvPool {
public:
    EnvPool(const std::string& robot_name, const std::string& task_name,
            int num_envs, int num_threads = 0);
    ~EnvPool();

    BatchedState reset();
    BatchedState step(const Eigen::MatrixXd& actions);

    int num_envs() const { return num_envs_; }
    int observation_dim() const;
    int action_dim() const;

    env::Environment& env(int idx) { return *envs_[idx]; }
    const env::Environment& env(int idx) const { return *envs_[idx]; }

private:
    int num_envs_;
    std::vector<std::unique_ptr<env::Environment>> envs_;
    std::unique_ptr<ThreadPool> pool_;
};

} // namespace robosim::parallel
