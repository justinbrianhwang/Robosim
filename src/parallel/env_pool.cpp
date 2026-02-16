#include "robosim/parallel/env_pool.h"
#include <spdlog/spdlog.h>

namespace robosim::parallel {

EnvPool::EnvPool(const std::string& robot_name, const std::string& task_name,
                  int num_envs, int num_threads)
    : num_envs_(num_envs) {
    pool_ = std::make_unique<ThreadPool>(num_threads);

    envs_.reserve(num_envs);
    for (int i = 0; i < num_envs; i++) {
        envs_.push_back(std::make_unique<env::Environment>(robot_name, task_name, true));
    }
    spdlog::info("EnvPool created: {} envs, {} threads", num_envs, pool_->size());
}

EnvPool::~EnvPool() = default;

BatchedState EnvPool::reset() {
    int obs_dim = observation_dim();
    BatchedState state;
    state.observations.resize(num_envs_, obs_dim);
    state.rewards = Eigen::VectorXd::Zero(num_envs_);
    state.dones = Eigen::VectorXi::Zero(num_envs_);
    state.truncateds = Eigen::VectorXi::Zero(num_envs_);

    std::vector<std::future<env::ResetResult>> futures;
    for (int i = 0; i < num_envs_; i++) {
        futures.push_back(pool_->submit([this, i]() {
            return envs_[i]->reset(static_cast<unsigned int>(i));
        }));
    }

    for (int i = 0; i < num_envs_; i++) {
        auto result = futures[i].get();
        state.observations.row(i) = result.observation.transpose();
    }
    return state;
}

BatchedState EnvPool::step(const Eigen::MatrixXd& actions) {
    int obs_dim = observation_dim();
    BatchedState state;
    state.observations.resize(num_envs_, obs_dim);
    state.rewards.resize(num_envs_);
    state.dones.resize(num_envs_);
    state.truncateds.resize(num_envs_);

    std::vector<std::future<env::StepResult>> futures;
    for (int i = 0; i < num_envs_; i++) {
        Eigen::VectorXd action = actions.row(i).transpose();
        futures.push_back(pool_->submit([this, i, action]() {
            auto result = envs_[i]->step(action);
            // Auto-reset on episode end
            if (result.terminated || result.truncated) {
                auto reset_result = envs_[i]->reset();
                result.observation = reset_result.observation;
            }
            return result;
        }));
    }

    for (int i = 0; i < num_envs_; i++) {
        auto result = futures[i].get();
        state.observations.row(i) = result.observation.transpose();
        state.rewards(i) = result.reward;
        state.dones(i) = result.terminated ? 1 : 0;
        state.truncateds(i) = result.truncated ? 1 : 0;
    }
    return state;
}

int EnvPool::observation_dim() const {
    return envs_.empty() ? 0 : envs_[0]->observation_dim();
}

int EnvPool::action_dim() const {
    return envs_.empty() ? 0 : envs_[0]->action_dim();
}

} // namespace robosim::parallel
