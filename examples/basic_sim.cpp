/**
 * RoboSim Basic Simulation Example
 *
 * Demonstrates how to:
 * 1. Create an environment with a specific robot and task
 * 2. Run a simulation loop with random actions
 * 3. Log rewards and episode statistics
 */

#include "robosim/env/environment.h"
#include "robosim/robot/robot_factory.h"
#include <spdlog/spdlog.h>
#include <iostream>
#include <numeric>

int main() {
    spdlog::set_level(spdlog::level::info);

    // Register all robots
    robosim::robot::RobotFactory::register_all_robots();

    // List available robots
    auto robots = robosim::robot::RobotFactory::instance().available_robots();
    std::cout << "Available robots:\n";
    for (const auto& r : robots) {
        std::cout << "  - " << r.id << " (" << r.display_name << ")\n";
    }

    // Create environment
    std::string robot_name = "unitree_go2";
    std::string task_name = "walk_forward";

    robosim::env::Environment env(robot_name, task_name, true);
    auto [obs, info] = env.reset(42);

    std::cout << "\nSimulation started: " << robot_name << "\n";
    std::cout << "Observation dim: " << env.observation_dim() << "\n";
    std::cout << "Action dim: " << env.action_dim() << "\n";

    // Simulation loop
    int num_episodes = 5;
    int max_steps = 1000;

    for (int ep = 0; ep < num_episodes; ep++) {
        auto [obs_reset, info_reset] = env.reset(ep);
        double episode_reward = 0.0;
        int steps = 0;

        for (int step = 0; step < max_steps; step++) {
            // Random action in [-0.5, 0.5]
            Eigen::VectorXd action = Eigen::VectorXd::Random(env.action_dim()) * 0.5;

            auto result = env.step(action);
            episode_reward += result.reward;
            steps++;

            if (result.terminated || result.truncated) break;
        }

        std::cout << "Episode " << ep + 1 << ": steps=" << steps
                  << ", reward=" << episode_reward << "\n";
    }

    std::cout << "\nSimulation complete.\n";
    return 0;
}
