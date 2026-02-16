#include "robosim/env/environment.h"
#include "robosim/task/tasks/all_tasks.h"
#include "robosim/terrain/terrains/all_terrains.h"
#include <spdlog/spdlog.h>

namespace robosim::env {

Environment::Environment(const std::string& robot_name, const std::string& task_name,
                          bool headless) {
    // Initialize simulator
    config::ConfigNode empty_cfg;
    sim_ = core::Simulator(empty_cfg);
    sim_.initialize(robot_name);

    // Initialize task
    if (task_name == "walk_forward") {
        task_ = std::make_unique<task::WalkForwardTask>();
    } else if (task_name == "follow_path") {
        task_ = std::make_unique<task::FollowPathTask>();
    } else if (task_name == "reach_target") {
        task_ = std::make_unique<task::ReachTargetTask>();
    } else if (task_name == "pick_and_place") {
        task_ = std::make_unique<task::PickAndPlaceTask>();
    } else {
        task_ = std::make_unique<task::WalkForwardTask>();
    }

    // Default terrain
    terrain_ = std::make_unique<terrain::FlatTerrain>();

    // Setup sensors
    joint_encoder_.set_robot(&sim_.robot());
    contact_sensor_.set_robot(&sim_.robot());

    // Compute observation/action spaces
    compute_spaces();

    spdlog::info("Environment created: robot='{}', task='{}'", robot_name, task_name);
}

void Environment::compute_spaces() {
    // Observation: base_ori(4) + base_ang_vel(3) + base_lin_vel(3) + joint_pos(N) + joint_vel(N) + last_action(N) + task_obs
    int n_joints = sim_.robot().num_joints();
    int task_dim = task_ ? task_->task_obs_dim() : 0;
    int obs_dim = 4 + 3 + 3 + n_joints + n_joints + n_joints + task_dim;
    obs_space_ = BoxSpace::create(obs_dim, -100.0, 100.0);

    // Action: one per joint
    act_space_ = ActionSpace::create(n_joints, -1.0, 1.0);

    previous_action_ = Eigen::VectorXd::Zero(n_joints);
    last_action_ = Eigen::VectorXd::Zero(n_joints);
}

Eigen::VectorXd Environment::build_observation() {
    auto& robot = sim_.robot();
    int n = robot.num_joints();
    int task_dim = task_ ? task_->task_obs_dim() : 0;
    int obs_dim = 4 + 3 + 3 + n + n + n + task_dim;
    Eigen::VectorXd obs(obs_dim);

    int idx = 0;

    // Base orientation (quaternion in body frame projected gravity)
    Eigen::Quaterniond ori = robot.base_orientation();
    Eigen::Vector3d gravity_body = ori.inverse() * Eigen::Vector3d(0, 0, -1);
    obs(idx++) = ori.w();
    obs(idx++) = ori.x();
    obs(idx++) = ori.y();
    obs(idx++) = ori.z();

    // Base angular velocity (body frame)
    Eigen::Vector3d ang_vel = robot.body_frame_rotation().transpose() * robot.base_angular_velocity();
    obs.segment<3>(idx) = ang_vel;
    idx += 3;

    // Base linear velocity (body frame)
    Eigen::Vector3d lin_vel = robot.body_frame_rotation().transpose() * robot.base_linear_velocity();
    obs.segment<3>(idx) = lin_vel;
    idx += 3;

    // Joint positions
    obs.segment(idx, n) = robot.joint_positions();
    idx += n;

    // Joint velocities
    obs.segment(idx, n) = robot.joint_velocities();
    idx += n;

    // Last action
    obs.segment(idx, n) = last_action_;
    idx += n;

    // Task-specific observation
    if (task_ && task_dim > 0) {
        obs.segment(idx, task_dim) = task_->get_task_observation(robot, sim_.world());
        idx += task_dim;
    }

    return obs;
}

ResetResult Environment::reset(std::optional<unsigned int> seed) {
    if (seed.has_value()) {
        rng_.seed(seed.value());
    }

    sim_.reset();
    step_count_ = 0;
    previous_action_ = Eigen::VectorXd::Zero(sim_.robot().num_joints());
    last_action_ = Eigen::VectorXd::Zero(sim_.robot().num_joints());

    // Generate and apply terrain
    if (terrain_) {
        heightfield_ = terrain_->generate(rng_);
        auto heights = heightfield_.data_float();
        if (!heights.empty()) {
            sim_.world().set_terrain_heightfield(
                heights, heightfield_.rows(), heightfield_.cols(),
                heightfield_.resolution(),
                heightfield_.min_height(), heightfield_.max_height());
        }
    }

    // Reset task
    if (task_) {
        task_->reset(rng_, sim_.world(), sim_.robot());
    }

    auto obs = build_observation();
    return {obs, {}};
}

StepResult Environment::step(const Eigen::VectorXd& action) {
    previous_action_ = last_action_;
    last_action_ = action;

    // Step simulation
    sim_.step(action);
    step_count_++;

    // Build reward context
    reward::RewardContext ctx;
    auto& robot = sim_.robot();
    ctx.base_position = robot.base_position();
    ctx.base_orientation = robot.base_orientation();
    ctx.base_linear_velocity = robot.base_linear_velocity();
    ctx.base_angular_velocity = robot.base_angular_velocity();
    ctx.joint_positions = robot.joint_positions();
    ctx.joint_velocities = robot.joint_velocities();
    ctx.joint_torques = robot.joint_torques();
    ctx.previous_actions = previous_action_;
    ctx.current_actions = action;
    ctx.contacts = robot.contacts();
    ctx.foot_contact_forces = robot.foot_contact_forces();
    ctx.simulation_time = sim_.simulation_time();
    ctx.step_count = step_count_;
    ctx.dt = sim_.timestep() * sim_.control_decimation();

    if (task_) {
        task_->populate_reward_context(ctx, robot, sim_.world());
    }

    // Compute reward
    double total_reward = 0.0;
    std::unordered_map<std::string, double> info;
    if (task_) {
        auto reward_info = task_->reward_function().compute(ctx);
        total_reward = reward_info.total_reward;
        for (const auto& [name, val] : reward_info.components) {
            info["reward/" + name] = val;
        }
    }

    // Check termination
    bool terminated = false, truncated = false;
    if (task_) {
        auto term = task_->check_termination(robot, sim_.world(), step_count_);
        terminated = term.terminated;
        truncated = term.truncated;
        if (!term.reason.empty()) {
            info["termination_reason_hash"] = static_cast<double>(std::hash<std::string>{}(term.reason) % 1000);
        }
    }

    auto obs = build_observation();
    info["step"] = static_cast<double>(step_count_);
    info["sim_time"] = sim_.simulation_time();

    return {obs, total_reward, terminated, truncated, info};
}

void Environment::close() {
    spdlog::info("Environment closed");
}

void Environment::set_task(std::unique_ptr<task::Task> task) {
    task_ = std::move(task);
    compute_spaces();
}

} // namespace robosim::env
