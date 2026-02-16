#include "robosim/core/simulator.h"
#include "robosim/physics/bullet_world.h"
#include "robosim/robot/robot_factory.h"
#include <spdlog/spdlog.h>

namespace robosim::core {

Simulator::Simulator(const config::ConfigNode& cfg) {
    timestep_ = cfg.get<double>("timestep", 0.002);
    control_decimation_ = cfg.get<int>("control_decimation", 4);
}

void Simulator::initialize(const std::string& robot_name) {
    // Create physics world
    WorldConfig wconfig;
    wconfig.timestep = timestep_;
    world_ = std::make_shared<physics::BulletWorld>(wconfig);
    world_->initialize();

    // Create robot
    auto& factory = robot::RobotFactory::instance();
    auto robot_def = factory.create(robot_name);
    robot_ = std::make_shared<Robot>(robot_def);

    // Add robot to world
    world_->add_robot(robot_);

    spdlog::info("Simulator initialized: robot='{}', dt={}, decimation={}",
                 robot_name, timestep_, control_decimation_);
}

void Simulator::step(const Eigen::VectorXd& action) {
    if (!robot_ || !world_) return;

    // Apply action
    robot_->apply_action(action, timestep_ * control_decimation_);

    // Step physics multiple times (decimation)
    for (int i = 0; i < control_decimation_; i++) {
        world_->step();
    }

    // Update contacts
    auto contacts = world_->get_contacts();
    robot_->set_contacts(contacts);
}

void Simulator::reset() {
    if (robot_) robot_->reset();
    // Re-add robot to physics world
    if (world_ && robot_) {
        world_->remove_robot(robot_->name());
        world_->add_robot(robot_);
    }
}

double Simulator::simulation_time() const {
    return world_ ? world_->simulation_time() : 0.0;
}

} // namespace robosim::core
