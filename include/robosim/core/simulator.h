#pragma once

#include "robosim/core/world.h"
#include "robosim/core/robot.h"
#include "robosim/config/config_node.h"
#include <memory>
#include <string>

namespace robosim::core {

class Simulator {
public:
    Simulator() = default;
    explicit Simulator(const config::ConfigNode& cfg);
    ~Simulator() = default;

    void initialize(const std::string& robot_name);
    void step(const Eigen::VectorXd& action);
    void reset();

    World& world() { return *world_; }
    const World& world() const { return *world_; }
    Robot& robot() { return *robot_; }
    const Robot& robot() const { return *robot_; }

    double timestep() const { return timestep_; }
    int control_decimation() const { return control_decimation_; }
    double simulation_time() const;

private:
    std::shared_ptr<World> world_;
    std::shared_ptr<Robot> robot_;
    double timestep_ = 0.002;
    int control_decimation_ = 4; // control at 125Hz, physics at 500Hz
};

} // namespace robosim::core
