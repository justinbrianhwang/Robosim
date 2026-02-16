#include "robosim/robot/robot_factory.h"
#include "robosim/robot/definitions/quadruped_def.h"
#include "robosim/robot/definitions/humanoid_def.h"
#include "robosim/robot/definitions/arm_def.h"
#include "robosim/robot/definitions/drone_def.h"
#include "robosim/robot/definitions/mobile_def.h"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <stdexcept>

namespace robosim::robot {

RobotFactory& RobotFactory::instance() {
    static RobotFactory factory;
    return factory;
}

void RobotFactory::register_robot(const std::string& id, const RobotInfo& info, Creator creator) {
    registry_[id] = Entry{info, std::move(creator)};
    spdlog::debug("Registered robot: {} ({})", id, info.display_name);
}

core::RobotDef RobotFactory::create(const std::string& id) const {
    auto it = registry_.find(id);
    if (it == registry_.end()) {
        throw std::runtime_error("Unknown robot: " + id);
    }
    return it->second.creator();
}

std::vector<RobotInfo> RobotFactory::available_robots() const {
    std::vector<RobotInfo> result;
    result.reserve(registry_.size());
    for (auto& [id, entry] : registry_) {
        result.push_back(entry.info);
    }
    std::sort(result.begin(), result.end(), [](const RobotInfo& a, const RobotInfo& b) {
        if (a.category != b.category) return static_cast<int>(a.category) < static_cast<int>(b.category);
        return a.id < b.id;
    });
    return result;
}

std::vector<RobotInfo> RobotFactory::robots_by_category(core::RobotCategory cat) const {
    std::vector<RobotInfo> result;
    for (auto& [id, entry] : registry_) {
        if (entry.info.category == cat) result.push_back(entry.info);
    }
    return result;
}

bool RobotFactory::has_robot(const std::string& id) const {
    return registry_.count(id) > 0;
}

const RobotInfo& RobotFactory::info(const std::string& id) const {
    auto it = registry_.find(id);
    if (it == registry_.end()) {
        throw std::runtime_error("Unknown robot: " + id);
    }
    return it->second.info;
}

void RobotFactory::register_all_robots() {
    auto& f = instance();

    // ====== Quadrupeds ======
    f.register_robot("unitree_go2", RobotInfo{
        "unitree_go2", "Unitree Go2",
        "Compact quadruped robot for research and education",
        core::RobotCategory::Quadruped, "Unitree", 12, 8.0
    }, definitions::create_go2);

    f.register_robot("boston_dynamics_spot", RobotInfo{
        "boston_dynamics_spot", "Boston Dynamics Spot",
        "Industrial inspection and exploration robot",
        core::RobotCategory::Quadruped, "Boston Dynamics", 12, 25.0
    }, definitions::create_spot);

    f.register_robot("anybotics_anymal_c", RobotInfo{
        "anybotics_anymal_c", "ANYbotics ANYmal C",
        "Industrial inspection quadruped",
        core::RobotCategory::Quadruped, "ANYbotics", 12, 30.0
    }, definitions::create_anymal);

    f.register_robot("xiaomi_cyberdog2", RobotInfo{
        "xiaomi_cyberdog2", "Xiaomi CyberDog 2",
        "Consumer quadruped robot",
        core::RobotCategory::Quadruped, "Xiaomi", 12, 8.9
    }, definitions::create_cyberdog);

    // ====== Humanoids ======
    f.register_robot("boston_dynamics_atlas", RobotInfo{
        "boston_dynamics_atlas", "Boston Dynamics Atlas",
        "Advanced bipedal humanoid for dynamic locomotion",
        core::RobotCategory::Humanoid, "Boston Dynamics", 23, 89.0
    }, definitions::create_atlas);

    f.register_robot("agility_digit", RobotInfo{
        "agility_digit", "Agility Robotics Digit",
        "Bipedal robot for logistics and warehouse",
        core::RobotCategory::Humanoid, "Agility Robotics", 20, 45.0
    }, definitions::create_digit);

    f.register_robot("tesla_optimus", RobotInfo{
        "tesla_optimus", "Tesla Optimus Gen 2",
        "General-purpose humanoid for manufacturing",
        core::RobotCategory::Humanoid, "Tesla", 23, 72.0
    }, definitions::create_optimus);

    f.register_robot("figure_01", RobotInfo{
        "figure_01", "Figure 01",
        "AI-powered humanoid for general tasks",
        core::RobotCategory::Humanoid, "Figure AI", 23, 60.0
    }, definitions::create_figure01);

    // ====== Robot Arms ======
    f.register_robot("franka_panda", RobotInfo{
        "franka_panda", "Franka Emika Panda",
        "7-DOF collaborative robot arm for research",
        core::RobotCategory::Arm, "Franka Emika", 9, 18.0
    }, definitions::create_panda);

    f.register_robot("universal_robots_ur5e", RobotInfo{
        "universal_robots_ur5e", "Universal Robots UR5e",
        "6-DOF collaborative industrial robot",
        core::RobotCategory::Arm, "Universal Robots", 6, 20.5
    }, definitions::create_ur5e);

    f.register_robot("kuka_iiwa14", RobotInfo{
        "kuka_iiwa14", "KUKA LBR iiwa 14",
        "7-DOF lightweight industrial robot",
        core::RobotCategory::Arm, "KUKA", 7, 29.9
    }, definitions::create_iiwa);

    // ====== Drones ======
    f.register_robot("generic_quadrotor", RobotInfo{
        "generic_quadrotor", "Generic Quadrotor",
        "Standard X-configuration research drone",
        core::RobotCategory::Drone, "Generic", 4, 0.8
    }, definitions::create_quadrotor);

    f.register_robot("dji_hexarotor", RobotInfo{
        "dji_hexarotor", "DJI Matrice-like Hexarotor",
        "Heavy-lift industrial drone",
        core::RobotCategory::Drone, "DJI", 6, 3.5
    }, definitions::create_hexarotor);

    // ====== Mobile Robots ======
    f.register_robot("differential_drive", RobotInfo{
        "differential_drive", "Differential Drive Robot",
        "TurtleBot-style mobile platform for SLAM and navigation",
        core::RobotCategory::MobileRobot, "Generic", 2, 6.0
    }, definitions::create_differential_drive);

    f.register_robot("ackermann_robot", RobotInfo{
        "ackermann_robot", "Ackermann Steering Robot",
        "Car-like mobile platform for autonomous driving",
        core::RobotCategory::MobileRobot, "Generic", 6, 12.0
    }, definitions::create_ackermann_robot);

    spdlog::info("Registered {} robot models", f.registry_.size());
}

} // namespace robosim::robot
