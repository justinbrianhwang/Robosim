#pragma once

#include "robosim/core/robot.h"
#include <string>
#include <vector>
#include <functional>
#include <unordered_map>
#include <memory>

namespace robosim::robot {

struct RobotInfo {
    std::string id;
    std::string display_name;
    std::string description;
    core::RobotCategory category;
    std::string manufacturer;
    int num_joints;
    double weight_kg;
};

class RobotFactory {
public:
    static RobotFactory& instance();

    using Creator = std::function<core::RobotDef()>;

    void register_robot(const std::string& id, const RobotInfo& info, Creator creator);
    core::RobotDef create(const std::string& id) const;
    std::vector<RobotInfo> available_robots() const;
    std::vector<RobotInfo> robots_by_category(core::RobotCategory cat) const;
    bool has_robot(const std::string& id) const;
    const RobotInfo& info(const std::string& id) const;

    // Initialize all built-in robot definitions
    static void register_all_robots();

private:
    RobotFactory() = default;
    struct Entry {
        RobotInfo info;
        Creator creator;
    };
    std::unordered_map<std::string, Entry> registry_;
};

#define REGISTER_ROBOT(id, info, creator_func) \
    static bool _reg_robot_##id = [] { \
        RobotFactory::instance().register_robot(#id, info, creator_func); \
        return true; \
    }()

} // namespace robosim::robot
