#pragma once

#include "robosim/config/config_node.h"
#include <string>
#include <vector>
#include <utility>

namespace robosim::config {

class ConfigManager {
public:
    ConfigManager() = default;

    void load_file(const std::string& filepath);
    void merge_file(const std::string& filepath);
    void set_override(const std::string& key, const std::string& value);
    void apply_overrides();
    const ConfigNode& root() const { return root_; }
    void dump(const std::string& filepath) const;

private:
    ConfigNode root_;
    std::vector<std::pair<std::string, std::string>> overrides_;
};

} // namespace robosim::config
