#include "robosim/config/config_manager.h"
#include <fstream>
#include <sstream>
#include <spdlog/spdlog.h>

namespace robosim::config {

void ConfigManager::load_file(const std::string& filepath) {
    spdlog::info("Loading config: {}", filepath);
    YAML::Node node = YAML::LoadFile(filepath);
    root_ = ConfigNode(node, "root");
}

void ConfigManager::merge_file(const std::string& filepath) {
    spdlog::info("Merging config: {}", filepath);
    YAML::Node node = YAML::LoadFile(filepath);
    ConfigNode overlay(node, "overlay");
    root_.merge(overlay);
}

void ConfigManager::set_override(const std::string& key, const std::string& value) {
    overrides_.emplace_back(key, value);
}

void ConfigManager::apply_overrides() {
    for (const auto& [key, value] : overrides_) {
        // Parse dotted key and set value in YAML node
        std::istringstream iss(key);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(iss, token, '.')) {
            tokens.push_back(token);
        }

        YAML::Node current = const_cast<YAML::Node&>(root_.raw());
        for (size_t i = 0; i < tokens.size() - 1; i++) {
            if (!current[tokens[i]]) {
                current[tokens[i]] = YAML::Node(YAML::NodeType::Map);
            }
            current = current[tokens[i]];
        }
        current[tokens.back()] = value;
        spdlog::debug("Override: {} = {}", key, value);
    }
    overrides_.clear();
}

void ConfigManager::dump(const std::string& filepath) const {
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        spdlog::error("Failed to dump config to: {}", filepath);
        return;
    }
    YAML::Emitter emitter;
    emitter << root_.raw();
    ofs << emitter.c_str();
    spdlog::info("Config dumped to: {}", filepath);
}

} // namespace robosim::config
