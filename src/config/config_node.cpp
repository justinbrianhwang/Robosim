#include "robosim/config/config_node.h"
#include <sstream>

namespace robosim::config {

ConfigNode::ConfigNode(YAML::Node node, std::string path)
    : node_(std::move(node)), path_(std::move(path)) {}

YAML::Node ConfigNode::resolve(const std::string& dotted_key) const {
    std::istringstream iss(dotted_key);
    std::string token;
    YAML::Node current = node_;
    while (std::getline(iss, token, '.')) {
        if (!current.IsMap() || !current[token]) {
            return YAML::Node();
        }
        current = current[token];
    }
    return current;
}

template<>
std::string ConfigNode::get<std::string>(const std::string& key) const {
    auto node = resolve(key);
    if (!node) throw ConfigError("Key not found: " + path_ + "." + key);
    return node.as<std::string>();
}

template<>
double ConfigNode::get<double>(const std::string& key) const {
    auto node = resolve(key);
    if (!node) throw ConfigError("Key not found: " + path_ + "." + key);
    return node.as<double>();
}

template<>
int ConfigNode::get<int>(const std::string& key) const {
    auto node = resolve(key);
    if (!node) throw ConfigError("Key not found: " + path_ + "." + key);
    return node.as<int>();
}

template<>
bool ConfigNode::get<bool>(const std::string& key) const {
    auto node = resolve(key);
    if (!node) throw ConfigError("Key not found: " + path_ + "." + key);
    return node.as<bool>();
}

template<>
float ConfigNode::get<float>(const std::string& key) const {
    auto node = resolve(key);
    if (!node) throw ConfigError("Key not found: " + path_ + "." + key);
    return node.as<float>();
}

template<>
std::string ConfigNode::get<std::string>(const std::string& key, const std::string& def) const {
    auto node = resolve(key);
    return node ? node.as<std::string>() : def;
}

template<>
double ConfigNode::get<double>(const std::string& key, const double& def) const {
    auto node = resolve(key);
    return node ? node.as<double>() : def;
}

template<>
int ConfigNode::get<int>(const std::string& key, const int& def) const {
    auto node = resolve(key);
    return node ? node.as<int>() : def;
}

template<>
bool ConfigNode::get<bool>(const std::string& key, const bool& def) const {
    auto node = resolve(key);
    return node ? node.as<bool>() : def;
}

template<>
float ConfigNode::get<float>(const std::string& key, const float& def) const {
    auto node = resolve(key);
    return node ? node.as<float>() : def;
}

bool ConfigNode::has(const std::string& key) const {
    return resolve(key).IsDefined();
}

ConfigNode ConfigNode::child(const std::string& key) const {
    auto node = resolve(key);
    return ConfigNode(node, path_ + "." + key);
}

void ConfigNode::merge(const ConfigNode& overlay) {
    if (!overlay.node_.IsMap()) return;
    for (auto it = overlay.node_.begin(); it != overlay.node_.end(); ++it) {
        std::string key = it->first.as<std::string>();
        if (node_[key] && node_[key].IsMap() && it->second.IsMap()) {
            ConfigNode sub(node_[key], path_ + "." + key);
            ConfigNode sub_overlay(it->second, overlay.path_ + "." + key);
            sub.merge(sub_overlay);
        } else {
            node_[key] = it->second;
        }
    }
}

std::vector<ConfigNode> ConfigNode::as_list() const {
    std::vector<ConfigNode> result;
    if (!node_.IsSequence()) return result;
    for (size_t i = 0; i < node_.size(); i++) {
        result.emplace_back(node_[i], path_ + "[" + std::to_string(i) + "]");
    }
    return result;
}

Eigen::Vector3d ConfigNode::getVector3d(const std::string& key,
                                          const Eigen::Vector3d& def) const {
    auto node = resolve(key);
    if (!node || !node.IsSequence() || node.size() < 3) return def;
    return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

} // namespace robosim::config
