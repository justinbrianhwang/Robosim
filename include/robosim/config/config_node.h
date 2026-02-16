#pragma once

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <optional>
#include <stdexcept>
#include <Eigen/Dense>

namespace robosim::config {

class ConfigError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

class ConfigNode {
public:
    ConfigNode() = default;
    explicit ConfigNode(YAML::Node node, std::string path = "root");

    template<typename T>
    T get(const std::string& key) const;

    template<typename T>
    T get(const std::string& key, const T& default_value) const;

    bool has(const std::string& key) const;
    ConfigNode child(const std::string& key) const;
    void merge(const ConfigNode& overlay);
    std::vector<ConfigNode> as_list() const;
    const YAML::Node& raw() const { return node_; }
    std::string path() const { return path_; }

    // Eigen vector support
    Eigen::Vector3d getVector3d(const std::string& key,
                                 const Eigen::Vector3d& def = Eigen::Vector3d::Zero()) const;

private:
    YAML::Node node_;
    std::string path_;
    YAML::Node resolve(const std::string& dotted_key) const;
};

} // namespace robosim::config
