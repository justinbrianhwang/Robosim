#include "robosim/checkpoint/state_snapshot.h"
#include <spdlog/spdlog.h>
#include <filesystem>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <ctime>

namespace fs = std::filesystem;

namespace robosim::checkpoint {

// StateSnapshot serialization (simple binary format)
bool StateSnapshot::save(const std::string& filepath) const {
    std::ofstream ofs(filepath, std::ios::binary);
    if (!ofs.is_open()) return false;

    auto write_double = [&](double v) { ofs.write(reinterpret_cast<const char*>(&v), sizeof(v)); };
    auto write_int = [&](int v) { ofs.write(reinterpret_cast<const char*>(&v), sizeof(v)); };
    auto write_vec = [&](const Eigen::VectorXd& v) {
        int size = static_cast<int>(v.size());
        write_int(size);
        ofs.write(reinterpret_cast<const char*>(v.data()), size * sizeof(double));
    };
    auto write_str = [&](const std::string& s) {
        int len = static_cast<int>(s.size());
        write_int(len);
        ofs.write(s.data(), len);
    };

    // Header
    write_str("ROBOSIM_CKPT_V1");
    write_double(simulation_time);
    write_int(step_count);
    write_int(static_cast<int>(rng_seed));

    // Base state
    for (int i = 0; i < 3; i++) write_double(base_position(i));
    write_double(base_orientation.w());
    write_double(base_orientation.x());
    write_double(base_orientation.y());
    write_double(base_orientation.z());
    for (int i = 0; i < 3; i++) write_double(base_linear_velocity(i));
    for (int i = 0; i < 3; i++) write_double(base_angular_velocity(i));

    // Joint state
    write_vec(joint_positions);
    write_vec(joint_velocities);

    // Task
    write_str(task_name);

    return ofs.good();
}

bool StateSnapshot::load(const std::string& filepath) {
    std::ifstream ifs(filepath, std::ios::binary);
    if (!ifs.is_open()) return false;

    auto read_double = [&]() -> double { double v; ifs.read(reinterpret_cast<char*>(&v), sizeof(v)); return v; };
    auto read_int = [&]() -> int { int v; ifs.read(reinterpret_cast<char*>(&v), sizeof(v)); return v; };
    auto read_vec = [&]() -> Eigen::VectorXd {
        int size = read_int();
        Eigen::VectorXd v(size);
        ifs.read(reinterpret_cast<char*>(v.data()), size * sizeof(double));
        return v;
    };
    auto read_str = [&]() -> std::string {
        int len = read_int();
        std::string s(len, '\0');
        ifs.read(s.data(), len);
        return s;
    };

    std::string header = read_str();
    if (header != "ROBOSIM_CKPT_V1") return false;

    simulation_time = read_double();
    step_count = read_int();
    rng_seed = static_cast<unsigned int>(read_int());

    for (int i = 0; i < 3; i++) base_position(i) = read_double();
    base_orientation.w() = read_double();
    base_orientation.x() = read_double();
    base_orientation.y() = read_double();
    base_orientation.z() = read_double();
    for (int i = 0; i < 3; i++) base_linear_velocity(i) = read_double();
    for (int i = 0; i < 3; i++) base_angular_velocity(i) = read_double();

    joint_positions = read_vec();
    joint_velocities = read_vec();
    task_name = read_str();

    return ifs.good();
}

// CheckpointManager
CheckpointManager::CheckpointManager(const std::string& directory)
    : directory_(directory) {
    fs::create_directories(directory);
}

std::string CheckpointManager::save(const StateSnapshot& snapshot, const std::string& tag) {
    std::string filename;
    if (tag.empty()) {
        filename = "checkpoint_" + std::to_string(counter_++) + ".bin";
    } else {
        filename = "checkpoint_" + tag + ".bin";
    }
    std::string filepath = directory_ + "/" + filename;

    if (snapshot.save(filepath)) {
        spdlog::info("Checkpoint saved: {}", filepath);
        enforce_max();
        return filepath;
    }
    spdlog::error("Failed to save checkpoint: {}", filepath);
    return "";
}

bool CheckpointManager::load(StateSnapshot& snapshot, const std::string& filepath) {
    if (snapshot.load(filepath)) {
        spdlog::info("Checkpoint loaded: {}", filepath);
        return true;
    }
    spdlog::error("Failed to load checkpoint: {}", filepath);
    return false;
}

bool CheckpointManager::load_latest(StateSnapshot& snapshot) {
    auto files = list_checkpoints();
    if (files.empty()) return false;
    return load(snapshot, files.back());
}

std::vector<std::string> CheckpointManager::list_checkpoints() const {
    std::vector<std::string> files;
    if (!fs::exists(directory_)) return files;
    for (const auto& entry : fs::directory_iterator(directory_)) {
        if (entry.path().extension() == ".bin") {
            files.push_back(entry.path().string());
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

void CheckpointManager::enforce_max() {
    auto files = list_checkpoints();
    while (static_cast<int>(files.size()) > max_checkpoints_) {
        fs::remove(files.front());
        files.erase(files.begin());
    }
}

} // namespace robosim::checkpoint
