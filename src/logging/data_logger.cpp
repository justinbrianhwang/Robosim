#include "robosim/logging/data_logger.h"
#include <spdlog/spdlog.h>
#include <filesystem>
#include <iomanip>
#include <chrono>
#include <sstream>

namespace robosim::logging {

DataLogger::DataLogger(const std::string& log_dir) : log_dir_(log_dir) {
    std::filesystem::create_directories(log_dir);
    is_open_ = true;
    spdlog::info("DataLogger initialized: {}", log_dir);
}

DataLogger::~DataLogger() {
    close();
}

void DataLogger::register_signal(const SignalDescriptor& desc) {
    signals_[desc.name] = desc;
    series_[desc.name] = TimeSeries{};
}

void DataLogger::log_scalar(const std::string& name, double time, double value) {
    auto it = series_.find(name);
    if (it == series_.end()) return;
    auto& ts = it->second;
    ts.times.push_back(time);
    ts.values.push_back(value);
    while (static_cast<int>(ts.times.size()) > ts.max_size) {
        ts.times.pop_front();
        ts.values.pop_front();
    }

    // Write to CSV if open
    if (csv_file_ && csv_file_->is_open()) {
        *csv_file_ << time << "," << name << "," << value << "\n";
    }
}

void DataLogger::log_vector(const std::string& name, double time, const Eigen::VectorXd& value) {
    for (int i = 0; i < value.size(); i++) {
        std::string key = name + "/" + std::to_string(i);
        // Auto-register sub-signal if not yet registered
        if (series_.find(key) == series_.end()) {
            register_signal({key, "", 1});
        }
        log_scalar(key, time, value(i));
    }
}

void DataLogger::new_episode() {
    episode_count_++;
    // Clear time series
    for (auto& [name, ts] : series_) {
        ts.times.clear();
        ts.values.clear();
    }

    // Open new CSV file
    if (csv_file_) csv_file_->close();
    std::string filename = log_dir_ + "/episode_" + std::to_string(episode_count_) + ".csv";
    csv_file_ = std::make_unique<std::ofstream>(filename);
    if (csv_file_->is_open()) {
        *csv_file_ << "time,signal,value\n";
    }
}

void DataLogger::flush() {
    if (csv_file_ && csv_file_->is_open()) csv_file_->flush();
}

void DataLogger::close() {
    if (csv_file_) {
        csv_file_->close();
        csv_file_.reset();
    }
    is_open_ = false;
}

const DataLogger::TimeSeries* DataLogger::get_series(const std::string& name) const {
    auto it = series_.find(name);
    return (it != series_.end()) ? &it->second : nullptr;
}

} // namespace robosim::logging
