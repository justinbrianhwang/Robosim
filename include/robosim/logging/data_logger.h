#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <memory>
#include <deque>

namespace robosim::logging {

struct SignalDescriptor {
    std::string name;
    std::string unit;
    int dim = 1;
};

class DataLogger {
public:
    DataLogger() = default;
    explicit DataLogger(const std::string& log_dir);
    ~DataLogger();

    void register_signal(const SignalDescriptor& desc);
    void log_scalar(const std::string& name, double time, double value);
    void log_vector(const std::string& name, double time, const Eigen::VectorXd& value);
    void new_episode();
    void flush();
    void close();

    // Real-time access for plotting
    struct TimeSeries {
        std::deque<double> times;
        std::deque<double> values;
        int max_size = 10000;
    };
    const TimeSeries* get_series(const std::string& name) const;

    bool is_open() const { return is_open_; }
    int episode_count() const { return episode_count_; }

private:
    std::string log_dir_;
    bool is_open_ = false;
    int episode_count_ = 0;
    std::unordered_map<std::string, SignalDescriptor> signals_;
    std::unordered_map<std::string, TimeSeries> series_;
    std::unique_ptr<std::ofstream> csv_file_;
};

} // namespace robosim::logging
