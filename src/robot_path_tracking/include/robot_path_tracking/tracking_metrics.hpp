#pragma once

#include <cstddef>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace robot_path_tracking {

struct TrackingMetricSample {
  double timestamp_sec = 0.0;
  std::string controller;
  std::string path_name;
  std::size_t sample_index = 0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double ref_x = 0.0;
  double ref_y = 0.0;
  double ref_yaw = 0.0;
  double linear_velocity = 0.0;
  double angular_velocity = 0.0;
  double lateral_error = 0.0;
  double heading_error = 0.0;
  double distance_to_goal = 0.0;
  bool goal_reached = false;
};

class TrackingCsvLogger {
 public:
  TrackingCsvLogger() = default;

  TrackingCsvLogger(const bool enabled, const std::string& output_path,
                    const std::string& controller_name, rclcpp::Logger logger)
      : enabled_(enabled), logger_(logger) {
    if (!enabled_) {
      return;
    }
    output_path_ = output_path.empty() ? DefaultOutputPath(controller_name) : output_path;
    try {
      const auto parent = std::filesystem::path(output_path_).parent_path();
      if (!parent.empty()) {
        std::filesystem::create_directories(parent);
      }
      stream_.open(output_path_, std::ios::out | std::ios::trunc);
    } catch (const std::exception& error) {
      enabled_ = false;
      RCLCPP_ERROR(logger_, "failed to prepare tracking CSV '%s': %s", output_path_.c_str(),
                   error.what());
      return;
    }
    if (!stream_) {
      enabled_ = false;
      RCLCPP_ERROR(logger_, "failed to open tracking CSV '%s'", output_path_.c_str());
      return;
    }
    stream_
        << "timestamp_sec,controller,path_name,sample_index,x,y,yaw,ref_x,ref_y,ref_yaw,"
           "linear_velocity,angular_velocity,lateral_error,heading_error,distance_to_goal,"
           "goal_reached\n";
    RCLCPP_INFO(logger_, "tracking CSV logging enabled: %s", output_path_.c_str());
  }

  ~TrackingCsvLogger() {
    if (stream_.is_open()) {
      stream_.close();
    }
  }

  TrackingCsvLogger(const TrackingCsvLogger&) = delete;
  TrackingCsvLogger& operator=(const TrackingCsvLogger&) = delete;

  bool Enabled() const { return enabled_ && stream_.is_open(); }
  const std::string& OutputPath() const { return output_path_; }

  void Write(const TrackingMetricSample& sample) {
    if (!Enabled()) {
      return;
    }
    stream_ << std::fixed << std::setprecision(6) << sample.timestamp_sec << ','
            << CsvEscape(sample.controller) << ',' << CsvEscape(sample.path_name) << ','
            << sample.sample_index << ',' << sample.x << ',' << sample.y << ',' << sample.yaw
            << ',' << sample.ref_x << ','
            << sample.ref_y << ',' << sample.ref_yaw << ',' << sample.linear_velocity << ','
            << sample.angular_velocity << ',' << sample.lateral_error << ','
            << sample.heading_error << ',' << sample.distance_to_goal << ','
            << (sample.goal_reached ? "true" : "false") << '\n';
    if (!stream_) {
      enabled_ = false;
      RCLCPP_ERROR(logger_, "failed while writing tracking CSV '%s'", output_path_.c_str());
    }
  }

  static std::string DefaultOutputPath(const std::string& controller_name) {
    std::time_t now = std::time(nullptr);
    std::tm local_time{};
#ifdef _WIN32
    localtime_s(&local_time, &now);
#else
    localtime_r(&now, &local_time);
#endif
    std::ostringstream stamp;
    stamp << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    return "src/robot_experiments/results/tracking_" + controller_name + "_" + stamp.str() +
           ".csv";
  }

  static std::string CsvEscape(const std::string& value) {
    if (value.find_first_of(",\"\n\r") == std::string::npos) {
      return value;
    }
    std::string escaped = "\"";
    for (const char character : value) {
      if (character == '"') {
        escaped += "\"\"";
      } else {
        escaped += character;
      }
    }
    escaped += '"';
    return escaped;
  }

 private:
  bool enabled_ = false;
  std::string output_path_;
  std::ofstream stream_;
  rclcpp::Logger logger_{rclcpp::get_logger("tracking_csv_logger")};
};

}  // namespace robot_path_tracking
