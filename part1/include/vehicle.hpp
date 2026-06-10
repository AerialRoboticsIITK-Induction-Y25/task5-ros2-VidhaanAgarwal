#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include "drone_exceptions.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using Waypoint = std::tuple<float, float, float>;

namespace drone_utils {
inline std::string timestamp_now() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
#if defined(_WIN32)
  localtime_s(&local_time, &now_time);
#else
  localtime_r(&now_time, &local_time);
#endif
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S");
  return stream.str();
}

inline std::string format_float(float value, int precision = 1) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

inline std::string waypoint_to_string(const Waypoint &waypoint) {
  std::ostringstream stream;
  stream << "(" << format_float(std::get<0>(waypoint), 1) << ", "
         << format_float(std::get<1>(waypoint), 1) << ", "
         << format_float(std::get<2>(waypoint), 1) << ")";
  return stream.str();
}

inline float waypoint_distance(const Waypoint &lhs, const Waypoint &rhs) {
  const float dx = std::get<0>(lhs) - std::get<0>(rhs);
  const float dy = std::get<1>(lhs) - std::get<1>(rhs);
  const float dz = std::get<2>(lhs) - std::get<2>(rhs);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline std::string join_lines(const std::vector<std::string> &lines) {
  std::ostringstream stream;
  for (std::size_t index = 0; index < lines.size(); ++index) {
    stream << lines[index];
    if (index + 1 < lines.size()) {
      stream << '\n';
    }
  }
  return stream.str();
}
} // namespace drone_utils

class Vehicle {
public:
  explicit Vehicle(const std::string &name, float battery_level = 100.0f, const std::string &status = "idle");
  virtual ~Vehicle() = default;

  virtual std::string get_info() const = 0;

  void drain_battery(float amount);
  void charge_battery(float amount, int duration_seconds);
  bool is_critical() const;
  std::string get_flight_log() const;

  const std::string &get_name() const;
  float get_battery_level() const;
  const std::string &get_status() const;
  const std::vector<std::string> &get_flight_log_entries() const;

protected:
  void set_status(const std::string &new_status);
  void append_log(const std::string &entry);

  std::string name_;

private:
  float battery_level_;
  std::string status_;
  std::vector<std::string> flight_log_;
};

#endif