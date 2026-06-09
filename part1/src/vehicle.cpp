#include "vehicle.hpp"

#include <cmath>

Vehicle::Vehicle(const std::string &name, float battery_level, const std::string &status)
    : name_(name), battery_level_(std::clamp(battery_level, 0.0f, 100.0f)), status_("idle") {
  set_status(status);
  
  std::string current_time = drone_utils::timestamp_now();
  std::string log_entry = "[" + current_time + "] vehicle created for " + name_;
  append_log(log_entry);
}

void Vehicle::drain_battery(float amount) {
  if (battery_level_ <= 0.0f) {
    throw BatteryDepletedError("Battery already depleted for " + name_);
  }

  const float drain_amount = std::max(0.0f, amount);
  battery_level_ = std::max(0.0f, battery_level_ - drain_amount);
  
  std::string current_time = drone_utils::timestamp_now();
  std::string drain_str = drone_utils::format_float(drain_amount, 1);
  std::string remaining_str = drone_utils::format_float(battery_level_, 1);
  
  std::string log_entry = "[" + current_time + "] drained " + drain_str + "% battery; remaining " + remaining_str + "%";
  append_log(log_entry);
}

void Vehicle::charge_battery(float amount, int duration_seconds) {
  if (status_ != "charging") {
    throw InvalidStateError("Battery can only be charged while charging for " + name_);
  }

  const float charge_amount = std::max(0.0f, amount);
  battery_level_ = std::min(100.0f, battery_level_ + charge_amount);
  
  std::string current_time = drone_utils::timestamp_now();
  std::string charge_str = drone_utils::format_float(charge_amount, 1);
  std::string new_battery_str = drone_utils::format_float(battery_level_, 1);
  std::string duration_str = std::to_string(duration_seconds);
  
  std::string log_entry = "[" + current_time + "] charged for " + duration_str + "s by " + charge_str + "%; now " + new_battery_str + "%";
  append_log(log_entry);
}

bool Vehicle::is_critical() const {
  return battery_level_ <= 20.0f;
}

std::string Vehicle::get_flight_log() const {
  return drone_utils::join_lines(flight_log_);
}

const std::string &Vehicle::get_name() const {
  return name_;
}

float Vehicle::get_battery_level() const {
  return battery_level_;
}

const std::string &Vehicle::get_status() const {
  return status_;
}

const std::vector<std::string> &Vehicle::get_flight_log_entries() const {
  return flight_log_;
}

void Vehicle::set_status(const std::string &new_status) {
  bool is_valid_idle = (new_status == "idle");
  bool is_valid_flying = (new_status == "flying");
  bool is_valid_charging = (new_status == "charging");
  bool is_valid = (is_valid_idle || is_valid_flying || is_valid_charging);
  
  if (!is_valid) {
    throw InvalidStateError("Invalid status for " + name_ + ": " + new_status);
  }

  status_ = new_status;
  
  std::string current_time = drone_utils::timestamp_now();
  std::string log_entry = "[" + current_time + "] status set to " + status_;
  append_log(log_entry);
}

void Vehicle::append_log(const std::string &entry) {
  flight_log_.push_back(entry);
}