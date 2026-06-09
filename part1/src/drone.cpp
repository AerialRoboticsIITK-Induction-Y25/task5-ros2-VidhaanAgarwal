#include "drone.hpp"

#include <algorithm>
#include <sstream>

Drone::Drone(const std::string &name, float battery_level, float max_altitude)
    : Vehicle(name, battery_level), altitude_(0.0f), max_altitude_(max_altitude), speed_(0.0f) {}

void Drone::take_off(float target_altitude) {
  if (target_altitude > max_altitude_) {
    throw AltitudeError("Requested altitude exceeds limit for " + get_name());
  }

  if (get_battery_level() <= 0.0f) {
    throw BatteryDepletedError("Cannot take off with empty battery for " + get_name());
  }

  altitude_ = std::max(0.0f, target_altitude);
  
  float calculated_speed = altitude_ / 10.0f + 1.0f;
  float new_speed = std::max(1.0f, calculated_speed);
  set_speed(new_speed);
  
  set_status("flying");
}

void Drone::land() {
  altitude_ = 0.0f;
  set_speed(0.0f);
  set_status("idle");
}

void Drone::emergency_stop() {
  try {
    drain_battery(30.0f);
  } catch (const BatteryDepletedError &) {
  }
  altitude_ = 0.0f;
  set_speed(0.0f);
  set_status("idle");
}

float Drone::get_altitude() const {
  return altitude_;
}

float Drone::get_max_altitude() const {
  return max_altitude_;
}

float Drone::get_speed() const {
  return speed_;
}

std::string Drone::get_info() const {
  std::ostringstream stream;
  
  std::string name = get_name();
  std::string battery_str = drone_utils::format_float(get_battery_level(), 1);
  std::string status = get_status();
  std::string altitude_str = drone_utils::format_float(altitude_, 1);
  std::string speed_str = drone_utils::format_float(speed_, 1);
  
  stream << "Vehicle[name=" << name << ", battery=" << battery_str << "%, status=" << status 
         << ", altitude=" << altitude_str << ", speed=" << speed_str << "]";
  
  return stream.str();
}

void Drone::set_speed(float speed) {
  speed_ = std::max(0.0f, speed);
}