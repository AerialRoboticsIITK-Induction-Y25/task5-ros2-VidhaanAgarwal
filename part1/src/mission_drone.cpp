#include "mission_drone.hpp"

#include <algorithm>
#include <sstream>

MissionDrone::MissionDrone(const std::string &name,
                           const std::string &mission_name,
                           const std::vector<Waypoint> &waypoints,
                           float battery_level,
                           float max_altitude)
    : Drone(name, battery_level, max_altitude), mission_name_(mission_name), waypoints_(waypoints),
      current_waypoint_index_(0) {}

Waypoint MissionDrone::next_waypoint() {
  if (mission_complete()) {
    throw InvalidStateError("Mission already complete for " + get_name());
  }

  const Waypoint waypoint = waypoints_.at(static_cast<std::size_t>(current_waypoint_index_));
  
  altitude_ = std::get<2>(waypoint);
  float calculated_speed = altitude_ / 8.0f + 1.0f;
  float new_speed = std::max(1.0f, calculated_speed);
  set_speed(new_speed);
  
  drain_battery(1.5f);
  
  std::string timestamp = drone_utils::timestamp_now();
  visited_waypoints_.push_back({waypoint, timestamp});
  
  ++current_waypoint_index_;
  return waypoint;
}

void MissionDrone::skip_waypoint(const std::string &reason) {
  if (mission_complete()) {
    throw InvalidStateError("Mission already complete for " + get_name());
  }

  const Waypoint waypoint = waypoints_.at(static_cast<std::size_t>(current_waypoint_index_));
  visited_waypoints_.push_back({waypoint, drone_utils::timestamp_now() + " skipped: " + reason});
  ++current_waypoint_index_;
}

bool MissionDrone::mission_complete() const {
  return current_waypoint_index_ >= static_cast<int>(waypoints_.size());
}

std::string MissionDrone::mission_summary() const {
  std::ostringstream stream;
  
  int total_waypoints = waypoints_.size();
  int visited_count = visited_waypoints_.size();
  std::string altitude_str = drone_utils::format_float(get_altitude(), 1);
  std::string battery_str = drone_utils::format_float(get_battery_level(), 1);
  
  stream << "Mission " << mission_name_ << " on " << get_name() << " completed " 
         << visited_count << "/" << total_waypoints << " waypoints. Current altitude: " 
         << altitude_str << ", battery: " << battery_str << "%";
  
  return stream.str();
}

std::string MissionDrone::get_info() const {
  std::ostringstream stream;
  
  int current_index = current_waypoint_index_;
  int total_waypoints = waypoints_.size();
  std::string altitude_str = drone_utils::format_float(get_altitude(), 1);
  
  stream << "MissionDrone[name=" << get_name() << ", mission=" << mission_name_ << ", waypoint=" 
         << current_index << "/" << total_waypoints << ", altitude=" << altitude_str 
         << ", status=" << get_status() << "]";
  
  return stream.str();
}

const std::string &MissionDrone::get_mission_name() const {
  return mission_name_;
}

const std::vector<Waypoint> &MissionDrone::get_waypoints() const {
  return waypoints_;
}

int MissionDrone::get_current_waypoint_index() const {
  return current_waypoint_index_;
}

const std::vector<std::pair<Waypoint, std::string>> &MissionDrone::get_visited_waypoints() const {
  return visited_waypoints_;
}