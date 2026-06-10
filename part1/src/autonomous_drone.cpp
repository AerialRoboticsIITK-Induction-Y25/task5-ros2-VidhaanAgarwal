#include "autonomous_drone.hpp"

#include <cmath>
#include <sstream>

AutonomousDrone::AutonomousDrone(const std::string &name,
                                 const std::string &mission_name,
                                 const std::vector<Waypoint> &waypoints,
                                 const Waypoint &home_position,
                                 float battery_level,
                                 float max_altitude)
    : MissionDrone(name, mission_name, waypoints, battery_level, max_altitude), ai_mode_("manual"),
      home_position_(home_position) {}

void AutonomousDrone::set_ai_mode(const std::string &mode) {
  bool is_manual = (mode == "manual");
  bool is_auto = (mode == "auto");
  bool is_return_home = (mode == "return_home");
  bool is_valid = (is_manual || is_auto || is_return_home);
  
  if (!is_valid) {
    throw InvalidStateError("Unsupported AI mode for " + get_name() + ": " + mode);
  }

  ai_mode_ = mode;
  
  if (ai_mode_ == "return_home") {
    int insert_index = std::min(current_waypoint_index_, static_cast<int>(waypoints_.size()));
    auto insert_position = waypoints_.begin() + insert_index;
    
    bool position_end = (insert_position == waypoints_.end());
    bool position_matches = (!position_end && *insert_position == home_position_);
    bool should_insert = (position_end || !position_matches);
    
    if (should_insert) {
      waypoints_.insert(insert_position, home_position_);
    }
  }
}

void AutonomousDrone::detect_obstacle(Waypoint position, const std::string &severity) {
  std::ostringstream stream;
  
  std::string timestamp = drone_utils::timestamp_now();
  std::string position_str = drone_utils::waypoint_to_string(position);
  
  stream << "[" << timestamp << "] obstacle at " << position_str << " severity=" << severity;
  obstacle_log_.push_back(stream.str());
  
  bool is_high_severity = (severity == "high");
  if (is_high_severity) {
    emergency_stop();
  }
}

std::vector<Waypoint> AutonomousDrone::auto_replan(const std::vector<Waypoint> &obstacles) const {
  std::vector<Waypoint> replanned_waypoints;
  
  for (const auto &waypoint : waypoints_) {
    bool blocked = false;
    
    for (const auto &obstacle : obstacles) {
      float distance = drone_utils::waypoint_distance(waypoint, obstacle);
      float obstacle_radius = 5.0f;
      
      if (distance <= obstacle_radius) {
        blocked = true;
        break;
      }
    }
    
    if (!blocked) {
      replanned_waypoints.push_back(waypoint);
    }
  }
  
  return replanned_waypoints;
}

std::string AutonomousDrone::get_info() const {
  std::ostringstream stream;
  
  std::string altitude_str = drone_utils::format_float(get_altitude(), 1);
  std::string battery_str = drone_utils::format_float(get_battery_level(), 1);
  
  stream << "AutonomousDrone[name=" << get_name() << ", ai_mode=" << ai_mode_ << ", mission=" 
         << get_mission_name() << ", altitude=" << altitude_str << ", battery=" << battery_str << "%]";
  
  return stream.str();
}

const std::string &AutonomousDrone::get_ai_mode() const {
  return ai_mode_;
}

const Waypoint &AutonomousDrone::get_home_position() const {
  return home_position_;
}

const std::vector<std::string> &AutonomousDrone::get_obstacle_log() const {
  return obstacle_log_;
}