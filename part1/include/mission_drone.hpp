#ifndef MISSION_DRONE_HPP
#define MISSION_DRONE_HPP

#include "drone.hpp"

class MissionDrone : public Drone {
public:
  MissionDrone(const std::string &name,
               const std::string &mission_name,
               const std::vector<Waypoint> &waypoints,
               float battery_level = 100.0f,
               float max_altitude = 120.0f);

  Waypoint next_waypoint();
  void skip_waypoint(const std::string &reason);
  bool mission_complete() const;
  std::string mission_summary() const;
  std::string get_info() const override;

  const std::string &get_mission_name() const;
  const std::vector<Waypoint> &get_waypoints() const;
  int get_current_waypoint_index() const;
  const std::vector<std::pair<Waypoint, std::string>> &get_visited_waypoints() const;

protected:
  std::string mission_name_;
  std::vector<Waypoint> waypoints_;
  int current_waypoint_index_;

private:
  std::vector<std::pair<Waypoint, std::string>> visited_waypoints_;
};

#endif