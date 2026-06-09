#ifndef AUTONOMOUS_DRONE_HPP
#define AUTONOMOUS_DRONE_HPP

#include "mission_drone.hpp"

class AutonomousDrone : public MissionDrone {
public:
  AutonomousDrone(const std::string &name,
                  const std::string &mission_name,
                  const std::vector<Waypoint> &waypoints,
                  const Waypoint &home_position,
                  float battery_level = 100.0f,
                  float max_altitude = 120.0f);

  void set_ai_mode(const std::string &mode);
  void detect_obstacle(Waypoint position, const std::string &severity);
  std::vector<Waypoint> auto_replan(const std::vector<Waypoint> &obstacles) const;

  std::string get_info() const override;

  const std::string &get_ai_mode() const;
  const Waypoint &get_home_position() const;
  const std::vector<std::string> &get_obstacle_log() const;

protected:
  std::string ai_mode_;
  Waypoint home_position_;

private:
  std::vector<std::string> obstacle_log_;
};

#endif