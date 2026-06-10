#ifndef DRONE_HPP
#define DRONE_HPP

#include "vehicle.hpp"

class Drone : public Vehicle {
public:
  explicit Drone(const std::string &name, float battery_level = 100.0f, float max_altitude = 120.0f);

  void take_off(float target_altitude);
  void land();
  void emergency_stop();

  float get_altitude() const;
  float get_max_altitude() const;
  float get_speed() const;

  std::string get_info() const override;

protected:
  void set_speed(float speed);

  float altitude_;
  float max_altitude_;

private:
  float speed_;
};

#endif