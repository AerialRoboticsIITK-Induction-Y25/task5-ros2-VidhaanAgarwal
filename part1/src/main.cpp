#include "autonomous_drone.hpp"
#include "drone.hpp"
#include "mission_drone.hpp"

#include <iostream>
#include <vector>

int main() {
  // Initialize drones with explicit separate statements
  Drone base_drone("Alpha", 10.0f, 80.0f);
  
  std::vector<Waypoint> mission_waypoints_beta;
  mission_waypoints_beta.push_back(Waypoint{0.0f, 0.0f, 10.0f});
  mission_waypoints_beta.push_back(Waypoint{5.0f, 5.0f, 15.0f});
  mission_waypoints_beta.push_back(Waypoint{10.0f, 10.0f, 20.0f});
  MissionDrone mission_drone("Beta", "Survey Mission", mission_waypoints_beta, 50.0f, 90.0f);
  
  std::vector<Waypoint> mission_waypoints_gamma;
  mission_waypoints_gamma.push_back(Waypoint{1.0f, 1.0f, 12.0f});
  mission_waypoints_gamma.push_back(Waypoint{2.0f, 2.0f, 18.0f});
  mission_waypoints_gamma.push_back(Waypoint{3.0f, 3.0f, 25.0f});
  Waypoint home_position{0.0f, 0.0f, 0.0f};
  AutonomousDrone autonomous_drone("Gamma", "Rescue Mission", mission_waypoints_gamma, home_position, 60.0f, 100.0f);

  // Display vehicle info
  std::vector<Vehicle *> vehicles = {&base_drone, &mission_drone, &autonomous_drone};
  for (const auto *vehicle : vehicles) {
    std::cout << vehicle->get_info() << std::endl;
  }

  // Test exception handling for battery drain
  try {
    base_drone.drain_battery(15.0f);
    base_drone.drain_battery(5.0f);
  } catch (const DroneException &exception) {
    std::cout << "Caught drone exception: " << exception.what() << std::endl;
  }

  // Test exception handling for altitude
  try {
    mission_drone.take_off(200.0f);
  } catch (const AltitudeError &exception) {
    std::cout << "Caught altitude exception: " << exception.what() << std::endl;
  }

  // Test autonomous drone obstacle detection
  try {
    Waypoint obstacle_location{4.0f, 4.0f, 4.0f};
    autonomous_drone.detect_obstacle(obstacle_location, "high");
  } catch (const DroneException &exception) {
    std::cout << "Caught autonomous exception: " << exception.what() << std::endl;
  }

  // Test autonomous drone mission execution
  try {
    autonomous_drone.take_off(20.0f);
    autonomous_drone.set_ai_mode("auto");
    
    while (!autonomous_drone.mission_complete()) {
      const Waypoint waypoint = autonomous_drone.next_waypoint();
      std::string waypoint_str = drone_utils::waypoint_to_string(waypoint);
      std::cout << "Visited waypoint: " << waypoint_str << std::endl;
    }
    
    Waypoint final_obstacle{9.0f, 9.0f, 9.0f};
    autonomous_drone.detect_obstacle(final_obstacle, "high");
    
    std::string mission_summary = autonomous_drone.mission_summary();
    std::cout << mission_summary << std::endl;
  } catch (const DroneException &exception) {
    std::cout << "Mission error: " << exception.what() << std::endl;
  }

  // Display flight log
  std::string flight_log = autonomous_drone.get_flight_log();
  std::cout << "\nFlight log for Gamma:\n" << flight_log << std::endl;
  
  return 0;
}