#include "vehicle.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct DroneSnapshot {
  std::string name;
  double battery{0.0};
  double altitude{0.0};
  std::string waypoint{"0/0"};
  std::string status{"unknown"};
  double speed{0.0};
};

std::string trim(const std::string& value) {
  size_t begin = value.find_first_not_of(" \t\n\r");
  if (begin == std::string::npos) {
    return "";
  }
  
  size_t end = value.find_last_not_of(" \t\n\r");
  return value.substr(begin, end - begin + 1);
}
std::string extract_pipe_value(const std::string& payload, const std::string& key) {
  std::string needle = key + ":";
  size_t start = payload.find(needle);
  if (start == std::string::npos) return "";
  
  start += needle.length();
  size_t end = payload.find("|", start);
  if (end == std::string::npos) {
    end = payload.length();
  }
  
  return trim(payload.substr(start, end - start));
}
std::string extract_json_value(const std::string& json, const std::string& key) {
  std::string needle = "\"" + key + "\"";
  size_t key_position = json.find(needle);
  
  if (key_position == std::string::npos) {
    return "";
  }

  size_t colon_position = json.find(':', key_position + needle.size());
  if (colon_position == std::string::npos) {
    return "";
  }

  size_t value_begin = json.find_first_not_of(" \t\n\r", colon_position + 1);
  if (value_begin == std::string::npos) {
    return "";
  }

  // Handle string values (quoted)
  if (json[value_begin] == '"') {
    size_t value_end = json.find('"', value_begin + 1);
    if (value_end == std::string::npos) {
      return "";
    }
    return json.substr(value_begin + 1, value_end - value_begin - 1);
  }

  // Handle numeric values (unquoted)
  size_t value_end = json.find_first_of(",}", value_begin);
  return trim(json.substr(value_begin, value_end - value_begin));
}

std::string timestamp_now() {
  return drone_utils::timestamp_now();
}

std::string report_line(const DroneSnapshot& snapshot) {
  std::ostringstream stream;
  
  stream << std::left;
  stream << std::setw(10) << snapshot.name;
  stream << std::setw(12) << drone_utils::format_float(static_cast<float>(snapshot.battery), 1);
  stream << std::setw(12) << drone_utils::format_float(static_cast<float>(snapshot.altitude), 1);
  stream << std::setw(12) << snapshot.waypoint;
  stream << std::setw(12) << snapshot.status;
  
  return stream.str();
}

}  // namespace

class FleetManager : public rclcpp::Node {
public:
  FleetManager() : Node("fleet_manager") {
    this->setup_subscriptions();
    this->setup_service();
    this->setup_timer();
  }

private:
  void setup_subscriptions() {
    for (const auto& drone_name : drone_names_) {
      states_[drone_name].name = drone_name;
      
      this->subscribe_to_status(drone_name);
      this->subscribe_to_alert(drone_name);
      this->subscribe_to_mission_complete(drone_name);
      this->subscribe_to_telemetry(drone_name);
    }
  }

  void subscribe_to_status(const std::string& drone_name) {
    auto subscription = this->create_subscription<std_msgs::msg::String>(
        "/drone/" + drone_name + "/status", 10,
        [this, drone_name](const std_msgs::msg::String::SharedPtr message) {
          this->on_status_message(drone_name, message);
        });
    subscriptions_.push_back(subscription);
  }

  void subscribe_to_alert(const std::string& drone_name) {
    auto subscription = this->create_subscription<std_msgs::msg::String>(
        "/drone/" + drone_name + "/alert", 10,
        [this, drone_name](const std_msgs::msg::String::SharedPtr message) {
          this->on_alert_message(drone_name, message);
        });
    subscriptions_.push_back(subscription);
  }

  void subscribe_to_mission_complete(const std::string& drone_name) {
    auto subscription = this->create_subscription<std_msgs::msg::String>(
        "/drone/" + drone_name + "/mission_complete", 10,
        [this, drone_name](const std_msgs::msg::String::SharedPtr message) {
          this->on_mission_complete(drone_name, message);
        });
    subscriptions_.push_back(subscription);
  }

  void subscribe_to_telemetry(const std::string& drone_name) {
    auto subscription = this->create_subscription<std_msgs::msg::String>(
        "/drone/" + drone_name + "/telemetry", 10,
        [this, drone_name](const std_msgs::msg::String::SharedPtr message) {
          this->on_telemetry_message(drone_name, message);
        });
    subscriptions_.push_back(subscription);
  }

void on_status_message(const std::string& drone_name, const std_msgs::msg::String::SharedPtr message) {
    auto& state = states_[drone_name];
    std::string data = message->data;

    // Pluck exactly what you need out of the pipe-delimited string
    state.status = extract_pipe_value(data, "status");
    state.waypoint = extract_pipe_value(data, "waypoint");
    
    // You can also update altitude and speed here since they exist in the status string!
    std::string alt_str = extract_pipe_value(data, "altitude");
    if (!alt_str.empty()) state.altitude = std::stod(alt_str);
    
    std::string speed_str = extract_pipe_value(data, "speed");
    if (!speed_str.empty()) state.speed = std::stod(speed_str);
  }

  void on_alert_message(const std::string& /* drone_name */, const std_msgs::msg::String::SharedPtr message) {
    std::string timestamp = timestamp_now();
    RCLCPP_WARN(this->get_logger(), "[%s] alert received: %s", timestamp.c_str(), message->data.c_str());
  }

  void on_mission_complete(const std::string& drone_name, const std_msgs::msg::String::SharedPtr message) {
    RCLCPP_INFO(this->get_logger(), "Mission complete notice for %s: %s",
                drone_name.c_str(), message->data.c_str());
  }

  void on_telemetry_message(const std::string& drone_name, const std_msgs::msg::String::SharedPtr message) {
    this->update_from_telemetry(drone_name, message->data);
  }

  void setup_service() {
    auto service_callback = std::bind(&FleetManager::on_status_report_request, this,
                                      std::placeholders::_1, std::placeholders::_2);
    report_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/fleet/status_report", service_callback);
  }

  void on_status_report_request(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    response->success = true;
    response->message = this->print_report();
  }

  void setup_timer() {
    report_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&FleetManager::on_report_timer, this));
  }

  void on_report_timer() {
    this->print_report();
  }

  void update_from_telemetry(const std::string& drone_name, const std::string& json_message) {
    auto& state = states_[drone_name];
    // ONLY update what the JSON actually provides (name and battery)
    state.name = extract_json_value(json_message, "name");
    
    std::string battery_str = extract_json_value(json_message, "battery");
    if (!battery_str.empty()) {
      state.battery = std::stod(battery_str);
    }
  }

  std::string print_report() const {
    std::ostringstream report;
    
    std::string current_time = timestamp_now();
    report << "\nFleet report @ " << current_time << '\n';
    
    // Print header
    report << std::left;
    report << std::setw(10) << "Drone";
    report << std::setw(12) << "Battery";
    report << std::setw(12) << "Altitude";
    report << std::setw(12) << "Waypoint";
    report << std::setw(12) << "Status";
    report << '\n';
    
    
    report << "------------------------------------------------------------\n";
    
    // Print each drone's data
    for (const auto& drone_name : drone_names_) {
      auto it = states_.find(drone_name);
      if (it != states_.end()) {
        report << report_line(it->second) << '\n';
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "%s", report.str().c_str());
    return report.str();
  }

  const std::vector<std::string> drone_names_{"Alpha", "Beta", "Gamma"};
  std::map<std::string, DroneSnapshot> states_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr report_service_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<FleetManager>();
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}