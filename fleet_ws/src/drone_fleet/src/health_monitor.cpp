#include "vehicle.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <deque>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Sample {
  double time{0.0};
  double battery{0.0};
};

struct HealthState {
  std::deque<Sample> samples;
  double battery{0.0};
  double altitude{0.0};
  std::string status{"unknown"};
  std::string waypoint{"0/0"};
  double drain_rate{0.0};
};

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
  return json.substr(value_begin, value_end - value_begin);
}

std::string timestamp_now() {
  return drone_utils::timestamp_now();
}

std::string diagnostics_row(const std::string& drone_name, const HealthState& state) {
  std::ostringstream stream;
  
  double time_to_critical = 0.0;
  double time_to_depletion = 0.0;
  
  if (state.drain_rate > 0.0) {
    time_to_critical = std::max(0.0, (state.battery - 20.0) / state.drain_rate);
    time_to_depletion = std::max(0.0, state.battery / state.drain_rate);
  }
  
  stream << std::left;
  stream << std::setw(10) << drone_name;
  stream << std::setw(12) << drone_utils::format_float(static_cast<float>(state.drain_rate), 2);
  stream << std::setw(14) << drone_utils::format_float(static_cast<float>(time_to_critical), 1);
  stream << std::setw(14) << drone_utils::format_float(static_cast<float>(time_to_depletion), 1);
  
  return stream.str();
}

}  // namespace

class HealthMonitor : public rclcpp::Node {
public:
  HealthMonitor() : Node("health_monitor") {
    this->setup_subscriptions();
    this->setup_publishers();
    this->setup_timer();
  }

private:
  void setup_subscriptions() {
    for (const auto& drone_name : drone_names_) {
      states_[drone_name] = HealthState{};
      
      auto subscription = this->create_subscription<std_msgs::msg::String>(
          "/drone/" + drone_name + "/telemetry", 10,
          [this, drone_name](const std_msgs::msg::String::SharedPtr message) {
            this->on_telemetry_message(drone_name, message);
          });
      subscriptions_.push_back(subscription);
    }
  }

  void setup_publishers() {
    warning_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/fleet/health_warning", 10);
    summary_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/fleet/health_summary", 10);
  }

  void setup_timer() {
    diagnostics_timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&HealthMonitor::on_diagnostics_timer, this));
  }

  void on_telemetry_message(const std::string& drone_name, const std_msgs::msg::String::SharedPtr message) {
    this->update_state(drone_name, message->data);
  }

  void on_diagnostics_timer() {
    this->publish_diagnostics();
  }

  void update_state(const std::string& drone_name, const std::string& json_message) {
    double current_time = this->now().seconds();
    auto& state = states_[drone_name];
    
    // Parse telemetry data
    std::string battery_str = extract_json_value(json_message, "battery");
    if (!battery_str.empty()) {
      state.battery = std::stod(battery_str);
    }
    
    std::string altitude_str = extract_json_value(json_message, "altitude");
    if (!altitude_str.empty()) {
      state.altitude = std::stod(altitude_str);
    }
    
    state.status = extract_json_value(json_message, "status");
    state.waypoint = extract_json_value(json_message, "waypoint");

    // Add sample to circular buffer
    state.samples.push_back(Sample{current_time, state.battery});
    while (state.samples.size() > 10) {
      state.samples.pop_front();
    }

    // Calculate drain rate from samples
    this->calculate_drain_rate(state);
    this->check_drain_rate_warning(drone_name, state);
  }

  void calculate_drain_rate(HealthState& state) {
    if (state.samples.size() < 2) {
      return;
    }
    
    const auto& first_sample = state.samples.front();
    const auto& last_sample = state.samples.back();
    
    double elapsed_time = last_sample.time - first_sample.time;
    if (elapsed_time < 0.001) {
      elapsed_time = 0.001;  // Avoid division by zero
    }
    
    double battery_drained = first_sample.battery - last_sample.battery;
    state.drain_rate = std::max(0.0, battery_drained / elapsed_time);
  }

  void check_drain_rate_warning(const std::string& drone_name, const HealthState& state) {
    double drain_threshold = 1.5;
    
    if (state.drain_rate > drain_threshold) {
      std::string drain_rate_str = drone_utils::format_float(static_cast<float>(state.drain_rate), 2);
      std::string warning_msg = drone_name + " drain rate high: " + drain_rate_str + " per second";
      
      std_msgs::msg::String warning_message;
      warning_message.data = warning_msg;
      warning_publisher_->publish(warning_message);
    }
  }

  void publish_diagnostics() {
    this->print_diagnostics_table();
    this->publish_health_summary();
  }

  void print_diagnostics_table() {
    std::ostringstream table;
    
    std::string current_time = timestamp_now();
    table << "\nHealth diagnostics @ " << current_time << '\n';
    
    // Print header
    table << std::left;
    table << std::setw(10) << "Drone";
    table << std::setw(12) << "Drain";
    table << std::setw(14) << "To Critical";
    table << std::setw(14) << "To Depletion";
    table << '\n';
    
    // Print separator
    table << "------------------------------------------------\n";
    
    // Print each drone's diagnostics
    for (const auto& drone_name : drone_names_) {
      const auto& state = states_[drone_name];
      table << diagnostics_row(drone_name, state) << '\n';
    }
    
    RCLCPP_INFO(this->get_logger(), "%s", table.str().c_str());
  }

  void publish_health_summary() {
    std::ostringstream summary_json;
    
    summary_json << "{";
    
    for (size_t index = 0; index < drone_names_.size(); ++index) {
      const auto& drone_name = drone_names_[index];
      const auto& state = states_[drone_name];
      
      summary_json << "\"" << drone_name << "\":{";
      summary_json << "\"battery\":" << drone_utils::format_float(static_cast<float>(state.battery), 1) << ",";
      summary_json << "\"drain_rate\":" << drone_utils::format_float(static_cast<float>(state.drain_rate), 2) << ",";
      summary_json << "\"status\":\"" << state.status << "\"";
      summary_json << "}";
      
      if (index + 1 < drone_names_.size()) {
        summary_json << ",";
      }
    }
    
    summary_json << "}";

    std_msgs::msg::String summary_message;
    summary_message.data = summary_json.str();
    summary_publisher_->publish(summary_message);
  }

  const std::vector<std::string> drone_names_{"Alpha", "Beta", "Gamma"};
  std::map<std::string, HealthState> states_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr warning_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<HealthMonitor>();
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}