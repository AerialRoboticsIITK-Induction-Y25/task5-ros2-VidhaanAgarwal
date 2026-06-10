#include "mission_drone.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string format_number(double value, int precision = 1) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

std::string make_waypoint_label(const MissionDrone& drone) {
  int current_waypoint = drone.get_current_waypoint_index();
  int total_waypoints = drone.get_waypoints().size();
  current_waypoint = std::min(current_waypoint, total_waypoints);
  
  return std::to_string(current_waypoint) + "/" + std::to_string(total_waypoints);
}

std::string make_json_payload(
    const MissionDrone& drone,
    const std::string& name,
    const std::string& mission_name) {
  std::ostringstream json_stream;
  
  json_stream << "{";
  json_stream << "\"name\":\"" << name << "\",";
  json_stream << "\"mission_name\":\"" << mission_name << "\",";
  json_stream << "\"battery\":" << format_number(drone.get_battery_level(), 1) << ",";
  json_stream << "\"altitude\":" << format_number(drone.get_altitude(), 1) << ",";
  json_stream << "\"status\":\"" << drone.get_status() << "\",";
  json_stream << "\"waypoint\":\"" << make_waypoint_label(drone) << "\",";
  json_stream << "\"speed\":" << format_number(drone.get_speed(), 1);
  json_stream << "}";
  
  return json_stream.str();
}

}  // namespace

class DroneNode : public rclcpp::Node {
public:
  DroneNode() : Node("drone_node") {
    this->declare_parameters();
    this->setup_waypoints();
    this->reset_mission();
    this->create_publishers();
    this->create_timers();
  }

private:
  void declare_parameters() {
    drone_name_ = this->declare_parameter<std::string>("drone_name", "Alpha");
    initial_battery_ = this->declare_parameter<double>("initial_battery", 100.0);
    mission_name_ = this->declare_parameter<std::string>("mission_name", drone_name_ + " Mission");
  }

  void setup_waypoints() {
    waypoints_.clear();
    waypoints_.push_back(Waypoint{0.0f, 0.0f, 10.0f});
    waypoints_.push_back(Waypoint{4.0f, 2.0f, 15.0f});
    waypoints_.push_back(Waypoint{8.0f, 4.0f, 20.0f});
    waypoints_.push_back(Waypoint{12.0f, 6.0f, 25.0f});
    waypoints_.push_back(Waypoint{16.0f, 8.0f, 30.0f});
  }

  void reset_mission() {
    float battery_level = static_cast<float>(initial_battery_);
    mission_ = MissionDrone(drone_name_, mission_name_, waypoints_, battery_level, 120.0f);
    publish_count_ = 0;
    alert_sent_ = false;
  }

  void create_publishers() {
    std::string topic_prefix = "/drone/" + drone_name_;
    
    status_publisher_ = this->create_publisher<std_msgs::msg::String>(
        topic_prefix + "/status", 10);
    alert_publisher_ = this->create_publisher<std_msgs::msg::String>(
        topic_prefix + "/alert", 10);
    mission_complete_publisher_ = this->create_publisher<std_msgs::msg::String>(
        topic_prefix + "/mission_complete", 10);
    telemetry_publisher_ = this->create_publisher<std_msgs::msg::String>(
        topic_prefix + "/telemetry", 10);
  }

  void create_timers() {
    status_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&DroneNode::on_status_timer, this));
    
    telemetry_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&DroneNode::on_telemetry_timer, this));
  }

  void on_status_timer() {
    this->handle_mission_completion();
    this->drain_battery_for_status();
    this->advance_waypoint_if_needed();
    this->check_battery_critical();
    this->publish_status_message();
  }

  void handle_mission_completion() {
    if (mission_.mission_complete()) {
      std_msgs::msg::String complete_message;
      complete_message.data = drone_name_ + " mission_complete";
      mission_complete_publisher_->publish(complete_message);
      this->reset_mission();
    }
  }

  void drain_battery_for_status() {
    try {
      mission_.drain_battery(0.5f);
    } catch (const BatteryDepletedError& error) {
      RCLCPP_ERROR(this->get_logger(), "Battery depleted: %s", error.what());
      mission_.emergency_stop();
    }
  }

  void advance_waypoint_if_needed() {
    publish_count_++;
    
    bool should_advance_waypoint = (publish_count_ % 3 == 0);
    bool not_at_mission_end = !mission_.mission_complete();
    
    if (should_advance_waypoint && not_at_mission_end) {
      try {
        mission_.next_waypoint();
      } catch (const DroneException& error) {
        RCLCPP_WARN(this->get_logger(), "Waypoint advance failed for %s: %s",
                    drone_name_.c_str(), error.what());
      }
    }
  }

  void check_battery_critical() {
    if (mission_.is_critical() && !alert_sent_) {
      std::string battery_value = format_number(mission_.get_battery_level(), 1);
      std::string alert_msg = "CRITICAL battery for " + drone_name_ + ": " + battery_value;
      
      std_msgs::msg::String alert_message;
      alert_message.data = alert_msg;
      alert_publisher_->publish(alert_message);
      
      mission_.land();
      alert_sent_ = true;
    }
  }

  void publish_status_message() {
    if (mission_.mission_complete()) {
      std_msgs::msg::String complete_message;
      complete_message.data = drone_name_ + " mission_complete";
      mission_complete_publisher_->publish(complete_message);
      this->reset_mission();
      return;
    }

    std_msgs::msg::String status_message;
    std::string battery_str = format_number(mission_.get_battery_level(), 1);
    std::string altitude_str = format_number(mission_.get_altitude(), 1);
    std::string speed_str = format_number(mission_.get_speed(), 1);
    std::string waypoint_str = make_waypoint_label(mission_);
    
    status_message.data = "name:" + drone_name_ +
                          "|battery:" + battery_str +
                          "|altitude:" + altitude_str +
                          "|status:" + mission_.get_status() +
                          "|waypoint:" + waypoint_str +
                          "|speed:" + speed_str;
    
    status_publisher_->publish(status_message);
  }

  void on_telemetry_timer() {
    std_msgs::msg::String telemetry_message;
    telemetry_message.data = make_json_payload(mission_, drone_name_, mission_name_);
    telemetry_publisher_->publish(telemetry_message);
  }

  std::string drone_name_;
  double initial_battery_;
  std::string mission_name_;
  std::vector<Waypoint> waypoints_;
  MissionDrone mission_{"temp", "temp", {}, 100.0f, 120.0f};
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_complete_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_publisher_;
  
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  
  int publish_count_{0};
  bool alert_sent_{false};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<DroneNode>();
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}