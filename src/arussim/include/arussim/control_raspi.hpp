#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <sys/types.h>

#include <atomic>
#include <optional>
#include <string>

namespace control_raspi
{

class ControlRaspi : public rclcpp::Node
{
public:
  explicit ControlRaspi(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void launch_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void reset_callback(const std_msgs::msg::Bool::SharedPtr msg);

  void start_control_rasp();
  void stop_control_rasp();

  std::string resolve_control_rasp_path() const;
  std::string resolve_control_rasp_working_dir(const std::string & control_rasp_path) const;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr launch_patata_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_patata_sub_;

  std::optional<pid_t> control_rasp_pid_;
  std::atomic<bool> stopping_{false};
};

}  // namespace control_raspi
