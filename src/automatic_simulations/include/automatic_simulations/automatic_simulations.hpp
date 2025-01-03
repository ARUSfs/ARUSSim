/**
 * @file automatic_simulations.hpp
 * @author Rafael Guil (rafaguilvalero@gmail.com)
 * @brief Header file for automatic_simulations.cpp.
 * @version 0.1
 * @date 2024-12-28
 */
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include <string>

/**
 * @class AutomaticSimulations
 * @brief AutomaticSimulations class that declares variables
 */
class AutomaticSimulations : public rclcpp::Node
{
public:
    AutomaticSimulations();

private:
    std::string kEvent;
    double kSimulationSpeedMultiplier;
    double kLapsTarget;

    int current_laps_ = 0;
    int current_iterations_ = 0;

    void lap_time_callback(const std_msgs::msg::Float32::SharedPtr msg);


    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lap_time_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    //Loginfo colors
    const std::string red = "\033[1;31m";
    const std::string green = "\033[1;32m";
    const std::string yellow = "\033[1;33m";
    const std::string blue = "\033[1;34m";
    const std::string magenta = "\033[1;35m";
    const std::string cyan = "\033[1;36m";
    const std::string reset = "\033[0m";

};