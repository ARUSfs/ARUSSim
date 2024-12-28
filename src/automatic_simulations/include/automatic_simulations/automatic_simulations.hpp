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
    double kIterations;
    double kLapsTarget;

    int current_laps_ = 0;
    int current_iterations_ = 0;

    void lap_time_callback(const std_msgs::msg::Float32::SharedPtr msg);


    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lap_time_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

};