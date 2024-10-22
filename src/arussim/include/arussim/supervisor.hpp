/**
 * @file supervisor.hpp
 * @author Rafael Guil (rafaguilvalero@gmail.com)
 * @brief Time per lap header file.
 * @date 2024-10-21
 * 
 */
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include <random>

/**
 * @class Supervisor
 * @brief Time per lap class for the ARUSSIM package.
 * 
 * This class simulates the Time per lap of the vehicle on the track.
 */
class Supervisor : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Supervisor object
     * 
     */
    Supervisor();

private:
    /**
     * @brief Callback to check if the car is between two TPLs
     * 
     * @param msg 
     */
    void tpl_signal_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg);

    //Variables
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr between_tpl_sub_;

    bool between_tpl_;
    bool started_;

    double prev_time;

    std::vector<double> time_list_;

};