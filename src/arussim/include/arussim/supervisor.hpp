/**
 * @file supervisor.hpp
 * @author Rafael Guil (rafaguilvalero@gmail.com)
 * @brief Time per lap header file.
 * @date 2024-10-21
 * 
 */
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "arussim_msgs/msg/point_xy.hpp"
#include "std_msgs/msg/float32.hpp"
#include <algorithm>
#include <vector>
#include <utility>
#include "arussim_msgs/srv/set_timer.hpp"
#include "arussim/csv_generator.hpp"

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

    /**
     * @brief Callback to check if the car has hit a cone
     * 
     * @param msg 
     */
    void hit_cones_callback(const arussim_msgs::msg::PointXY::SharedPtr msg);

    /**
     * @brief Callback for receiving reset commands.
     * 
     * This method resets the lap times.
     * 
     * @param msg The reset command message.
     */
    void reset_callback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Service handler for setting the timer.
     * 
     * @param request 
     * @param response 
     */
    void handle_set_timer(
        const std::shared_ptr<arussim_msgs::srv::SetTimer::Request> request,
        std::shared_ptr<arussim_msgs::srv::SetTimer::Response> response);

    /**
     * @brief Timer callback to update the timer.
     * 
     */
    void timer_callback();



    //Publishers and subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr between_tpl_sub_;
    rclcpp::Subscription<arussim_msgs::msg::PointXY>::SharedPtr hit_cones_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hit_cones_pub_;
    rclcpp::Service<arussim_msgs::srv::SetTimer>::SharedPtr set_timer_service_;
    rclcpp::TimerBase::SharedPtr timer_;


    //Variables
    double simulation_speed_multiplier = 1.0;
    std::vector<double> speed_multiplier_list_;
    double mean_;

    bool between_tpl_;
    bool started_;
    
    double prev_time_;

    CSVGenerator csv_generator_{"supervisor"};

    //Loginfo colors
    const std::string red = "\033[1;31m";
    const std::string green = "\033[1;32m";
    const std::string yellow = "\033[1;33m";
    const std::string blue = "\033[1;34m";
    const std::string magenta = "\033[1;35m";
    const std::string cyan = "\033[1;36m";
    const std::string reset = "\033[0m";

    std::vector<double> time_list_;
    std::vector<std::pair<double, double>> hit_cones_lap_;
    std::vector<std::vector<std::pair<double, double>>> list_total_hit_cones_;
};