/**
 * @file supervisor.cpp
 * @author Rafael Guil (rafaguilvalero@gmail.com)
 * @version 0.1
 * @date 2024-10-19
 * 
 * 
 */
#include "arussim/supervisor.hpp"
#include <random>


/**
 * @class Supervisor
 * @brief Constructor for the Supervisor class
 */
Supervisor::Supervisor() : Node("Supervisor")
{
    between_tpl_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/arussim/tpl_signal", 10, 
        std::bind(&Supervisor::tpl_signal_callback, this, std::placeholders::_1)
    );

    hit_cones_sub_ = this->create_subscription<arussim_msgs::msg::PointXY>(
        "/arussim/hit_cones", 10,
        std::bind(&Supervisor::hit_cones_callback, this, std::placeholders::_1)
    );
}

/**
 * @brief Callback to check if the car is between two TPLs and calculate lap time.
 * 
 * @param msg 
 */
void Supervisor::tpl_signal_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!started_){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lap started");
        started_ = true;
    }
    else{
        time_list_.push_back(this->get_clock()->now().seconds() - prev_time_);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lap %zu: %f", time_list_.size(), time_list_.back());
    }
    prev_time_ = this->get_clock()->now().seconds();
}

/**
 * @brief Callback to check if the car has hit a cone.
 * 
 * @param msg 
 */
void Supervisor::hit_cones_callback(const arussim_msgs::msg::PointXY::SharedPtr msg)
{
    hit_cones_.insert(std::make_pair(msg->x, msg->y));

    static size_t previous_size_ = 0;
    if (hit_cones_.size() != previous_size_) {
        RCLCPP_INFO(this->get_logger(), "Hit cones: %ld", hit_cones_.size());
        previous_size_ = hit_cones_.size();
    }

}

/**
 * @brief Main function for the supervisor node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Supervisor>());
    rclcpp::shutdown();
    return 0;
}
