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
 * @brief Callback to check if the car is between two TPLs.
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

        // Detect hit cones in this lap and add to total
        list_total_hit_cones_.push_back(hit_cones_lap_);
        hit_cones_lap_.clear();

        for (const auto& i : list_total_hit_cones_) {
            n_total_cones_hit_ += i.size();
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LAP %zu: %f. TOTAL HIT CONES: %zu", time_list_.size(), time_list_.back(), n_total_cones_hit_);
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
    auto cone_position = std::make_pair(static_cast<double>(msg->x), static_cast<double>(msg->y));

    if (std::find(hit_cones_lap_.begin(), hit_cones_lap_.end(), cone_position) == hit_cones_lap_.end()) {
        hit_cones_lap_.push_back(cone_position);
        RCLCPP_WARN(this->get_logger(), "Hit cones: %zu", hit_cones_lap_.size());
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
