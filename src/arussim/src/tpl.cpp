/**
 * @file tpl.cpp
 * @author Rafael Guil (rafaguilvalero@gmail.com)
 * @brief Time per lap node for the ARUSSIM package. This node simulates the Time per lap of the vehicle on the track.
 * @version 0.1
 * @date 2024-10-19
 * 
 * 
 */
#include "arussim/tpl.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <random>


/**
 * @class Tpl
 * @brief Constructor for the Tpl class
 */
Tpl::Tpl() : Node("Time_per_lap")
{
    between_tpl_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/arussim/between_tpl", 10, 
        std::bind(&Tpl::between_tpl_callback, this, std::placeholders::_1)
    );

    state_sub_ = this->create_subscription<custom_msgs::msg::State>(
        "/arussim/state", 100, 
        std::bind(&Tpl::state_callback, this, std::placeholders::_1)
    );
    
    tpl_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(100)),
        std::bind(&Tpl::tpl_timer, this)
    );
}


void Tpl::between_tpl_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    between_tpl_ = msg->data;
}

void Tpl::state_callback(const custom_msgs::msg::State::SharedPtr msg)
{
    x_ = msg->x;
    y_ = msg->y;
    yaw_ = msg->yaw;
    vx_ = msg->vx;
    vy_ = msg->vy;
    r_ = msg->r;
}

/**
 * @brief Timer function for the time per lap
 * 
 */
void Tpl::tpl_timer()
{
    // Start the lap if the car is between the points and the list is empty (start of lap)
    if ((between_tpl_) && (time_list_.empty())) {
    //if (vx_ > 0.15 && time_list_.empty()) {
        start_time_ = this->get_clock()->now();    // Start the timer at the current time
        time_list_.push_back(0);  // Add 0 at the start of the lap
        RCLCPP_INFO(this->get_logger(), "Lap started");
    }

    // If the lap has already started
    if (!time_list_.empty()) {
        
        if (between_tpl_) {
            current_time_ = this->get_clock()->now();    // Start the timer at the current time
            total_time_list = std::accumulate(time_list_.begin(), time_list_.end(), 0.0);
            diff_time_ = current_time_.seconds() - start_time_.seconds() - total_time_list;

            if ((diff_time_ > 2) && ((time_list_.back() == 0) || (time_list_.back() > 2))){
                    time_list_.push_back(diff_time_);
                    RCLCPP_INFO(this->get_logger(), "Lap %ld: %f", time_list_.size() - 1, time_list_.back());
            }
            //RCLCPP_INFO(this->get_logger(), "Lap %ld: %f", time_list_.size() - 1, diff_time_);
        }
    }  

    // If 10 laps have been recorded
    if (time_list_.size() - 1 == 10) {
        double total_time = std::accumulate(time_list_.begin(), time_list_.end(), 0.0);
        RCLCPP_INFO(this->get_logger(), "Total trackdrive time: %f", total_time);
    }
}

/**
 * @brief Main function for the Tpl node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tpl>());
    rclcpp::shutdown();
    return 0;
}
