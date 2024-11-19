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

    reset_sub_ = this->create_subscription<std_msgs::msg::Bool>("/arussim/reset", 1, 
        std::bind(&Supervisor::reset_callback, this, std::placeholders::_1));

    lap_time_pub_ = this->create_publisher<std_msgs::msg::Float32>("/arussim/lap_time", 1);

    hit_cones_pub_ = this->create_publisher<std_msgs::msg::Bool>("/arussim/hit_cones_bool", 1);

    set_timer_service_ = this->create_service<arussim_msgs::srv::SetTimer>(
        "arussim/set_timer",
        std::bind(&Supervisor::handle_set_timer, this, 
            std::placeholders::_1, std::placeholders::_2));
}

/**
 * @brief Service handler for setting the timer.
 * 
 * This method updates the timer parameter based on a service request.
 * 
 * @param request The service request message.
 * @param response The service response message.
 */
void Supervisor::handle_set_timer(
    const std::shared_ptr<arussim_msgs::srv::SetTimer::Request> request,
    std::shared_ptr<arussim_msgs::srv::SetTimer::Response> response)
{
    try {
        kSimulationSpeedMultiplier = request->timer;
        response->success = true;
        response->message = "timer updated successfully to " + std::to_string(kSimulationSpeedMultiplier);
    } catch (const std::exception& e) {
        response->success = false;
        response->message = "Error updating timer: " + std::string(e.what());
        RCLCPP_ERROR(get_logger(), "Error updating timer: %s", e.what());
    }
}

/**
 * @brief Callback for receiving reset commands.
 * 
 * This method resets the lap times.
 * 
 * @param msg 
 */
void Supervisor::reset_callback([[maybe_unused]]const std_msgs::msg::Bool::SharedPtr msg)
{
    time_list_.clear();
    list_total_hit_cones_.clear();
    hit_cones_lap_.clear();
    started_ = false;
}

/**
 * @brief Callback to check if the car is between two TPLs.
 * 
 * @param msg 
 */
void Supervisor::tpl_signal_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!started_){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%sLap started%s", green.c_str(), reset.c_str());
        started_ = true;
    }
    else{
        time_list_.push_back((this->get_clock()->now().seconds() - prev_time_) * kSimulationSpeedMultiplier);

        std_msgs::msg::Float32 lap_time_msg;
        lap_time_msg.data = time_list_.back();
        lap_time_pub_->publish(lap_time_msg);

        // Detect hit cones in this lap and add to total
        list_total_hit_cones_.push_back(hit_cones_lap_);
        hit_cones_lap_.clear();

        size_t n_total_cones_hit_ = 0;
        for (const auto& i : list_total_hit_cones_) {
            n_total_cones_hit_ += i.size();
        }
        if(n_total_cones_hit_ == 0){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%sLAP %zu: %f. TOTAL HIT CONES: %zu%s", 
            green.c_str(), time_list_.size(), time_list_.back(), n_total_cones_hit_, reset.c_str());
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%sLAP %zu: %f. %sTOTAL HIT CONES: %zu%s", 
            green.c_str(), time_list_.size(), time_list_.back(), yellow.c_str(), n_total_cones_hit_, reset.c_str());
        }
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
        RCLCPP_INFO(this->get_logger(), "%sHit cones: %zu%s", 
        yellow.c_str(), hit_cones_lap_.size(), reset.c_str());

        std_msgs::msg::Bool hit_cones_msg;
        hit_cones_msg.data = true;
        hit_cones_pub_->publish(hit_cones_msg);
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
