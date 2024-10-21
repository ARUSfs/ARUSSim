#include <rclcpp/rclcpp.hpp>
#include "custom_msgs/msg/state.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "std_msgs/msg/bool.hpp"
#include <vector>
#include <ctime>
#include <numeric>
#include <cmath> 
#include <random>

class Tpl : public rclcpp::Node
{
public:
    Tpl();

private:
    void between_tpl_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void state_callback(const custom_msgs::msg::State::SharedPtr msg);
    void tpl_timer();

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr between_tpl_sub_;
    rclcpp::Subscription<custom_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr tpl_timer_;
    rclcpp::TimerBase::SharedPtr tpl_timer_acc_;

    std::string kTrackName;
    bool between_tpl_;
    double x_, y_, yaw_, vx_, vy_, r_ = 0;
    double total_time_list = 0;
    double diff_time_ = 0;

    rclcpp::Time current_time_;
    rclcpp::Time start_time_;

    std::vector<double> time_list_;

};