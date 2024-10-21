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

class Supervisor : public rclcpp::Node
{
public:
    Supervisor();

private:
    void between_tpl_callback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr between_tpl_sub_;

    bool between_tpl_;
    bool started_;

    double prev_time;

    std::vector<double> time_list_;

};