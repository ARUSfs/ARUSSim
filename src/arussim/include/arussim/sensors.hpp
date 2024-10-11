#include <rclcpp/rclcpp.hpp>
#include "custom_msgs/msg/state.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/msg/imu.hpp>
#include <random>

class Sensors : public rclcpp::Node
{
public:
    Sensors(); // Constructor

private:
    // Variables
    double x_ = 0;
    double y_ = 0;
    double yaw_ = 0;
    double vx_ = 0;
    double vy_ = 0;
    double r_ = 0;

    double kNoiseSensor;
    std::string kTrackName;

    // Functions
    void imu();
    void state_callback(const custom_msgs::msg::State::SharedPtr msg);
    
    // ROS Communication
    rclcpp::Subscription<custom_msgs::msg::State>::SharedPtr state_sub_; // State subscriber
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_; // IMU publisher
    rclcpp::TimerBase::SharedPtr imu_timer_; // IMU timer
};

