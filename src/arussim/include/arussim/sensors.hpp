#include <rclcpp/rclcpp.hpp>
#include "custom_msgs/msg/state.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/msg/imu.hpp>
#include "custom_msgs/msg/four_wheel_drive.hpp"
#include "std_msgs/msg/float32.hpp"
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

    double noisy_vx_ = 0;
    double noisy_vy_ = 0;
    double noisy_r_ = 0;
    double noisy_yaw_ = 0;
    double noisy_delta_ = 0;

    double delta_ = 0;
    double speed_front_right_ = 0;
    double speed_front_left_ = 0;
    double speed_rear_right_ = 0;
    double speed_rear_left_ = 0;

    double kExtensometerFrequency;
    double kNoiseExtensometer;

    double kWheelSpeedFrequency;
    double kNoiseWheelSpeedFrontRight;
    double kNoiseWheelSpeedFrontLeft;
    double kNoiseWheelSpeedRearRight;
    double kNoiseWheelSpeedRearLeft;

    double kImuFrequency;
    double kNoiseImuX;
    double kNoiseImuY;
    double kNoiseImuYaw;
    double kNoiseImuVx;
    double kNoiseImuVy;
    double kNoiseImuR;

    // Functions
    void state_callback(const custom_msgs::msg::State::SharedPtr msg);
    void extensometer_timer();
    void wheel_speed_timer();
    void imu_timer();
    
    
    // ROS Communication
    rclcpp::Subscription<custom_msgs::msg::State>::SharedPtr state_sub_; // State subscriber

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_; // IMU publisher
    rclcpp::TimerBase::SharedPtr imu_timer_; // IMU timer

    rclcpp::Publisher<custom_msgs::msg::FourWheelDrive>::SharedPtr ws_pub_; // Wheel speed publisher
    rclcpp::TimerBase::SharedPtr ws_timer_; // Wheel speed timer

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ext_pub_; // Wheel speed publisher
    rclcpp::TimerBase::SharedPtr ext_timer_; // Wheel speed timer
};

