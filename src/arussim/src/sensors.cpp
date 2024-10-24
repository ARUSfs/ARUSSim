/**
 * @file sensors.cpp
 * @author Rafael Guil (rafaguilvalero@gmail.com)
 * @brief Sensors node for the ARUSSIM package. This node simulates the sensors of the vehicle.
 * @version 0.1
 * @date 2024-10-16
 * 
 */
#include "arussim/sensors.hpp"
#include <random>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>

/**
 * @class Sensors
 * @brief Sensors class for ARUSsim
 * This class simulates some sensors of the vehicle like the IMU, wheel speed sensors and extensometer.
 */
Sensors::Sensors() : Node("sensors")
{
    // Declare and get noise parameters for each IMU variable
    this->declare_parameter<double>("imu.noise_imu_yaw", 0.01);
    this->declare_parameter<double>("imu.noise_imu_ax", 0.01);
    this->declare_parameter<double>("imu.noise_imu_ay", 0.01);
    this->declare_parameter<double>("imu.noise_imu_r", 0.01);
    this->declare_parameter<double>("imu.imu_frequency", 100.0);

    // Declare wheel speed parameters
    this->declare_parameter<double>("wheel_speed.noise_wheel_speed_front_right", 0.01);
    this->declare_parameter<double>("wheel_speed.noise_wheel_speed_front_left", 0.01);
    this->declare_parameter<double>("wheel_speed.noise_wheel_speed_rear_right", 0.01);
    this->declare_parameter<double>("wheel_speed.noise_wheel_speed_rear_left", 0.01);
    this->declare_parameter<double>("wheel_speed.wheel_speed_frequency", 100.0);

    // Declare extensometer parameters
    this->declare_parameter<double>("extensometer.extensometer_frequency", 100.0);
    this->declare_parameter<double>("extensometer.noise_extensometer", 0.01);


    // Get parameters
    this->get_parameter("imu.noise_imu_yaw", kNoiseImuYaw);
    this->get_parameter("imu.noise_imu_ax", kNoiseImuAx);
    this->get_parameter("imu.noise_imu_ay", kNoiseImuAy);
    this->get_parameter("imu.noise_imu_r", kNoiseImuR);
    this->get_parameter("imu.imu_frequency", kImuFrequency);

    this->get_parameter("wheel_speed.noise_wheel_speed_front_right", kNoiseWheelSpeedFrontRight);
    this->get_parameter("wheel_speed.noise_wheel_speed_front_left", kNoiseWheelSpeedFrontLeft);
    this->get_parameter("wheel_speed.noise_wheel_speed_rear_right", kNoiseWheelSpeedRearRight);
    this->get_parameter("wheel_speed.noise_wheel_speed_rear_left", kNoiseWheelSpeedRearLeft);
    this->get_parameter("wheel_speed.wheel_speed_frequency", kWheelSpeedFrequency);

    this->get_parameter("extensometer.extensometer_frequency", kExtensometerFrequency);
    this->get_parameter("extensometer.noise_extensometer", kNoiseExtensometer);


    // State subscriber
    state_sub_ = this->create_subscription<arussim_msgs::msg::State>(
        "/arussim/state", 1, 
        std::bind(&Sensors::state_callback, this, std::placeholders::_1)
    );

    // IMU
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/arussim/imu", 10);

    imu_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kImuFrequency)),
        std::bind(&Sensors::imu_timer, this)
    );

    // Wheel speed
    ws_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>("/arussim/wheel_speeds", 10);

    ws_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kWheelSpeedFrequency)),
        std::bind(&Sensors::wheel_speed_timer, this)
    );

    // Extensometer
    ext_pub_ = this->create_publisher<std_msgs::msg::Float32>("/arussim/extensometer", 10);

    ext_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kExtensometerFrequency)),
        std::bind(&Sensors::extensometer_timer, this)
    );
}

/**
 * @brief Callback function for the state subscriber
 * 
 * @param msg 
 */
void Sensors::state_callback(const arussim_msgs::msg::State::SharedPtr msg)
{
    // Update state variables with incoming data
    x_ = msg->x;
    y_ = msg->y;
    yaw_ = msg->yaw;
    vx_ = msg->vx;
    vy_ = msg->vy;
    r_ = msg->r;
}

/**
 * @brief Timer function for the IMU
 * 
 */
void Sensors::imu_timer()
{
    // Random noise generation with different noise for each variable
    std::random_device rd;
    std::mt19937 gen(rd());

    std::normal_distribution<> dist_yaw(0.0, kNoiseImuYaw);
    std::normal_distribution<> dist_ax(0.0, kNoiseImuAx);
    std::normal_distribution<> dist_ay(0.0, kNoiseImuAy);
    std::normal_distribution<> dist_r(0.0, kNoiseImuR);

    // Apply noise to the state variables
    noisy_yaw_ = yaw_ + dist_yaw(gen);
    noisy_ax_ = ax_ + dist_ax(gen);
    noisy_ay_ = ay_ + dist_ay(gen);
    noisy_r_ = r_ + dist_r(gen);

    // Create the IMU message
    auto message = sensor_msgs::msg::Imu();

    // Convert yaw (Euler angle) to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, noisy_yaw_); // Roll and pitch are 0 since you're only working with yaw
    message.orientation.x = q.x();
    message.orientation.y = q.y();
    message.orientation.z = q.z();
    message.orientation.w = q.w();

    // Fill in the angular velocity
    message.angular_velocity.x = 0.0;  // Roll is 0, no roll velocity
    message.angular_velocity.y = 0.0;  // Pitch is 0, no pitch velocity
    message.angular_velocity.z = noisy_r_;   // Yaw rate (r_) goes here

    // Fill in the linear acceleration
    message.linear_acceleration.x = noisy_ax_;  // Linear acceleration in X
    message.linear_acceleration.y = noisy_ay_;  // Linear acceleration in Y
    message.linear_acceleration.z = 0.0;  // No acceleration in Z, so it's 0

    // Publish the IMU message
    imu_pub_->publish(message);
}

/**
 * @brief Timer function for the wheel speed sensors
 * 
 */
void Sensors::wheel_speed_timer()
{
    // Random noise generation with different noise for each wheel speed
    std::random_device rd; 
    std::mt19937 gen(rd());
    std::normal_distribution<> dist_front_right(0.0, kNoiseWheelSpeedFrontRight);
    std::normal_distribution<> dist_front_left(0.0, kNoiseWheelSpeedFrontLeft);
    std::normal_distribution<> dist_rear_right(0.0, kNoiseWheelSpeedRearRight);
    std::normal_distribution<> dist_rear_left(0.0, kNoiseWheelSpeedRearLeft);

    // Apply noise to the state variables
    speed_front_right_ = std::sqrt(vx_ * vx_ + vy_ * vy_) + dist_front_right(gen);
    speed_front_left_ = std::sqrt(vx_ * vx_ + vy_ * vy_) + dist_front_left(gen);
    speed_rear_right_ = std::sqrt(vx_ * vx_ + vy_ * vy_) + dist_rear_right(gen);
    speed_rear_left_ = std::sqrt(vx_ * vx_ + vy_ * vy_) + dist_rear_left(gen);

    // Create the wheel speed message
    auto message = arussim_msgs::msg::FourWheelDrive();

    message.front_right = speed_front_right_;    // Speed until physics is created
    message.front_left = speed_front_left_;      // Speed until physics is created
    message.rear_right = speed_rear_right_;      // Speed until physics is created   
    message.rear_left = speed_rear_left_;        // Speed until physics is created    

    // Publish the wheel speed message
    ws_pub_->publish(message);
}

/**
 * @brief Timer function for the extensometer
 * 
 */
void Sensors::extensometer_timer()
{
    // Random noise generation
    std::random_device rd; 
    std::mt19937 gen(rd());
    std::normal_distribution<> dist(0.0, kNoiseExtensometer);

    // Apply noise to the state variables
    noisy_delta_ = delta_ + dist(gen);

    // Create the extensometer message
    auto message = std_msgs::msg::Float32();

    // 0 until physics is created
    message.data = noisy_delta_;

    // Publish the extensometer message
    ext_pub_->publish(message);
}

/**
 * @brief Main function
 * 
 * This initializes the ROS 2 system and starts spinning the Sensor node.
 * 
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return int Exit status of the application. 
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sensors>());
  rclcpp::shutdown();
  return 0;
}