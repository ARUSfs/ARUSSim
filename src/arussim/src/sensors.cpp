#include "arussim/sensors.hpp"
#include <random>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>

Sensors::Sensors() : Node("sensors")
{
    // Declare and get noise parameter
    this->declare_parameter<double>("sensor.noise_sigma", 0.01);
    this->declare_parameter<double>("sensor.imu_frequency", 0.01);
    this->declare_parameter<double>("sensor.wheel_speed_frequency", 0.01);
    this->declare_parameter<double>("sensor.extensometer_frequency", 0.01);


    this->get_parameter("sensor.noise_sigma", kNoiseSensor);
    this->get_parameter("sensor.imu_frequency", kImuFrequency);
    this->get_parameter("sensor.wheel_speed_frequency", kWheelSpeedFrequency);
    this->get_parameter("sensor.extensometer_frequency", kExtensometerFrequency);


    // Create State subscriber
    state_sub_ = this->create_subscription<custom_msgs::msg::State>(
        "/arussim/state", 1, 
        std::bind(&Sensors::state_callback, this, std::placeholders::_1)
    );

    // Create Cmd subscriber
    cmd_sub_ = this->create_subscription<custom_msgs::msg::Cmd>(
        "/arussim/cmd", 1, 
        std::bind(&Sensors::cmd_callback, this, std::placeholders::_1)
    );

    // IMU
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/sensors/imu", 10);

    imu_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kImuFrequency)),
        std::bind(&Sensors::imu, this)
    );

    // Wheel speed
    ws_pub_ = this->create_publisher<custom_msgs::msg::FourWheelDrive>("/sensors/wheel_speeds", 10);

    ws_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kWheelSpeedFrequency)),
        std::bind(&Sensors::wheel_speed, this)
    );

    // Extensometer
    ext_pub_ = this->create_publisher<std_msgs::msg::Float32>("/sensors/extensometer", 10);

    ext_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kExtensometerFrequency)),
        std::bind(&Sensors::extensometer, this)
    );
}

void Sensors::state_callback(const custom_msgs::msg::State::SharedPtr msg)
{
    // Update state variables with incoming data
    x_ = msg->x;
    y_ = msg->y;
    yaw_ = msg->yaw;
    vx_ = msg->vx;
    vy_ = msg->vy;
    r_ = msg->r;
}

void Sensors::cmd_callback(const custom_msgs::msg::Cmd::SharedPtr msg)
{
    delta_ = msg->delta;
}

void Sensors::imu()
{
    // Random noise generation
    std::random_device rd; 
    std::mt19937 gen(rd());
    std::normal_distribution<> dist(0.0, kNoiseSensor);

    // Apply noise to the state variables
    x_ += dist(gen);   
    y_ += dist(gen);   
    yaw_ += dist(gen); 
    vx_ += dist(gen);  
    vy_ += dist(gen);  
    r_ += dist(gen);

    // Create the IMU message
    auto message = sensor_msgs::msg::Imu();

    // Convert yaw (Euler angle) to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_); // Roll and pitch are 0 since you're only working with yaw
    message.orientation.x = q.x();
    message.orientation.y = q.y();
    message.orientation.z = q.z();
    message.orientation.w = q.w();

    // Fill in the angular velocity
    message.angular_velocity.x = 0.0;  // Roll is 0, no roll velocity
    message.angular_velocity.y = 0.0;  // Pitch is 0, no pitch velocity
    message.angular_velocity.z = r_;   // Yaw rate (r_) goes here

    // Fill in the linear acceleration
    message.linear_acceleration.x = vx_;  // Linear acceleration in X
    message.linear_acceleration.y = vy_;  // Linear acceleration in Y
    message.linear_acceleration.z = 0.0;  // No acceleration in Z, so it's 0

    // Publish the IMU message
    imu_pub_->publish(message);
}

void Sensors::wheel_speed()
{
    // Random noise generation
    std::random_device rd; 
    std::mt19937 gen(rd());
    std::normal_distribution<> dist(0.0, kNoiseSensor);

    // Apply noise to the state variables
    vx_ += dist(gen);
    vy_ += dist(gen);

    double speed = std::sqrt(vx_ * vx_ + vy_ * vy_);

    // Create the wheel speed message
    auto message = custom_msgs::msg::FourWheelDrive();

    message.front_right = speed;    // Speed until physics is created
    message.front_left = speed;     // Speed until physics is created     
    message.rear_right = speed;     // Speed until physics is created
    message.rear_left = speed;      // Speed until physics is created

    // Publish the wheel speed message
    ws_pub_->publish(message);
}

void Sensors::extensometer()
{
    // Random noise generation
    std::random_device rd; 
    std::mt19937 gen(rd());
    std::normal_distribution<> dist(0.0, kNoiseSensor);

    // Apply noise to the state variables
    delta_ += dist(gen);

    // Create the extensometer message
    auto message = std_msgs::msg::Float32();

    // speed until physics is created
    message.data = delta_;

    // Publish the extensometer message
    ext_pub_->publish(message);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sensors>());
  rclcpp::shutdown();
  return 0;
}