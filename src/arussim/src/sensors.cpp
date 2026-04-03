/**
 * @file sensors.cpp
 * @author Rafael Guil (rafaguilvalero@gmail.com)
 * @brief Sensors node for the ARUSSIM package. This node simulates the sensors of the vehicle.
 * @version 0.1
 * @date 2024-10-16
 * 
 */
#include "arussim/sensors.hpp"

/**
 * @class Sensors
 * @brief Sensors class for ARUSsim
 * This class simulates some sensors of the vehicle like the IMU, wheel speed sensors and extensometer.
 */
Sensors::Sensors() : Node("sensors")
{
    // Declare ground speed parameters
    this->declare_parameter<double>("gss.noise_gss_vx", 0.01);
    this->declare_parameter<double>("gss.noise_gss_vy", 0.01);
    this->declare_parameter<double>("gss.gss_frequency", 200.0);
    
    // Declare and get noise parameters for each IMU variable
    this->declare_parameter<double>("imu.noise_imu_ax", 0.01);
    this->declare_parameter<double>("imu.noise_imu_ay", 0.01);
    this->declare_parameter<double>("imu.noise_imu_r", 0.01);
    this->declare_parameter<double>("imu.imu_frequency", 200.0);

    // Declare wheel speed parameters
    this->declare_parameter<double>("inverter.noise_wheel_speed_front_right", 0.01);
    this->declare_parameter<double>("inverter.noise_wheel_speed_front_left", 0.01);
    this->declare_parameter<double>("inverter.noise_wheel_speed_rear_right", 0.01);
    this->declare_parameter<double>("inverter.noise_wheel_speed_rear_left", 0.01);
    this->declare_parameter<double>("inverter.noise_torque_front_right", 0.01);
    this->declare_parameter<double>("inverter.noise_torque_front_left", 0.01);
    this->declare_parameter<double>("inverter.noise_torque_rear_right", 0.01);
    this->declare_parameter<double>("inverter.noise_torque_rear_left", 0.01);
    this->declare_parameter<double>("inverter.inverter_frequency", 200.0);

    // Declare extensometer parameters
    this->declare_parameter<double>("extensometer.extensometer_frequency", 100.0);
    this->declare_parameter<double>("extensometer.noise_extensometer", 0.01);


    // Get parameters
    this->get_parameter("gss.noise_gss_vx", kNoiseGssVx);
    this->get_parameter("gss.noise_gss_vy", kNoiseGssVy);
    this->get_parameter("gss.gss_frequency", kGssFrequency);
    
    this->get_parameter("imu.noise_imu_ax", kNoiseImuAx);
    this->get_parameter("imu.noise_imu_ay", kNoiseImuAy);
    this->get_parameter("imu.noise_imu_r", kNoiseImuR);
    this->get_parameter("imu.imu_frequency", kImuFrequency);

    this->get_parameter("inverter.noise_wheel_speed_front_right", kNoiseWheelSpeedFrontRight);
    this->get_parameter("inverter.noise_wheel_speed_front_left", kNoiseWheelSpeedFrontLeft);
    this->get_parameter("inverter.noise_wheel_speed_rear_right", kNoiseWheelSpeedRearRight);
    this->get_parameter("inverter.noise_wheel_speed_rear_left", kNoiseWheelSpeedRearLeft);
    this->get_parameter("inverter.noise_torque_front_right", kNoiseTorqueFrontRight);
    this->get_parameter("inverter.noise_torque_front_left", kNoiseTorqueFrontLeft);
    this->get_parameter("inverter.noise_torque_rear_right", kNoiseTorqueRearRight);
    this->get_parameter("inverter.noise_torque_rear_left", kNoiseTorqueRearLeft);
    this->get_parameter("inverter.inverter_frequency", kInverterFrequency);

    this->get_parameter("extensometer.extensometer_frequency", kExtensometerFrequency);
    this->get_parameter("extensometer.noise_extensometer", kNoiseExtensometer);


    // State subscriber
    state_sub_ = this->create_subscription<arussim_msgs::msg::State>(
        "/arussim/state", 1, 
        std::bind(&Sensors::state_callback, this, std::placeholders::_1)
    );

    // Groundspeed
    gss_vx_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/arussim/gss/vx", 10);
    gss_vy_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/arussim/gss/vy", 10);

    gss_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kGssFrequency)),
        std::bind(&Sensors::groundspeed_timer, this)
    );

    // IMU
    ax_pub_ = this->create_publisher<std_msgs::msg::Float32>("/arussim/IMU/ax", 10);
    ay_pub_ = this->create_publisher<std_msgs::msg::Float32>("/arussim/IMU/ay", 10);
    r_pub_ = this->create_publisher<std_msgs::msg::Float32>("/arussim/IMU/yaw_rate", 10);

    imu_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kImuFrequency)),
        std::bind(&Sensors::imu_timer, this)
    );

    // Inverter
    ws_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>(
        "/arussim/wheel_speed", 10);
    torque_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>(
        "/arussim/torque4WD", 10);
        
    inv_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kInverterFrequency)),
        std::bind(&Sensors::inverter_timer, this)
    );

    // Extensometer
    ext_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/arussim/extensometer", 10);

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
    ax_ = msg->ax;
    ay_ = msg->ay;
    delta_ = msg->delta;
    wheel_speed_msg_ = msg->wheel_speeds;
    torque_cmd_msg_ = msg->torque;
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

    std::normal_distribution<> dist_ax(0.0, kNoiseImuAx);
    std::normal_distribution<> dist_ay(0.0, kNoiseImuAy);
    std::normal_distribution<> dist_r(0.0, kNoiseImuR);

    // Create IMU data messages
    auto msg_ax = std_msgs::msg::Float32();
    auto msg_ay = std_msgs::msg::Float32();
    auto msg_r = std_msgs::msg::Float32();

    // Yaw rate
    msg_r.data = r_ + dist_r(gen);  

    // Linear acceleration
    msg_ax.data = ax_ + dist_ax(gen);  
    msg_ay.data = ay_ + dist_ay(gen);  

    // Publish the IMU message
    ax_pub_->publish(msg_ax);
    ay_pub_->publish(msg_ay);
    r_pub_->publish(msg_r);
}

/**
 * @brief Timer function for the wheel speed sensors
 * 
 */
void Sensors::inverter_timer()
{
    // Random noise generation with different noise for each wheel
    std::random_device rd; 
    std::mt19937 gen(rd());

    // ---------- Wheelspeed ------------
    std::normal_distribution<> dist_front_right(0.0, kNoiseWheelSpeedFrontRight);
    std::normal_distribution<> dist_front_left(0.0, kNoiseWheelSpeedFrontLeft);
    std::normal_distribution<> dist_rear_right(0.0, kNoiseWheelSpeedRearRight);
    std::normal_distribution<> dist_rear_left(0.0, kNoiseWheelSpeedRearLeft);

    // Apply noise to the state variables
    wheel_speed_.fr_ = wheel_speed_msg_.front_right + dist_front_right(gen);
    wheel_speed_.fl_ = wheel_speed_msg_.front_left + dist_front_left(gen);
    wheel_speed_.rr_ = wheel_speed_msg_.rear_right + dist_rear_right(gen);
    wheel_speed_.rl_ = wheel_speed_msg_.rear_left + dist_rear_left(gen);

    // Create the wheel speed message
    auto message = arussim_msgs::msg::FourWheelDrive();

    message.front_right = wheel_speed_.fr_;    
    message.front_left = wheel_speed_.fl_;
    message.rear_right = wheel_speed_.rr_;
    message.rear_left = wheel_speed_.rl_;

    // Publish the torque message
    ws_pub_->publish(message);

    // ---------- Torque ------------
    std::normal_distribution<> dist_fr(0.0, kNoiseTorqueFrontRight);
    std::normal_distribution<> dist_fl(0.0, kNoiseTorqueFrontLeft);
    std::normal_distribution<> dist_rr(0.0, kNoiseTorqueRearRight);
    std::normal_distribution<> dist_rl(0.0, kNoiseTorqueRearLeft);

    // Apply noise to the state variables
    torque_cmd_.fr_ = torque_cmd_msg_.front_right + dist_fr(gen);
    torque_cmd_.fl_ = torque_cmd_msg_.front_left + dist_fl(gen);
    torque_cmd_.rr_ = torque_cmd_msg_.rear_right + dist_rr(gen);
    torque_cmd_.rl_ = torque_cmd_msg_.rear_left + dist_rl(gen);

    // Create the torque message
    message.front_right = torque_cmd_.fr_;    
    message.front_left = torque_cmd_.fl_;      
    message.rear_right = torque_cmd_.rr_;     
    message.rear_left = torque_cmd_.rl_;     

    // Publish the torque message
    torque_pub_->publish(message);
}

/**
 * @brief Timer function for the groundspeed 
 * 
 */
void Sensors::groundspeed_timer()
{
    // Random noise generation
    std::random_device rd; 
    std::mt19937 gen(rd());
    std::normal_distribution<> dist_vx(0.0, kNoiseGssVx);
    std::normal_distribution<> dist_vy(0.0, kNoiseGssVy);

    auto msg_vx = std_msgs::msg::Float32();
    auto msg_vy = std_msgs::msg::Float32();

    msg_vx.data = vx_ + dist_vx(gen);
    msg_vy.data = vy_ + dist_vy(gen);

    gss_vx_pub_->publish(msg_vx);
    gss_vy_pub_->publish(msg_vy);
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


    auto message = std_msgs::msg::Float32();
    message.data = delta_ + dist(gen);
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