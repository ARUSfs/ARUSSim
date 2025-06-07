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
    // Declare and get noise parameters for each IMU variable
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

    // Declare 4WD torque parameters
    this->declare_parameter<double>("torque.noise_torque_front_right", 0.01);
    this->declare_parameter<double>("torque.noise_torque_front_left", 0.01);
    this->declare_parameter<double>("torque.noise_torque_rear_right", 0.01);
    this->declare_parameter<double>("torque.noise_torque_rear_left", 0.01);
    this->declare_parameter<double>("torque.torque_frequency", 100.0);
    
    // Declare extensometer parameters
    this->declare_parameter<double>("extensometer.extensometer_frequency", 100.0);
    this->declare_parameter<double>("extensometer.noise_extensometer", 0.01);


    // Get parameters
    this->get_parameter("imu.noise_imu_ax", kNoiseImuAx);
    this->get_parameter("imu.noise_imu_ay", kNoiseImuAy);
    this->get_parameter("imu.noise_imu_r", kNoiseImuR);
    this->get_parameter("imu.imu_frequency", kImuFrequency);

    this->get_parameter("wheel_speed.noise_wheel_speed_front_right", kNoiseWheelSpeedFrontRight);
    this->get_parameter("wheel_speed.noise_wheel_speed_front_left", kNoiseWheelSpeedFrontLeft);
    this->get_parameter("wheel_speed.noise_wheel_speed_rear_right", kNoiseWheelSpeedRearRight);
    this->get_parameter("wheel_speed.noise_wheel_speed_rear_left", kNoiseWheelSpeedRearLeft);
    this->get_parameter("wheel_speed.wheel_speed_frequency", kWheelSpeedFrequency);

    this->get_parameter("torque.noise_torque_front_right", kNoiseTorqueFrontRight);
    this->get_parameter("torque.noise_torque_front_left", kNoiseTorqueFrontLeft);
    this->get_parameter("torque.noise_torque_rear_right", kNoiseTorqueRearRight);
    this->get_parameter("torque.noise_torque_rear_left", kNoiseTorqueRearLeft);
    this->get_parameter("torque.torque_frequency", kTorqueFrequency);

    this->get_parameter("extensometer.extensometer_frequency", kExtensometerFrequency);
    this->get_parameter("extensometer.noise_extensometer", kNoiseExtensometer);


    // State subscriber
    state_sub_ = this->create_subscription<arussim_msgs::msg::State>(
        "/arussim/state", 1, 
        std::bind(&Sensors::state_callback, this, std::placeholders::_1)
    );

    // IMU
    ax_pub_ = this->create_publisher<std_msgs::msg::Float32>("/arussim/IMU/ax", 10);
    ay_pub_ = this->create_publisher<std_msgs::msg::Float32>("/arussim/IMU/ay", 10);
    r_pub_ = this->create_publisher<std_msgs::msg::Float32>("/arussim/IMU/yaw_rate", 10);

    imu_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kImuFrequency)),
        std::bind(&Sensors::imu_timer, this)
    );

    // Wheel speed
    ws_fr_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/arussim/fr_wheel_speed", 10);
    ws_fl_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/arussim/fl_wheel_speed", 10);
    ws_rr_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/arussim/rr_wheel_speed", 10);
    ws_rl_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/arussim/rl_wheel_speed", 10);
        
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

    // Torque cmd
    torque_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>("/arussim/torque4WD", 10);

    torque_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kTorqueFrequency)),
        std::bind(&Sensors::torque_cmd_timer, this)
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
    wheel_speed = msg->wheel_speeds;
    torque_cmd = msg->torque;
}

/**
 * @brief Timer function for the 4WD torque
 * 
 */
void Sensors::torque_cmd_timer(){
    // Random noise generation with different noise for each wheel speed
    std::random_device rd; 
    std::mt19937 gen(rd());

    // Corrected noise distributions
    std::normal_distribution<> dist_fr(0.0, kNoiseTorqueFrontRight);
    std::normal_distribution<> dist_fl(0.0, kNoiseTorqueFrontLeft);
    std::normal_distribution<> dist_rr(0.0, kNoiseTorqueRearRight);
    std::normal_distribution<> dist_rl(0.0, kNoiseTorqueRearLeft);

    // Apply noise to the state variables
    torque_cmd_.fr_ = torque_cmd.front_right + dist_fr(gen);
    torque_cmd_.fl_ = torque_cmd.front_left + dist_fl(gen);
    torque_cmd_.rr_ = torque_cmd.rear_right + dist_rr(gen);
    torque_cmd_.rl_ = torque_cmd.rear_left + dist_rl(gen);

    // Create the torque message
    auto message = arussim_msgs::msg::FourWheelDrive();

    message.front_right = torque_cmd_.fr_;    
    message.front_left = torque_cmd_.fl_;      
    message.rear_right = torque_cmd_.rr_;     
    message.rear_left = torque_cmd_.rl_;     

    // Publish the torque message
    torque_pub_->publish(message);
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
    wheel_speed_.fr_ = wheel_speed.front_right * 0.225 + dist_front_right(gen);
    wheel_speed_.fl_ = wheel_speed.front_left * 0.225 + dist_front_left(gen);
    wheel_speed_.rr_ = wheel_speed.rear_right * 0.225 + dist_rear_right(gen);
    wheel_speed_.rl_ = wheel_speed.rear_left * 0.225 + dist_rear_left(gen);

    // Create the wheel speed message
    auto msg_fr = std_msgs::msg::Float32();
    auto msg_fl = std_msgs::msg::Float32();
    auto msg_rr = std_msgs::msg::Float32();
    auto msg_rl = std_msgs::msg::Float32();

    msg_fr.data = wheel_speed_.fr_;    
    msg_fl.data = wheel_speed_.fl_;
    msg_rr.data = wheel_speed_.rr_;
    msg_rl.data = wheel_speed_.rl_;

    // Publish the torque message
    ws_fr_pub_->publish(msg_fr);
    ws_fl_pub_->publish(msg_fl);
    ws_rr_pub_->publish(msg_rr);
    ws_rl_pub_->publish(msg_rl);
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